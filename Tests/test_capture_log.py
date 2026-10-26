import binascii
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "Tools" / "capture_log.py"
SPEC = importlib.util.spec_from_file_location("capture_log", SCRIPT)
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)


def wire(payload):
    data = payload.encode("ascii")
    return data + f" crc={binascii.crc_hqx(data, 0xffff):04X}\r\n".encode("ascii")


DATA = (
    "log=V10L1 type=data t=100 seq=1 state=IDLE fault=0 raw=01 line=80 "
    "n=1 valid=1 pos10=35 marker=0 s=0 cnt=123,0,0,0 denc=8,0,0,0 "
    "v=100,0,0,0 v10=1003,0,0,0 target10=4020,4020,3980,3980 "
    "ramp10=1005,1005,1005,1005 vmin10=900,0,0,0 vmax10=1100,0,0,0 "
    "pwm=10,20,30,40 out=0,2010,0,2020,2030,0,2040,0 "
    "stage=S1 side=0,0 seg=0,0 goal=944,944 cmd=402,398 arm=0 "
    "imu=2 cal=300 imu_age=0 bias1000=1250 yaw10=0 rate10=0 yaw100=12 rate100=-3 "
    "ref10=0 off10=0 dt=10 dtmin=10 dtmax=12 costmax=4 ctrl_n=10 ctrl_age=0 "
    "drop=0 txerr=0 badnum=0"
)
CONFIG = "log=V10L1 type=cfg t=0 seq=0 baud=115200 period=100"


class CaptureTests(unittest.TestCase):
    def test_scaled_wheels_and_heading_become_numeric_csv_columns(self):
        row = capture.to_csv_row(capture.parse_record(wire(DATA)), "fixture", 0)
        self.assertEqual(row["m1_speed_mm_s"], 100.3)
        self.assertEqual(row["m1_target_mm_s"], 402)
        self.assertAlmostEqual(row["m1_error_mm_s"], 0.2)
        self.assertEqual(row["m1_out_b"], 2010)
        self.assertEqual(row["yaw_deg"], 0.12)
        self.assertEqual(row["rate_deg_s"], -0.03)
        self.assertEqual(row["gyro_bias_deg_s"], 1.25)

    def test_corrupted_duplicate_and_incomplete_records_are_rejected(self):
        corrupted = wire(DATA).replace(b"seq=1", b"seq=2")
        for record in (corrupted, wire(DATA + " seq=1"), wire(DATA.replace("cnt=123,0,0,0", "cnt=1,2")), wire("log=V10L1 type=data t=1 seq=2")):
            with self.subTest(record=record[:80]):
                with self.assertRaises(ValueError):
                    capture.to_csv_row(capture.parse_record(record), "fixture", 0)

    def test_fragmented_input_is_preserved_and_gaps_and_resets_are_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            prefix = Path(tmp) / "run"
            blob = wire(CONFIG) + wire(DATA) + wire(DATA.replace("seq=1", "seq=4"))
            blob += wire(CONFIG) + wire(DATA)
            with capture.CaptureWriter(prefix) as output:
                for start in range(0, len(blob), 17):
                    output.feed(blob[start:start + 17])
            self.assertEqual(prefix.with_suffix(".log").read_bytes(), blob)
            summary = json.loads(prefix.with_suffix(".json").read_text())
            self.assertEqual(summary["sequence_gaps"], 2)
            self.assertEqual(summary["board_resets"], 1)
            self.assertEqual(summary["data_rows"], 3)
            with prefix.with_suffix(".csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["epoch"] for row in rows], ["0", "0", "1"])

    def test_bad_and_partial_records_remain_in_raw_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            prefix = Path(tmp) / "run"
            blob = b"old firmware text\r\n" + wire(DATA).replace(b"seq=1", b"seq=2") + b"partial"
            with capture.CaptureWriter(prefix) as output:
                output.feed(blob)
            summary = json.loads(prefix.with_suffix(".json").read_text())
            self.assertEqual(prefix.with_suffix(".log").read_bytes(), blob)
            self.assertEqual(summary["invalid_records"], 1)
            self.assertEqual(summary["unrecognized_lines"], 1)
            self.assertEqual(summary["partial_bytes"], 7)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            prefix = Path(tmp) / "run"
            prefix.with_suffix(".csv").write_text("keep me")
            with self.assertRaises(FileExistsError):
                capture.CaptureWriter(prefix)
            self.assertEqual(prefix.with_suffix(".csv").read_text(), "keep me")

    def test_sequence_wrap_is_not_misclassified_as_reboot(self):
        with tempfile.TemporaryDirectory() as tmp:
            prefix = Path(tmp) / "run"
            first = CONFIG.replace("seq=0", "seq=4294967295").replace("t=0", "t=4294967295")
            second = DATA.replace("seq=1", "seq=0").replace("t=100", "t=0")
            with capture.CaptureWriter(prefix) as output:
                output.feed(wire(first) + wire(second))
            summary = json.loads(prefix.with_suffix(".json").read_text())
            self.assertEqual(summary["board_resets"], 0)
            self.assertEqual(summary["sequence_gaps"], 0)

    def test_serial_capture_configures_before_open_and_never_sends_commands(self):
        class ReceiveOnlySerial:
            def __init__(self, port=None):
                self.port = port
                self.is_open = False

            def open(self):
                self.assert_configured()
                self.is_open = True

            def assert_configured(self):
                if self.port != "TEST" or self.dtr or self.rts or self.baudrate != 115200:
                    raise AssertionError("Serial must be configured before open")

            def __enter__(self):
                if not self.is_open:
                    self.open()
                return self

            def __exit__(self, *_):
                self.is_open = False

            def read(self, _):
                return wire(CONFIG) + wire(DATA)

            def write(self, _):
                raise AssertionError("Capture must never transmit commands")

        with tempfile.TemporaryDirectory() as tmp:
            prefix = Path(tmp) / "run"
            argv = ["capture_log.py", "--port", "TEST", "--output", str(prefix), "--seconds", "0.1"]
            with patch("sys.argv", argv), patch.dict("sys.modules", {"serial": SimpleNamespace(Serial=ReceiveOnlySerial)}):
                with patch.object(capture.time, "monotonic", side_effect=[0, 0, 1]):
                    capture.main()
            self.assertEqual(json.loads(prefix.with_suffix(".json").read_text())["data_rows"], 1)


if __name__ == "__main__":
    unittest.main()
