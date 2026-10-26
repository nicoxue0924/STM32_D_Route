#!/usr/bin/env python3
"""Receive V10L1 telemetry, retain exact bytes and export checked CSV. Never sends commands."""
import argparse
import binascii
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import time


SCALARS = {
    "t": ("device_ms", 1), "seq": ("seq", 1), "fault": ("fault", 1),
    "n": ("active_count", 1), "valid": ("line_valid", 1),
    "pos10": ("line_position", 10), "marker": ("marker", 1),
    "s": ("travel_mm", 1), "arm": ("finish_armed", 1),
    "imu": ("imu_status", 1), "cal": ("calibration_samples", 1),
    "imu_age": ("imu_age_ms", 1), "bias1000": ("gyro_bias_deg_s", 1000),
    "yaw100": ("yaw_deg", 100), "rate100": ("rate_deg_s", 100),
    "ref10": ("heading_ref_deg", 10), "off10": ("boundary_offset_deg", 10),
    "dt": ("control_dt_ms", 1), "dtmin": ("control_dt_min_ms", 1),
    "dtmax": ("control_dt_max_ms", 1), "costmax": ("control_cost_max_ms", 1),
    "ctrl_n": ("control_ticks", 1), "ctrl_age": ("control_age_ms", 1),
    "drop": ("device_dropped", 1), "txerr": ("device_tx_errors", 1),
    "badnum": ("invalid_numeric_values", 1),
}
VECTORS = {
    "cnt": ([f"m{i}_count" for i in range(1, 5)], 1),
    "denc": ([f"m{i}_delta" for i in range(1, 5)], 1),
    "v10": ([f"m{i}_speed_mm_s" for i in range(1, 5)], 10),
    "target10": ([f"m{i}_target_mm_s" for i in range(1, 5)], 10),
    "ramp10": ([f"m{i}_ramped_mm_s" for i in range(1, 5)], 10),
    "vmin10": ([f"m{i}_window_min_mm_s" for i in range(1, 5)], 10),
    "vmax10": ([f"m{i}_window_max_mm_s" for i in range(1, 5)], 10),
    "pwm": ([f"m{i}_logical_pwm" for i in range(1, 5)], 1),
    "out": ([f"m{i}_out_{pin}" for i in range(1, 5) for pin in ("a", "b")], 1),
    "side": (["left_trip_mm", "right_trip_mm"], 1),
    "seg": (["left_segment_mm", "right_segment_mm"], 1),
    "goal": (["left_goal_mm", "right_goal_mm"], 1),
    "cmd": (["left_command_mm_s", "right_command_mm_s"], 1),
}
FIELDNAMES = ["host_utc", "epoch", "state", "stage", "raw_hex", "line_hex"]
FIELDNAMES += [name for name, _ in SCALARS.values()]
FIELDNAMES += [name for names, _ in VECTORS.values() for name in names]
FIELDNAMES += [f"m{i}_error_mm_s" for i in range(1, 5)]


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def parse_record(line):
    if not line.startswith(b"log=V10L1 "):
        return None
    try:
        payload, crc_text = line.rstrip(b"\r\n").rsplit(b" crc=", 1)
        if len(crc_text) != 4 or int(crc_text, 16) != binascii.crc_hqx(payload, 0xffff):
            raise ValueError("CRC mismatch")
        record = {}
        for token in payload.decode("ascii").split():
            key, value = token.split("=", 1)
            if key in record or not value:
                raise ValueError("Duplicate/empty field")
            record[key] = value
        if record["type"] not in ("cfg", "data", "overflow"):
            raise ValueError("Unknown record type")
        for key in ("t", "seq"):
            if not 0 <= int(record[key]) <= 0xffffffff:
                raise ValueError("Invalid 32-bit time/sequence")
        return record
    except (KeyError, UnicodeError, ValueError) as exc:
        raise ValueError(f"Invalid V10L1 record: {exc}") from exc


def to_csv_row(record, host_utc, epoch):
    try:
        row = {"host_utc": host_utc, "epoch": epoch,
               "state": record["state"], "stage": record["stage"],
               "raw_hex": record["raw"], "line_hex": record["line"]}
        if not all(0 <= int(record[key], 16) <= 255 for key in ("raw", "line")):
            raise ValueError("Invalid sensor bitmask")
        for key, (name, scale) in SCALARS.items():
            value = int(record[key])
            row[name] = value if scale == 1 else value / scale
        for key, (names, scale) in VECTORS.items():
            values = [int(value) for value in record[key].split(",")]
            if len(values) != len(names):
                raise ValueError(f"Wrong vector length: {key}")
            row.update(zip(names, (value if scale == 1 else value / scale for value in values)))
        for i in range(1, 5):
            row[f"m{i}_error_mm_s"] = row[f"m{i}_ramped_mm_s"] - row[f"m{i}_speed_mm_s"]
        return row
    except (KeyError, ValueError) as exc:
        raise ValueError(f"Incomplete/invalid data record: {exc}") from exc


class CaptureWriter:
    def __init__(self, prefix):
        prefix = Path(prefix)
        paths = {suffix: Path(str(prefix) + suffix) for suffix in (".log", ".csv", ".json")}
        for path in paths.values():
            if path.exists():
                raise FileExistsError(f"Refusing to overwrite {path}")
        prefix.parent.mkdir(parents=True, exist_ok=True)
        self.files = ExitStack()
        try:
            self.raw = self.files.enter_context(paths[".log"].open("xb"))
            self.table = self.files.enter_context(paths[".csv"].open("x", newline="", encoding="utf-8"))
            self.meta = self.files.enter_context(paths[".json"].open("x", encoding="utf-8"))
        except BaseException:
            self.files.close()
            raise
        self.csv = csv.DictWriter(self.table, FIELDNAMES)
        self.csv.writeheader()
        self.pending = bytearray()
        self.previous_sequence = None
        self.epoch = 0
        self.closed = False
        self.summary = dict(started_utc=utc_now(), bytes=0, data_rows=0,
                            invalid_records=0, unrecognized_lines=0, overflows=0,
                            sequence_gaps=0, board_resets=0, configs=[])

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def feed(self, chunk):
        self.raw.write(chunk)
        self.summary["bytes"] += len(chunk)
        self.pending.extend(chunk)
        while b"\n" in self.pending:
            line, _, rest = self.pending.partition(b"\n")
            self.pending = bytearray(rest)
            try:
                record = parse_record(bytes(line))
                if record is None:
                    self.summary["unrecognized_lines"] += 1
                    continue
                row = to_csv_row(record, utc_now(), self.epoch) if record["type"] == "data" else None
                sequence = int(record["seq"])
                if self.previous_sequence is not None:
                    delta = (sequence - self.previous_sequence) & 0xffffffff
                    if delta == 0 or delta > 0x80000000:
                        self.epoch += 1
                        self.summary["board_resets"] += 1
                    else:
                        self.summary["sequence_gaps"] += delta - 1
                self.previous_sequence = sequence
                if row is not None:
                    row["epoch"] = self.epoch
                    self.csv.writerow(row)
                    self.summary["data_rows"] += 1
                elif record["type"] == "cfg":
                    self.summary["configs"].append(dict(record, epoch=self.epoch, host_utc=utc_now()))
                else:
                    self.summary["overflows"] += 1
            except ValueError:
                self.summary["invalid_records"] += 1
        if len(self.pending) > 16384:
            self.summary["invalid_records"] += 1
            self.pending.clear()
        self.raw.flush()
        self.table.flush()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.summary.update(ended_utc=utc_now(), partial_bytes=len(self.pending))
        try:
            json.dump(self.summary, self.meta, indent=2)
            self.meta.write("\n")
        finally:
            self.files.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Serial port, e.g. COM5 or /dev/cu.usbserial-...")
    source.add_argument("--input", type=Path, help="Convert an existing raw log, without opening a serial port")
    parser.add_argument("--output", type=Path, required=True, help="New output prefix; creates .log, .csv, .json")
    parser.add_argument("--seconds", type=float, default=0, help="Capture duration; 0 means until Ctrl+C")
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error("--seconds must be nonnegative")
    if args.input:
        with args.input.open("rb") as stream, CaptureWriter(args.output) as output:
            for chunk in iter(lambda: stream.read(4096), b""):
                output.feed(chunk)
    else:
        try:
            import serial
        except ImportError:
            parser.error("Install pyserial first: python -m pip install pyserial")
        # Set control-line states before opening; no data/commands are transmitted.
        # Some USB drivers may still pulse these lines: do not wire them to reset/boot.
        with CaptureWriter(args.output) as output:
            port = serial.Serial(port=None)
            port.baudrate = 115200
            port.bytesize, port.parity, port.stopbits = 8, "N", 1
            port.xonxoff = port.rtscts = port.dsrdtr = False
            port.timeout = 0.2
            port.dtr = False
            port.rts = False
            port.port = args.port
            port.open()
            with port:
                print("Recording receive-only telemetry. Ctrl+C stops recording; it does NOT stop the car.")
                start = time.monotonic()
                try:
                    while not args.seconds or time.monotonic() - start < args.seconds:
                        output.feed(port.read(4096))
                except KeyboardInterrupt:
                    pass
    print(json.dumps({key: value for key, value in output.summary.items() if key != "configs"}, indent=2))
    if not output.summary["data_rows"]:
        parser.exit(2, "No valid V10L1 samples. Keep the raw log; check firmware, baud rate and wiring.\n")


if __name__ == "__main__":
    main()
