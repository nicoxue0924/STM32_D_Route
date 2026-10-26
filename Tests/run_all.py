#!/usr/bin/env python3
"""Run offline tests only. Does not open serial ports or build/flash target firmware."""
import argparse
import csv
import io
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=shutil.which("clang") or shutil.which("gcc"), help="Host Clang or GCC executable")
    parser.add_argument("--sanitize", action="store_true", help="Enable address/undefined sanitizers where supported")
    args = parser.parse_args()
    if not args.cc:
        parser.error("Host Clang/GCC not found; specify --cc")
    flags = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic"]
    if args.sanitize:
        flags.append("-fsanitize=address,undefined")
    common = [f"-I{ROOT / 'Tests/host'}", f"-I{ROOT / 'App/Inc'}"]
    route_sources = ["Tests/test_route.c"] + [f"App/Src/{name}.c" for name in
        ("route", "odometry", "crossline_stop", "app_run", "chassis", "speed_control", "pid", "hmi", "imu")]
    tests = {
        "route": (route_sources, []),
        "parking": (["Tests/test_parking.c", "App/Src/parking.c"], []),
        "debug_uart": (["Tests/test_debug_uart.c", "App/Src/debug_uart.c"], []),
        "motor_outputs": (["Tests/test_motor_outputs.c", "App/Src/motor.c"], [f"-I{ROOT / 'Tests/motor_host'}"]),
    }
    with tempfile.TemporaryDirectory(prefix="stm32-logging-tests-") as tmp:
        executables = {}
        for name, (sources, includes) in tests.items():
            executable = Path(tmp) / (name + (".exe" if sys.platform == "win32" else ""))
            subprocess.run([args.cc, *flags, *includes, *common,
                            *(str(ROOT / source) for source in sources), "-lm", "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
            executables[name] = executable
        subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", str(ROOT / "Tests"),
                        "-p", "test_capture_log.py", "-v"], check=True)
        wire = subprocess.check_output([str(executables["debug_uart"]), "--emit"])
        raw = Path(tmp) / "fixture.log"
        raw.write_bytes(wire)
        prefix = Path(tmp) / "converted"
        subprocess.run([sys.executable, str(ROOT / "Tools/capture_log.py"), "--input", str(raw),
                        "--output", str(prefix)], check=True)
        rows = list(csv.DictReader(io.StringIO(prefix.with_suffix(".csv").read_text())))
        assert len(rows) == 1 and rows[0]["m1_out_b"] == "2010"
        assert prefix.with_suffix(".log").read_bytes() == wire
        assert max(map(len, wire.splitlines(keepends=True))) <= 1024
        print("PASS: actual C wire output -> CRC-checked CSV/raw recorder")
    print("PASS: complete offline suite; no target linking or hardware validation")


if __name__ == "__main__":
    main()
