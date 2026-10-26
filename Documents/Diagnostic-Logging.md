# V10L1 Diagnostic Logging

This change adds observation, not a driving-mode or tuning change. It preserves
the 10 ms control period, route and parking logic, PID gains, direction signs,
speeds, safety decisions and motor PWM calculation. The UART log period changes
from 200 to 100 ms. No target HEX is rebuilt or flashed by these changes.

## Receive and Save

Use USART1 at **115200 baud, 8N1, no flow control**. Board TX is PA9 and RX is
PA10. Receiving needs board TX -> adapter RX and common GND. Use a 3.3 V logic
compatible USB-TTL adapter, not RS-232 voltage levels. Do not wire DTR/RTS to
RESET/BOOT. Close other software using the same port.

Python 3.9+ is sufficient. In the repository directory, replace COM5 with the
actual port:

```powershell
python -m pip install pyserial
python Tools/capture_log.py --port COM5 --output Logs/run-01 --seconds 60
```

The tool writes three new files, refusing to overwrite existing ones:

- `.log`: exact received bytes, including damaged or partial records.
- `.csv`: complete, CRC-checked data samples, with units converted.
- `.json`: configuration records, gaps, invalid data and suspected resets.

The tool never sends start/stop commands. **Ctrl+C or the duration limit stops
recording, not the car. Use KEY2 for the car's stop function.** Control-line
states are set before opening the port, but some drivers may still pulse them;
follow the wiring restriction above.

An existing serial-terminal text capture can be converted without pyserial or
opening a port:

```powershell
python Tools/capture_log.py --input serial-saved.txt --output Logs/converted-01
```

A capture with no valid V10L1 samples exits nonzero. Keep its raw log to diagnose
wrong firmware, baud rate or wiring. Board-reset counting is inference from
sequence discontinuities, not a guaranteed boot counter; normal uint32 sequence
rollover is not counted as a reset.

## Data and Units

Records start with `log=V10L1 type=data`, `type=cfg` or `type=overflow`.
Wheel vectors are always in **M1, M2, M3, M4** socket order. The configuration
record supplies their physical mapping and direction signs.

| Fields | Meaning |
| --- | --- |
| t, seq, state, fault | Uptime ms, packet sequence, application state and fault |
| cnt, denc | Four cumulative encoder counts and last-control-tick deltas |
| v10, target10, ramp10 | Measured, requested and acceleration-ramped speeds; divide by 10 for mm/s |
| vmin10, vmax10 | Speed extrema over all control ticks since the last successful data packet; divide by 10 |
| pwm | Logical controller outputs, before dead-zone and direction compensation |
| out | Eight commanded timer compare values: M1 A/B through M4 A/B |
| raw, line | Hex masks: raw bit0=X1; normalized line bit0=physical left |
| n, valid, pos10, marker | Active count, usable position flag, position x10, wide-marker flag |
| s, side, seg, goal, cmd, arm, stage | Travel, side/segment distances, goals, wheel commands, finish gate and route stage |
| imu, cal, imu_age | Status 0 offline/1 calibrating/2 ready/3 error; sample count and age ms |
| yaw100, rate100, bias1000 | Divide by 100 for heading degrees and degrees/s; bias by 1000 for degrees/s |
| ref10, off10 | Nominal heading and boundary offset x10; add them for the current heading target |
| dt, dtmin, dtmax, costmax, ctrl_n, ctrl_age | Last/min/max interval, max control-task duration, window tick count and age; times in ms |
| drop, txerr, badnum, crc | Device loss/error counters, invalid numerical values, checksum |

The CSV adds per-wheel ramped-target minus measured-speed error. `out` reports
software commands, not measured motor voltage/current. Duty fraction is compare
value / `pwm_period` (currently 3600). This firmware has no battery-voltage/current
sampling, so no such measurements are invented.

Configuration packets appear initially and every five seconds, replacing one
data slot. They include baud/log period, wheel diameter/CPR, motor/encoder signs,
wheel mapping, PWM limits/dead zone, speed gains/ramps, gray polarity/orientation,
IMU sign, heading gains/limits, trim and nominal speeds.

## Timing and Integrity

Logging is roughly 10 Hz, **not a complete 100 Hz waveform**. Every control tick
contributes speed extrema and interval statistics. Nonfinite speeds are counted
even if they recover before publication. Extrema use finite samples only;
nonfinite/oversized numbers are flagged and encoded as zero/saturated values.
`costmax` excludes subsequent log formatting and OLED service; their delays can
appear in later `dt` values. Brief states/faults cleared between log samples may
not appear as separate records.

The logger never waits for UART completion or overwrites its active interrupt
buffer. Frames are at most 1024 bytes (<89 ms at 115200/8N1). A busy UART or
failed send increments counters. Oversized formatting emits `type=overflow`
instead of a truncated normal sample. CRC-16/CCITT-FALSE uses initial 0xFFFF,
polynomial 0x1021, and covers bytes from `log=` up to but excluding ` crc=`.

During route arcs the original code suppresses gray sampling. `raw=0` in that
state is not a new all-white measurement. Idle/done route fields may retain the
last route state; interpret them together with the application state.

No Wi-Fi, onboard Flash/SD recording or hard-power-loss durability is added.
The computer must receive bytes while recording.

## First Capture and Verification

Start recording, power the stationary car, wait for CAL -> IDLE without KEY1,
leave it still for 10 seconds, manually turn right about 90 degrees, then leave
it still another 5 seconds. The CSV heading should increase by about 90 degrees.
With motors still off, move actual track material under left/center/right probes.
Analyze these results before arranging a low-speed drive; this change does not
lower the existing driving speeds.

Rebuild `MDK-ARM/STM32_D_Route.uvprojx` with Keil for target use. No new production
C file needs to be registered. Existing HEX files are not diagnostic builds.

Offline tests need host Clang/GCC and Python, not pyserial:

```powershell
python Tests/run_all.py --cc D:\mingw64\bin\gcc.exe
```

On supported macOS/Linux compilers: `python3 Tests/run_all.py --sanitize`.
Tests cover route, parking, logger framing/window/transport failures, commanded
motor outputs and capture parsing/lifecycle. Real target formatting/interrupt
latency, Keil linkage, electrical interfaces and vehicle motion remain unverified.
