#!/usr/bin/env python3
"""
record_cycle.py - trigger cycles on the SMA controller and auto-save the CSV.

The Arduino streams CSV over USB serial; it cannot write to your PC's disk on its
own. This host-side helper opens the serial port, optionally sends a command
(e.g. a `cycle ...`), captures the streamed rows, and writes them to a
timestamped file in analysis/logs/. It can also repeat on an interval for
unattended logging over time.

Examples
--------
  # One 5x heat/cool cycle, saved automatically:
  python record_cycle.py --port COM4 --cycle 3.5 100 0.5 2000 5

  # Repeat that cycle 20 times, 60 s apart (unattended):
  python record_cycle.py --port COM4 --cycle 3.5 100 0.5 2000 5 --repeat 20 --interval 60

  # Repeat forever until Ctrl-C:
  python record_cycle.py --port COM4 --cycle 3.5 100 0.5 2000 5 --repeat 0 --interval 300

  # Capture a calibration table instead:
  python record_cycle.py --port COM4 --command cal --until "# Calibration complete"

  # Just record whatever the board prints for 30 s (no command sent):
  python record_cycle.py --port COM4 --duration 30

Requires: pyserial  (pip install pyserial)
"""
import argparse
import datetime as dt
import os
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed. Run: pip install pyserial")

LOGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def capture_once(ser, command, stop_marker, duration, out_path):
    """Send one command (if any) and capture its output to out_path.
    Returns the number of lines written."""
    ser.reset_input_buffer()
    if command:
        print(f"> {command}")
        ser.write((command + "\n").encode())

    lines, t0 = [], time.time()
    while True:
        raw = ser.readline().decode(errors="ignore").rstrip("\r\n")
        if raw:
            print(raw)
            lines.append(raw)
            if (not duration) and stop_marker and stop_marker in raw:
                break
        if duration and (time.time() - t0) >= duration:
            break

    with open(out_path, "w", newline="") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  saved {len(lines)} lines -> {out_path}")
    return len(lines)


def main():
    ap = argparse.ArgumentParser(description="Record SMA controller cycles to CSV.")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM4 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cycle", nargs=5, metavar=("vH", "tH", "vC", "tC", "n"),
                    help="shortcut: sends `cycle vH tH vC tC n`")
    ap.add_argument("--command", help="raw command to send (e.g. 'cal', 'read')")
    ap.add_argument("--until", default="# END cycle",
                    help="stop a capture when a line contains this text (default: '# END cycle')")
    ap.add_argument("--duration", type=float, default=0,
                    help="instead of --until, capture for this many seconds per run")
    ap.add_argument("--repeat", type=int, default=1,
                    help="number of runs; 0 (or negative) = repeat forever until Ctrl-C")
    ap.add_argument("--interval", type=float, default=0,
                    help="seconds to wait between runs (default 0)")
    ap.add_argument("--outdir", default=LOGDIR, help="directory for saved CSVs")
    args = ap.parse_args()

    command = ("cycle " + " ".join(args.cycle)) if args.cycle else args.command
    tag = (command.split()[0] if command else "capture")
    os.makedirs(args.outdir, exist_ok=True)

    forever = args.repeat <= 0
    total = "infinite" if forever else args.repeat
    print(f"Opening {args.port} @ {args.baud} ... (runs: {total})")

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        time.sleep(2.0)            # allow the board to (re)boot after the port opens
        run = 0
        try:
            while forever or run < args.repeat:
                run += 1
                stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
                out = os.path.join(args.outdir, f"{tag}_{stamp}.csv")
                label = f"run {run}" + ("" if forever else f"/{args.repeat}")
                print(f"\n=== {label} @ {stamp} ===")
                capture_once(ser, command, args.until, args.duration, out)

                more = forever or run < args.repeat
                if more and args.interval > 0:
                    print(f"  waiting {args.interval:g}s ...")
                    time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\n(stopped by Ctrl-C)")

    print(f"\nDone. {run} run(s) saved in {args.outdir}")


if __name__ == "__main__":
    main()
