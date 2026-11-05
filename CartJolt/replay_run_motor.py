#!/usr/bin/env python3
"""Replay the exact motor commands of a logged swing-up run, with no controller in the loop.

The run CSVs log every timer value sent to the Mega (motor_timer_command). Sending the same
sequence again, with the arms OFF, separates the causes:
  - jolts / red LED also with arms off  -> the motor + driver cannot do the commanded profile
                                           (config, current, supply, or too-sudden commands);
  - only with arms on                   -> the arms' reaction forces are the extra load;
  - never on replay, only in real runs  -> look at the controller / radio (code side).

Usage (on the Pi):
  $HOME/PythonLibraries/bin/python replay_run_motor.py RUN.csv --info
        lists the swing attempts (start/end time, peak speed and acceleration, biggest jumps)
  $HOME/PythonLibraries/bin/python replay_run_motor.py RUN.csv --attempt 2
        replays attempt 2 (cart centred on the tape mark, arms off)
  ... --from-s 41.2 --to-s 45.0      replay a time window instead
  ... --speed-scale 0.7              same timing, 70 % of the speed (and acceleration)
  ... --label "arms off, I_work 2.5A"

Film it with a phone in slow motion with the terminal in view: the script prints the run's own
time stamps every 0.25 s so a jolt in the video can be matched to the log.
"""
import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

from cart_link import Cart, T, M_PER_STEP, speed_of_timer, timer_for_speed, trapezoid, HOLD

HERE = Path(__file__).resolve().parent
TRACK_LIMIT_M = 0.40


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r.get("record_type") != "sample":
                continue
            try:
                t = float(r["elapsed_s"])
                n = int(float(r["motor_timer_command"]))
            except (KeyError, ValueError):
                continue
            mode = ""
            msg = r.get("message", "")
            if "mode=" in msg:
                mode = msg.split("mode=")[1].split(";")[0].split()[0]
            rows.append((t, n, mode))
    if not rows:
        sys.exit("No sample rows with motor_timer_command found in this CSV.")
    return rows


def attempts(rows):
    """Contiguous SWING stretches (from the message column); falls back to moving stretches."""
    out, start = [], None
    for i, (t, n, mode) in enumerate(rows):
        sw = mode == "SWING"
        if sw and start is None:
            start = i
        if not sw and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(rows)))
    return out


def stats(seg):
    v = [speed_of_timer(n) for _, n, _ in seg]
    acc = [(b - a) / T for a, b in zip([0.0] + v, v)]
    x, xs = 0.0, []
    for s in v:
        x += s * T
        xs.append(x)
    big = sorted(range(len(acc)), key=lambda k: -abs(acc[k]))[:3]
    return v, acc, xs, big


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--attempt", type=int)
    ap.add_argument("--from-s", type=float)
    ap.add_argument("--to-s", type=float)
    ap.add_argument("--lead-s", type=float, default=0.3, help="seconds of run before the attempt to include")
    ap.add_argument("--speed-scale", type=float, default=1.0)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    rows = load(args.csv)
    att = attempts(rows)
    if args.info or (args.attempt is None and args.from_s is None):
        print(f"{len(rows)} samples, {rows[0][0]:.2f}..{rows[-1][0]:.2f} s, {len(att)} swing attempts")
        for k, (a, b) in enumerate(att, 1):
            v, acc, xs, big = stats(rows[a:b])
            jumps = ", ".join(f"t={rows[a + j][0]:.2f}s {acc[j]:+.0f}" for j in big)
            print(f" attempt {k}: t {rows[a][0]:7.2f}..{rows[b - 1][0]:7.2f} s ({b - a} cycles)  peak "
                  f"{max(map(abs, v)):.2f} m/s, {max(map(abs, acc)):.1f} m/s^2, travel {min(xs)*1000:+.0f}.."
                  f"{max(xs)*1000:+.0f} mm; biggest accel steps (m/s^2): {jumps}")
        if args.attempt is None and args.from_s is None:
            print("\nPick one with --attempt N (or --from-s/--to-s).")
        return

    if args.attempt is not None:
        a, b = att[args.attempt - 1]
        t0 = rows[a][0] - args.lead_s
        t1 = rows[b - 1][0] + 0.3
    else:
        t0, t1 = args.from_s, args.to_s if args.to_s is not None else rows[-1][0]
    seg = [r for r in rows if t0 <= r[0] <= t1]
    v = [speed_of_timer(n) * args.speed_scale for _, n, _ in seg]
    xs, x = [], 0.0
    for s in v:
        x += s * T
        xs.append(x)
    print(f"Replaying {len(seg)} cycles, run time {seg[0][0]:.2f}..{seg[-1][0]:.2f} s, speed x{args.speed_scale}: "
          f"peak {max(map(abs, v)):.2f} m/s, travel {min(xs)*1000:+.0f}..{max(xs)*1000:+.0f} mm from the start")
    if max(map(abs, xs)) > TRACK_LIMIT_M:
        sys.exit(f"Travel exceeds +/-{TRACK_LIMIT_M} m; pick a shorter window or a smaller --speed-scale.")
    end_v = v[-1]
    print("Arms OFF (first time). Cart centred on the tape mark. Motor power ON. Start the phone video.")
    input("Press Enter to replay (Ctrl+C stops): ")

    cart = Cart(args.port).open()
    log = []
    try:
        for i in (3, 2, 1):
            print(f"  {i}", flush=True)
            cart.hold(1.0)
        next_t = time.monotonic()
        next_print = 0.0
        for (t, n, mode), s in zip(seg, v):
            cmd = timer_for_speed(s) if args.speed_scale != 1.0 else n
            if cmd is None or (args.speed_scale == 1.0 and abs(n) >= HOLD):
                cmd = n if abs(n) >= HOLD else -HOLD
            cart.send(cmd)
            log.append((t, cmd, cart.steps, s))
            if t >= next_print:
                print(f"  run t = {t:7.2f} s  v = {s:+.2f} m/s  {mode}", flush=True)
                next_print = t + 0.25
            next_t += T
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
        # brake gently from the last speed, then go back to the start
        brake = []
        s = end_v
        while abs(s) > 0.005:
            s -= math.copysign(min(abs(s), 4.0 * T), s)
            brake.append(s)
        cart.run_speeds(brake, "brake")
        cart.hold(0.5)
        cart.go_to(0.0)
        cart.hold(0.5)
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        cart.close(motor_off=True)
    red = input("Red LED came on? [y/N]: ").strip().lower()[:1] or "n"
    jolt = input("Jolt / clunk? If yes, at which run t (from the screen or video)?: ").strip()
    off = input("Cart offset from the tape mark, mm (Enter = 0): ").strip() or "0"
    out = HERE / "results"
    out.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    with open(out / "replays.csv", "a", newline="") as f:
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(["time", "run_csv", "attempt", "from_s", "to_s", "speed_scale", "label", "red_led", "jolt", "mark_offset_mm"])
        w.writerow([stamp, Path(args.csv).name, args.attempt or "", f"{seg[0][0]:.2f}", f"{seg[-1][0]:.2f}",
                    args.speed_scale, args.label, red, jolt, off])
    with open(out / f"replay_{stamp}_cycles.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["run_t_s", "timer", "steps", "v_cmd_m_s"])
        w.writerows(log)
    print(f"Saved results/replays.csv and results/replay_{stamp}_cycles.csv")


if __name__ == "__main__":
    main()
