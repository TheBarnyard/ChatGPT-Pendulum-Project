#!/usr/bin/env python3
"""Cart motor stress test: finds the acceleration / speed at which the cart starts to jolt, skip,
or trip the UIM344 red LED, with no pendulum control in the loop.

Each trial is a short cart move with a known acceleration and top speed. After each one the
script asks you three things (red LED? jolt? how far is the cart from the tape mark?) and saves
them, so two runs (before / after a change) can be compared line by line.

How to read the answers:
  red LED on          -> the motor itself fell behind (UIM344 position-error lockout): torque,
                         current, voltage or too-sudden commands.
  cart off the mark,  -> the motor turned the right amount but the cart did not follow:
  no red LED             belt teeth skipping or the pulley slipping on the motor shaft.
  clean               -> the cart can do this; the swing-up may ask for it.

Safety: arms OFF (or tied up) for the first run. Cart centred on a tape mark. Hands off the rail.
Moves stay within +/-0.25 m of the start (track is about +/-0.54 m).

Usage (on the Pi):
  $HOME/PythonLibraries/bin/python motor_stress_test.py --list
  $HOME/PythonLibraries/bin/python motor_stress_test.py --label "baseline arms off"
  ... --start 7          start at trial 7 (after a lockout, for example)
  ... --only 12          run one trial
  ... --auto             no questions (just watch / film)
"""
import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

from cart_link import Cart, T, trapezoid, peak_accel

HERE = Path(__file__).resolve().parent
D = 0.20          # move length each way, m


def out_and_back(a, v, d=D):
    """+d, -2d, +d: both directions, two hard reversals, ends where it started."""
    return trapezoid(d, v, a) + trapezoid(-2 * d, v, a) + trapezoid(d, v, a)


def ramp(v0, v1, a):
    """Speeds going from v0 to v1 at acceleration a (one per 10 ms, v1 included)."""
    out, v = [], v0
    step = math.copysign(a * T, v1 - v0)
    while abs(v1 - v) > abs(step):
        v += step
        out.append(v)
    out.append(v1)
    return out


def reversal(v, a_up, a_brake):
    """Speed up to v, then brake hard through zero to -v/2 (the 'braking torque' case), stop,
    and come back gently to the start."""
    seq = ramp(0.0, v, a_up) + ramp(v, -0.5 * v, a_brake) + ramp(-0.5 * v, 0.0, 4.0)
    x = sum(seq) * T
    return seq + trapezoid(-x, 0.15, 1.5)


def jump(dv, hold_s=0.15):
    """Speed jumps from 0 to dv in ONE 10 ms cycle (what the Mega does every cycle), then gently back."""
    n = int(hold_s / T)
    seq = [dv] * n + [0.0]
    x = dv * n * T
    return seq + trapezoid(-x, 0.15, 1.5)


def shake(amp, freq=1.2, cycles=3):
    """Cart position amp*(1 - cos wt) at about the arms' swing frequency (starts and ends at rest)."""
    n = int(round(cycles / freq / T))
    w = 2 * math.pi * freq
    return [amp * w * math.sin(w * (k + 0.5) * T) for k in range(n)] + [0.0]


def trials():
    t = []
    for a, v in [(2, 0.3), (4, 0.4), (6, 0.5), (8, 0.6), (10, 0.7), (12, 0.75), (16, 0.75), (16, 0.85), (20, 0.85)]:
        t.append((f"move a={a} v={v}", a, v, out_and_back(a, v)))
    for v, a_up, a_br in [(0.6, 6, 12), (0.75, 8, 16), (0.75, 8, 24)]:
        t.append((f"reverse v={v} brake={a_br}", a_br, v, reversal(v, a_up, a_br)))
    for dv in (0.08, 0.16, 0.25):
        t.append((f"jump dv={dv}", dv / T, dv, jump(dv)))
    for amp in (0.03, 0.06, 0.09):
        w = 2 * math.pi * 1.2
        t.append((f"shake 1.2Hz amp={amp}", amp * w * w, amp * w, shake(amp)))
    return t


def ask(prompt, default=""):
    try:
        s = input(prompt).strip()
    except EOFError:
        s = ""
    return s or default


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--label", default="", help="what is different this run, e.g. 'I_work 2.5A arms off'")
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--only", type=int)
    ap.add_argument("--auto", action="store_true", help="no questions after each trial")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    tr = trials()
    if args.list:
        for i, (name, a, v, seq) in enumerate(tr, 1):
            x = [0.0]
            for s in seq:
                x.append(x[-1] + s * T)
            print(f"{i:2d}  {name:28s} peak accel {peak_accel(seq):5.1f} m/s^2  peak speed {max(map(abs, seq)):.2f} m/s  "
                  f"travel {min(x)*1000:+5.0f}..{max(x)*1000:+5.0f} mm  {len(seq)*T:4.1f} s")
        return

    pick = [args.only] if args.only else list(range(args.start, len(tr) + 1))
    out_dir = HERE / "results"
    out_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = "".join(c if c.isalnum() else "_" for c in args.label)[:40]
    summary_path = out_dir / f"stress_{stamp}_{tag}.csv"
    cycles_path = out_dir / f"stress_{stamp}_{tag}_cycles.csv"

    print("Arms OFF or tied up. Cart centred on a tape mark. Motor power ON. Hands off the rail.")
    ask("Press Enter to start (Ctrl+C stops at any time): ")
    cart = Cart(args.port).open()
    print(f"Mega replied, step count {cart.steps0}. Results -> {summary_path.name}")

    with open(summary_path, "w", newline="") as fs:
        w = csv.writer(fs)
        w.writerow(["time", "label", "trial", "name", "peak_accel_m_s2", "peak_speed_m_s",
                    "red_led", "jolt", "mark_offset_mm", "notes"])
        try:
            for i in pick:
                name, a, v, seq = tr[i - 1]
                print(f"\n[{i}/{len(tr)}] {name}: peak {peak_accel(seq):.1f} m/s^2, {max(map(abs, seq)):.2f} m/s "
                      f"(t = {time.monotonic() - cart.t0:.1f} s on the clock)")
                time.sleep(0.5)
                cart.run_speeds(seq, f"T{i}")
                cart.hold(0.4, f"T{i}")
                cart.go_to(0.0)
                if args.auto:
                    red = jolt = off = notes = ""
                    time.sleep(1.0)
                else:
                    red = ask("  Red LED came on? [y/N]: ", "n").lower()[:1]
                    jolt = ask("  Jolt / clunk / grinding / buzz? [y/N]: ", "n").lower()[:1]
                    off = ask("  Cart offset from the tape mark, mm (Enter = 0): ", "0")
                    notes = ask("  Notes (Enter = none, q = stop after this): ", "")
                w.writerow([datetime.now().isoformat(timespec="seconds"), args.label, i, name,
                            f"{peak_accel(seq):.2f}", f"{max(map(abs, seq)):.3f}", red, jolt, off, notes])
                fs.flush()
                if red == "y":
                    cart.close(motor_off=True)
                    print("  Motor is OFF. Slide the cart back to the tape mark by hand.")
                    ask("  Press Enter to re-enable the motor (this clears the lockout) and go on: ")
                    cart.reenable()
                elif off not in ("", "0") and not args.auto:
                    print("  Motor still on. Note: the count says the cart is at the start, the mark says not.")
                    print("  To re-zero: Ctrl+C now, slide the cart to the mark, run again with --start", i + 1)
                if notes.lower() == "q":
                    break
        except KeyboardInterrupt:
            print("\nStopped.")
        except Exception as exc:
            print(f"\nFAULT: {type(exc).__name__}: {exc}")
        finally:
            try:
                cart.close(motor_off=True)
            except Exception:
                pass
    with open(cycles_path, "w", newline="") as fc:
        w = csv.writer(fc)
        w.writerow(["t_s", "timer", "steps", "v_cmd_m_s", "trial"])
        w.writerows(cart.log)
    print(f"Saved {summary_path} and {cycles_path.name}")


if __name__ == "__main__":
    sys.exit(main())
