#!/usr/bin/env python3
"""Two-load stall test: estimates the moving mass of the cart and the motor's spare force, arms OFF.

Run it twice at the same speed: once with the bare cart, once with a weighed item strapped to the
cart (--added-kg). Each run finds the acceleration at which the motor first stalls during a
speed reversal. The motor's force at that speed is the same in both runs, so

    F = m * a1 + friction = (m + dm) * a2 + friction   ->   m = dm * a2 / (a1 - a2)

where m is the EFFECTIVE moving mass (cart + belt clamp + pulleys + about 0.05 kg from the motor
rotor) and m * a1 is the force the motor can give beyond friction at that speed.

Each trial: move gently (4 m/s^2) to a start speed, then speed up at the test acceleration for
60 ms so the cart reaches +0.8 m/s exactly at the end, then slow down gently and return to the
start. At high accelerations the start speed is negative (a reversal, up to -1.2 m/s) and the
hard phase gets shorter. The hard moment, speeding up near full speed, is what stalls the
swing-up, and it is the same in both runs. Travel stays within about 0.2 m of the start.

After each trial you answer one question:
    Enter = clean
    r     = red LED came on            (motor locked: counts as a stall)
    b     = buzz / grind / loud clunk   (motor slipped: counts as a stall)
    o     = cart ended off the tape mark (counts as a stall)
    q     = stop the run
Use the same judgement in both runs. After any stall the motor is switched off: slide the cart
back to the tape mark by hand, press Enter, and the motor comes back on (that clears the lockout).

The search: climb a coarse ladder until the first stall, then halve the gap between the last
clean and the first stalled acceleration until it is under 1 m/s^2, then re-check both ends.

Safety: arms OFF. Strap the added weight down hard (zip ties / tape): at 30 m/s^2 a 1 kg item
pushes with 30 N. Cart centred on a tape mark, at least 0.3 m from each limit switch. Hands off
the rail. Ctrl+C stops at any time (motor off).

Usage (on the Pi, in ~/Desktop/CartJolt):
  PY=$HOME/PythonLibraries/bin/python
  $PY two_load_stall_test.py --list                                  # show the ladder, no motion
  $PY two_load_stall_test.py --added-kg 0     --label "bare cart"     # run 1
  $PY two_load_stall_test.py --added-kg 1.234 --label "cart + 1.234 kg"   # run 2 (your weighed item)
  $PY two_load_stall_test.py --compute                               # mass + force from the two runs
Options: --end-speed 0.8 (m/s, both runs must match), --start-accel 6, --max-accel 40.
"""
import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

from cart_link import Cart, T, trapezoid, timer_for_speed, speed_of_timer

HERE = Path(__file__).resolve().parent
OUT = HERE / "results"
A_GENTLE = 4.0                      # m/s^2, the gentle speed-up and slow-down
COARSE = [6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 28, 31, 35, 40, 45]
HARD_CYCLES = 6                     # length of the hard phase, 10 ms cycles (shorter if the start speed would pass -max)
RESOLUTION = 1.0                    # m/s^2, stop halving below this gap
ROTOR_EQ_KG = 3e-5 / 0.0254 ** 2    # UIM5756PM rotor 300 g cm^2 at r 25.4 mm [assumed r]
R_PULLEY_M = 0.0254                 # 2.00 in pitch diameter, inferred from 0.638175 m / 6400 steps [?]


def ramp(v0, v1, a):
    """Speeds from v0 to v1 at acceleration a, one per 10 ms, v1 included."""
    out, v = [], v0
    step = math.copysign(a * T, v1 - v0)
    while abs(v1 - v) > abs(step) + 1e-12:
        v += step
        out.append(v)
    out.append(v1)
    return out


def quantize(v):
    return speed_of_timer(timer_for_speed(v))


def hard_profile(v_end, a, v_max):
    """Gentle to the start speed, HARD_CYCLES (or fewer) steps of a*T up to v_end, gentle to 0.
    Returns (speeds, hard slice)."""
    n = HARD_CYCLES
    while n > 2 and v_end - n * a * T < -v_max:
        n -= 1
    v0 = v_end - n * a * T
    up = ramp(0.0, v0, A_GENTLE) if abs(v0) > 1e-9 else [0.0]
    hard = [v0 + (k + 1) * a * T for k in range(n)]
    down = ramp(v_end, 0.0, A_GENTLE)
    seq = up + hard + down
    return seq, slice(len(up), len(up) + len(hard))


def describe(v_end, a, v_max):
    seq, hs = hard_profile(v_end, a, v_max)
    q = [quantize(x) for x in seq]
    acc = [abs(b - c) / T for c, b in zip([0.0] + q[:-1], q)]
    peak = max(acc[hs.start:hs.stop])
    n = hs.stop - hs.start
    mean = (q[hs.stop - 1] - q[hs.start - 1]) / (n * T)     # from the quantized speeds actually sent
    x, xmin, xmax = 0.0, 0.0, 0.0
    for sp in q:
        x += sp * T
        xmin, xmax = min(xmin, x), max(xmax, x)
    return dict(top=abs(q[hs.stop - 1]), v0=q[hs.start - 1], n=n, peak=peak, mean=mean,
                xmin=xmin, xmax=xmax, dur=len(seq) * T,
                text=f"{q[hs.start - 1]:+.2f} -> {q[hs.stop - 1]:+.2f} m/s, mean {mean:.1f}")


def ask(prompt, default=""):
    try:
        s = input(prompt).strip()
    except EOFError:
        s = ""
    return s or default


class Search:
    """Coarse climb, then bisection, then a re-check of both ends."""

    def __init__(self, start, cap):
        self.ladder = [a for a in COARSE if start <= a <= cap]
        self.i = 0
        self.lo = None      # highest clean
        self.hi = None      # lowest stalled
        self.phase = "climb"
        self.checks = []
        self.done = False
        self.note = ""

    def next(self):
        if self.phase == "climb":
            return self.ladder[self.i]
        if self.phase == "bisect":
            return round((self.lo + self.hi) / 2, 2)
        return self.checks[0]

    def report(self, a, stalled):
        if self.phase == "climb":
            if stalled:
                self.hi = a
                if self.i == 0:
                    self.done, self.note = True, f"stalled at the first step ({a} m/s^2): rerun with a lower --start-accel"
                    return
                self.lo = self.ladder[self.i - 1]
                self.phase = "bisect"
            else:
                self.lo = a
                self.i += 1
                if self.i >= len(self.ladder):
                    self.done, self.note = True, f"no stall up to {a} m/s^2 (raise --max-accel to go on)"
                    return
        elif self.phase == "bisect":
            if stalled:
                self.hi = a
            else:
                self.lo = a
        else:   # check
            self.checks.pop(0)
            if a == self.lo and stalled:            # the "clean" end stalled: move the bracket down
                self.hi = self.lo
                self.lo = max(self.lo - 2 * RESOLUTION, 0.5)
                self.phase = "bisect"
            elif a == self.hi and not stalled:      # the "stalled" end ran clean: move it up
                self.lo = self.hi
                self.hi = self.hi + 2 * RESOLUTION
                self.phase = "bisect"
            elif not self.checks:
                self.done = True
                return
        if self.phase == "bisect" and self.hi - self.lo <= RESOLUTION + 1e-9:
            self.phase = "check"
            self.checks = [self.lo, self.hi, self.lo]

    def result(self):
        if self.lo is None or self.hi is None:
            return None
        return (self.lo + self.hi) / 2, (self.hi - self.lo) / 2


def compute(paths, prefix="stall", where="at the end of a hard speed-up"):
    runs = []
    if paths:
        runs = [json.loads(Path(p).read_text()) for p in paths]
    else:
        files = sorted(OUT.glob(f"{prefix}_*.json"))
        allr = [dict(json.loads(f.read_text()), file=f.name) for f in files]
        bare = [r for r in allr if r.get("added_kg", 0) == 0 and r.get("a_stall")]
        load = [r for r in allr if r.get("added_kg", 0) > 0 and r.get("a_stall")]
        if not bare or not load:
            print("Need one finished bare-cart run (--added-kg 0) and one loaded run in results/.")
            return 1
        runs = [bare[-1], load[-1]]
    runs.sort(key=lambda r: r["added_kg"])
    r1, r2 = runs[0], runs[-1]
    for r in (r1, r2):
        print(f"  {r.get('file', ''):40s} added {r['added_kg']:.3f} kg, speed {r['top_speed']:.3f} m/s, "
              f"stall at {r['a_stall']:.2f} +/- {r['a_err']:.2f} m/s^2  ({r.get('label', '')})")
    if abs(r1["top_speed"] - r2["top_speed"]) > 0.02:
        print("WARNING: the two runs used different speeds; the motor force differs, the result is not valid.")
    dm = r2["added_kg"] - r1["added_kg"]
    a1, e1, a2, e2 = r1["a_stall"], r1["a_err"], r2["a_stall"], r2["a_err"]
    if a1 <= a2 or dm <= 0:
        print("The loaded cart did not stall at a lower acceleration: no estimate. Check the runs.")
        return 1
    m = dm * a2 / (a1 - a2)
    dm_da1 = -dm * a2 / (a1 - a2) ** 2
    dm_da2 = dm * a1 / (a1 - a2) ** 2
    m_err = math.hypot(dm_da1 * e1, dm_da2 * e2)
    F = m * a1
    F_err = math.hypot(m_err * a1, m * e1)
    Tq = F * R_PULLEY_M
    print()
    print(f"Effective moving mass (bare cart run): {m:.3f} +/- {m_err:.3f} kg")
    print(f"  minus the motor rotor's share ({ROTOR_EQ_KG:.3f} kg) = cart + belt + pulleys ~ {m - ROTOR_EQ_KG:.3f} kg")
    print(f"Motor force beyond friction at {r1['top_speed']:.2f} m/s, {where}: {F:.1f} +/- {F_err:.1f} N"
          f"  = {Tq:.3f} N m at r = {R_PULLEY_M * 1000:.1f} mm [r assumed]")
    rpm = r1["top_speed"] / (2 * math.pi * R_PULLEY_M) * 60
    print(f"  ({rpm:.0f} rpm; datasheet holding torque 1.26 N m at 0 rpm and 2.8 A)")
    print("Assumes friction is the same in both runs. Paste this output in the thread.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--added-kg", type=float, help="weighed mass strapped to the cart (0 for the bare cart)")
    ap.add_argument("--label", default="")
    ap.add_argument("--end-speed", type=float, default=0.8, help="speed at the end of the hard phase, m/s (same in both runs)")
    ap.add_argument("--max-speed", type=float, default=1.2, help="most negative start speed, m/s")
    ap.add_argument("--start-accel", type=float, default=6.0)
    ap.add_argument("--max-accel", type=float, default=40.0)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--list", action="store_true", help="show the coarse ladder, no motion")
    ap.add_argument("--compute", nargs="*", metavar="JSON", help="mass and force from two finished runs")
    ap.add_argument("--simulate-stall-at", type=float, help=argparse.SUPPRESS)   # testing without a person
    args = ap.parse_args()

    if args.compute is not None:
        return compute(args.compute)

    if args.list:
        print(f"Hard phase ends at +{args.end_speed} m/s; gentle phases {A_GENTLE} m/s^2; start speed >= -{args.max_speed} m/s")
        for a in [x for x in COARSE if args.start_accel <= x <= args.max_accel]:
            d = describe(args.end_speed, a, args.max_speed)
            print(f"  a {a:5.1f} m/s^2: {d['v0']:+.2f} -> {d['top']:+.3f} m/s in {d['n']} steps, mean {d['mean']:5.1f}, "
                  f"largest 10 ms step {d['peak']:5.1f} m/s^2, travel {d['xmin'] * 1000:+4.0f}..{d['xmax'] * 1000:+4.0f} mm, {d['dur']:.1f} s")
        return 0

    if args.added_kg is None:
        print("Give --added-kg (0 for the bare cart, or the weighed item's mass in kg).")
        return 1

    return run_search(args, lambda a: hard_profile(args.end_speed, a, args.max_speed)[0],
                      lambda a: describe(args.end_speed, a, args.max_speed),
                      "stall", "Two-load stall test", "two_load_stall_test.py")


def run_search(args, profile, describe_fn, prefix, title, script):
    """Shared by the stall and reversal tests: run the trials, ask after each, save, print the result."""
    OUT.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = "".join(c if c.isalnum() else "_" for c in args.label)[:40]
    base = OUT / f"{prefix}_{stamp}_{tag}"
    top = describe_fn(10)["top"]
    print(f"{title}, added mass {args.added_kg:.3f} kg, hard phase ends at {top:.3f} m/s.")
    print("Arms OFF. Weight strapped down. Cart on the tape mark, >= 0.3 m from each limit switch. Hands off the rail.")
    if args.simulate_stall_at is None:
        ask("Press Enter to start (Ctrl+C stops at any time): ")

    search = Search(args.start_accel, args.max_accel)
    cart = Cart(args.port).open()
    print(f"Mega replied, step count {cart.steps0}. Results -> {base.name}.csv")
    rows = []
    stop_reason = ""
    try:
        k = 0
        while not search.done:
            if k >= 30:
                search.done, search.note = True, "30 trials without a consistent edge (answers flip): no result"
                break
            k += 1
            a = search.next()
            d = describe_fn(a)
            seq = profile(a)
            print(f"\n[{k}] {search.phase:6s} a = {a:5.2f} m/s^2 ({d['text']})")
            time.sleep(0.4)
            cart.run_speeds(seq, f"S{k}")
            cart.hold(0.3, f"S{k}")
            cart.go_to(0.0)
            if args.simulate_stall_at is not None:
                ans = "r" if a >= args.simulate_stall_at else ""
                print(f"  (simulated answer: {ans or 'clean'})")
            else:
                ans = ask("  Enter = clean, r = red LED, b = buzz/grind/clunk, o = off the mark, q = stop: ", "").lower()[:1]
            if ans == "q":
                stop_reason = "stopped by user"
                break
            stalled = ans in ("r", "b", "o")
            rows.append([datetime.now().isoformat(timespec="seconds"), args.label, args.added_kg, f"{d['top']:.4f}",
                         k, search.phase, f"{a:.2f}", f"{d['mean']:.2f}", f"{d['peak']:.2f}",
                         "stall" if stalled else "clean", ans])
            search.report(a, stalled)
            if stalled:
                cart.close(motor_off=True)
                if args.simulate_stall_at is None:
                    print("  Motor OFF. Slide the cart back to the tape mark by hand.")
                    ask("  Press Enter to switch the motor back on and go on: ")
                cart.reenable()
            lo = f"{search.lo:.2f}" if search.lo is not None else "-"
            hi = f"{search.hi:.2f}" if search.hi is not None else "-"
            print(f"  clean up to {lo}, stalls from {hi} m/s^2")
    except KeyboardInterrupt:
        stop_reason = "Ctrl+C"
        print("\nStopped.")
    except Exception as exc:
        stop_reason = f"{type(exc).__name__}: {exc}"
        print(f"\nFAULT: {stop_reason}")
    finally:
        try:
            cart.close(motor_off=True)
        except Exception:
            pass

    with open(f"{base}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "label", "added_kg", "top_speed_m_s", "trial", "phase", "a_target_m_s2",
                    "a_mean_hard_m_s2", "a_peak_step_m_s2", "result", "answer"])
        w.writerows(rows)
    with open(f"{base}_cycles.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "timer", "steps", "v_cmd_m_s", "trial"])
        w.writerows(cart.log)
    res = None
    if search.done and not search.note and search.result():
        # use the mean acceleration of the speeds actually sent (timer rounding), not the target
        a_lo = describe_fn(search.lo)["mean"]
        a_hi = describe_fn(search.hi)["mean"]
        res = ((a_lo + a_hi) / 2, max(abs(a_hi - a_lo) / 2, 0.25))
    summary = dict(label=args.label, added_kg=args.added_kg, speed_target=args.end_speed, top_speed=top,
                   a_stall=res[0] if res else None, a_err=res[1] if res else None,
                   clean_up_to=search.lo, stalls_from=search.hi, trials=len(rows),
                   note=search.note or stop_reason, time=stamp)
    Path(f"{base}.json").write_text(json.dumps(summary, indent=1))
    print()
    if res:
        print(f"RESULT: stalls at {res[0]:.2f} +/- {res[1]:.2f} m/s^2 (targets: clean up to {search.lo:.2f}, "
              f"stalls from {search.hi:.2f}) at {top:.3f} m/s with {args.added_kg:.3f} kg added.")
    else:
        print(f"No result: {search.note or stop_reason}. Clean up to {search.lo}, stalls from {search.hi}.")
    print(f"Saved {base.name}.csv, .json and _cycles.csv in results/")
    if res and args.added_kg > 0:
        print(f"\nNow: $HOME/PythonLibraries/bin/python {script} --compute")
    return 0


if __name__ == "__main__":
    sys.exit(main())
