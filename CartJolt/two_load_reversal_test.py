#!/usr/bin/env python3
"""Two-load REVERSAL test: like two_load_stall_test.py, but each trial is back-to-back hard
reversals with no braking pause, the way the swing-up paths throw the cart: arms OFF.

Each trial: move gently (4 m/s^2) to -0.8 m/s, then reverse at the test acceleration to +0.8,
straight back to -0.8, straight back to +0.8 (3 reversals, --reversals to change), then slow
down gently and return to the start. Every reversal ends speeding up at full speed with the
exact test acceleration; any leftover speed change goes in the first 10 ms (while braking, the
easy part). Travel stays within about 0.15 m of the start.

Same answers, same search and same math as two_load_stall_test.py:
    Enter = clean, r = red LED, b = buzz / grind / loud clunk, o = off the tape mark, q = stop
    m = dm * a2 / (a1 - a2)  (effective moving mass),  m * a1 = motor force beyond friction

Compare the result with two_load_stall_test.py: if the reversals stall at a lower acceleration
than the single speed-up, the back-to-back direction changes cost the motor extra (lag carried
from one reversal into the next).

Safety: arms OFF. Strap the added weight down hard. Cart on a tape mark, at least 0.3 m from each
limit switch. Hands off the rail. Ctrl+C stops at any time (motor off).

Usage (on the Pi, in ~/Desktop/CartJolt):
  PY=$HOME/PythonLibraries/bin/python
  $PY two_load_reversal_test.py --list                                    # show the ladder, no motion
  $PY two_load_reversal_test.py --added-kg 0     --label "bare cart"       # run 1
  $PY two_load_reversal_test.py --added-kg 1.234 --label "cart + weight"   # run 2 (your weighed item)
  $PY two_load_reversal_test.py --compute                                 # mass + force from the two runs
Options: --end-speed 0.8 (m/s, both runs must match), --reversals 3, --start-accel 6, --max-accel 40.
"""
import argparse
import math
import sys

from cart_link import T
from two_load_stall_test import A_GENTLE, COARSE, ramp, quantize, compute, run_search


def reverse(v_from, v_to, a):
    """One hard reversal: a leftover first step, then whole steps of a*T ending exactly at v_to.
    Returns (speeds, index where the whole steps start)."""
    s = math.copysign(1.0, v_to - v_from)
    dv = abs(v_to - v_from)
    n = int(dv / (a * T) + 1e-9)
    rest = dv - n * a * T
    out, v = [], v_from
    if rest > 1e-6:
        v += s * rest
        out.append(v)
    first_whole = len(out)
    for _ in range(n):
        v += s * a * T
        out.append(v)
    out[-1] = v_to
    return out, first_whole


def reversal_profile(v, a, n_rev):
    """Gentle to -v, n_rev hard reversals between -v and +v, gentle to 0.
    Returns (speeds, list of (start, stop) index pairs of the whole-step parts)."""
    seq = ramp(0.0, -v, A_GENTLE)
    parts = []
    cur = -v
    for _ in range(n_rev):
        nxt = -cur
        r, w = reverse(cur, nxt, a)
        parts.append((len(seq) + w, len(seq) + len(r)))
        seq += r
        cur = nxt
    seq += ramp(cur, 0.0, A_GENTLE)
    return seq, parts


def describe(v, a, n_rev):
    seq, parts = reversal_profile(v, a, n_rev)
    q = [quantize(x) for x in seq]
    acc = [abs(b - c) / T for c, b in zip([0.0] + q[:-1], q)]
    means, peak = [], 0.0
    for i0, i1 in parts:
        n = i1 - i0
        if n > 0:
            means.append(abs(q[i1 - 1] - q[i0 - 1]) / (n * T))
        peak = max([peak] + acc[i0:i1])
    mean = sum(means) / len(means) if means else 0.0
    x, xmin, xmax = 0.0, 0.0, 0.0
    for sp in q:
        x += sp * T
        xmin, xmax = min(xmin, x), max(xmax, x)
    top = abs(q[parts[0][1] - 1])
    return dict(top=top, peak=peak, mean=mean, xmin=xmin, xmax=xmax, dur=len(seq) * T,
                text=f"{n_rev} reversals +/-{top:.2f} m/s, mean {mean:.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--added-kg", type=float, help="weighed mass strapped to the cart (0 for the bare cart)")
    ap.add_argument("--label", default="")
    ap.add_argument("--end-speed", type=float, default=0.8, help="reversal speed, m/s (same in both runs)")
    ap.add_argument("--reversals", type=int, default=3)
    ap.add_argument("--start-accel", type=float, default=6.0)
    ap.add_argument("--max-accel", type=float, default=40.0)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--list", action="store_true", help="show the coarse ladder, no motion")
    ap.add_argument("--compute", nargs="*", metavar="JSON", help="mass and force from two finished runs")
    ap.add_argument("--simulate-stall-at", type=float, help=argparse.SUPPRESS)   # testing without a person
    args = ap.parse_args()

    if args.compute is not None:
        return compute(args.compute, prefix="rev", where="after back-to-back reversals")

    if args.list:
        print(f"{args.reversals} reversals between -{args.end_speed} and +{args.end_speed} m/s; gentle phases {A_GENTLE} m/s^2")
        for a in [x for x in COARSE if args.start_accel <= x <= args.max_accel]:
            d = describe(args.end_speed, a, args.reversals)
            print(f"  a {a:5.1f} m/s^2: +/-{d['top']:.3f} m/s, mean {d['mean']:5.1f}, largest 10 ms step {d['peak']:5.1f} m/s^2, "
                  f"travel {d['xmin'] * 1000:+4.0f}..{d['xmax'] * 1000:+4.0f} mm, {d['dur']:.1f} s")
        return 0

    if args.added_kg is None:
        print("Give --added-kg (0 for the bare cart, or the weighed item's mass in kg).")
        return 1

    return run_search(args, lambda a: reversal_profile(args.end_speed, a, args.reversals)[0],
                      lambda a: describe(args.end_speed, a, args.reversals),
                      "rev", "Two-load reversal test", "two_load_reversal_test.py")


if __name__ == "__main__":
    sys.exit(main())
