#!/usr/bin/env python3
"""Brake vs speed-up test: can the cart slow down harder than it can speed up? Arms OFF, bare cart.

Two kinds of trial, at the same speed and the same acceleration:
  SPEED-UP: from standstill, speed up HARD to 0.8 m/s, cruise 60 ms, slow down gently (4 m/s^2).
  BRAKE   : speed up gently to 0.8 m/s, cruise 60 ms, brake HARD to a stop.
Each trial does it once forward and once backward, so it ends where it started (about 0.2 m of
travel each way). Each kind has its own stall search: climb 6, 8, 10 ... m/s^2 until the first
stutter, then halve the gap to 1 m/s^2 and re-check both ends. By default the two kinds take
turns (speed-up, brake, speed-up, ...), so motor warm-up affects both the same.

After each trial you answer:
    Enter = clean
    b     = any stutter, buzz, grind or clunk     (counts as a stall)
    r     = red LED came on                       (counts as a stall)
    o     = cart ended off the tape mark          (counts as a stall)
    q     = stop
After a stall the motor turns off: slide the cart back to the tape mark, press Enter.

Commands go out the way the swing-up V2b_D sends them (method D: every 5 ms, no waiting) unless
you pick another --method letter (A = every 10 ms and wait for each reply, as V2b; E = every 2 ms).

Read it: if BRAKE stalls at a clearly higher acceleration than SPEED-UP (more than about
1-2 m/s^2), the motor can slow down harder than it can speed up, and the swing-up limits could
treat the two differently.

Safety: arms OFF. Cart on a tape mark, >= 0.3 m from each limit switch. Hands off the rail.
Ctrl+C stops at any time (motor off).

Usage (on the Pi, in ~/Desktop/CartJolt):
  PY=$HOME/PythonLibraries/bin/python
  $PY brake_vs_speedup_test.py --list                 # what it would send, no motion
  $PY brake_vs_speedup_test.py --label "bare 24V"     # both searches, taking turns
  $PY brake_vs_speedup_test.py --test brake           # only the braking search
  $PY brake_vs_speedup_test.py --summary              # table of every finished run
Options: --method D (A..F), --speed 0.8, --cruise-ms 60, --start-accel 6, --max-accel 40.
"""
import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path

from cart_link import Cart, HOLD, speed_of_timer
from command_style_test import TECHNIQUES, timers, link_check
from reversal_method_test import make_method, run_trial, motor_back_on
from two_load_stall_test import A_GENTLE, COARSE, OUT, Search, ask

TRAVEL_LIMIT_M = 0.30
PAUSE_S = 0.15                      # standstill between the forward and the backward half
KINDS = ("speedup", "brake")
NAMES = {"speedup": "SPEED-UP", "brake": "BRAKE"}
MAX_TRIALS = 60


# ---------------------------------------------------------------- profiles
def ramp_p(v0, v1, a, p):
    """Speeds from v0 to v1 in steps of a*p, one per command period, ending exactly at v1."""
    out, v = [], v0
    step = math.copysign(a * p, v1 - v0)
    while abs(v1 - v) > abs(step) + 1e-12:
        v += step
        out.append(v)
    out.append(v1)
    return out


def hard_steps(v, a, p):
    """Whole steps of exactly a*p between 0 and |v|, plus the leftover (< a*p) as one small step.
    Returns (leftover, number of whole steps)."""
    n = int(abs(v) / (a * p) + 1e-9)
    return abs(v) - n * a * p, n


def half(kind, v, a, p, cruise):
    """One move in the direction of v's sign. The hard phase is whole steps of exactly a*p; the
    leftover small step goes where the speed is lowest (first when speeding up, last when braking),
    so the fast part always sees the full test acceleration. Returns (speeds, slice of the whole
    steps)."""
    s = math.copysign(1.0, v)
    n_cruise = max(1, int(round(cruise / p)))
    rest, n = hard_steps(v, a, p)
    if kind == "speedup":
        hard = ([s * rest] if rest > 1e-6 else []) + [s * (rest + k * a * p) for k in range(1, n + 1)]
        hard[-1] = v
        seq = hard + [v] * n_cruise + ramp_p(v, 0.0, A_GENTLE, p)
        hs = slice(len(hard) - n, len(hard))
    else:
        up = ramp_p(0.0, v, A_GENTLE, p)
        hard = [v - s * k * a * p for k in range(1, n + 1)] + ([0.0] if rest > 1e-6 else [])
        hard[-1] = 0.0 if rest <= 1e-6 else hard[-1]
        seq = up + [v] * n_cruise + hard
        i0 = len(up) + n_cruise
        hs = slice(i0, i0 + n)
    return seq, hs


def build(kind, m, v, a, cruise):
    """Timer commands for one trial (forward then backward) and what they do."""
    p = m["period"]
    seq1, h1 = half(kind, v, a, p, cruise)
    seq2, h2 = half(kind, -v, a, p, cruise)
    gap = [0.0] * max(1, int(round(PAUSE_S / p)))
    off = len(seq1) + len(gap)
    seq = seq1 + gap + seq2
    cmds = timers(seq, m["hyst"])
    cmds.append(-HOLD)
    q = [speed_of_timer(n) for n in cmds]
    means = []
    for hs in (h1, slice(h2.start + off, h2.stop + off)):
        before = q[hs.start - 1] if hs.start > 0 else 0.0
        n = hs.stop - hs.start
        means.append(abs(q[hs.stop - 1] - before) / (n * p))
    jumps = [abs(b - c) for c, b in zip([0.0] + q[:-1], q)]
    x, xmin, xmax = 0.0, 0.0, 0.0
    for s in q:
        x += s * p
        xmin, xmax = min(xmin, x), max(xmax, x)
    top = max(abs(s) for s in q)
    hard_ms = (h1.stop - h1.start) * p * 1000
    mean = sum(means) / 2
    what = (f"hard 0 -> {top:.2f} m/s in {hard_ms:.0f} ms, then gentle stop" if kind == "speedup"
            else f"gentle to {top:.2f} m/s, then hard stop in {hard_ms:.0f} ms")
    return cmds, dict(top=top, mean=mean, jump=max(jumps), xmin=xmin, xmax=xmax, end=x,
                      dur=len(cmds) * p, n=len(cmds), hard_ms=hard_ms,
                      text=f"{what}, mean {mean:.1f} m/s^2, both directions")


# ---------------------------------------------------------------- the run
def finish(kind, search, m, args):
    res = None
    if search.done and not search.note and search.result():
        a_lo = build(kind, m, args.speed, search.lo, args.cruise_ms / 1000)[1]["mean"]
        a_hi = build(kind, m, args.speed, search.hi, args.cruise_ms / 1000)[1]["mean"]
        res = ((a_lo + a_hi) / 2, max(abs(a_hi - a_lo) / 2, 0.25))
    if res:
        line = f"stalls at {res[0]:.2f} +/- {res[1]:.2f} m/s^2 (clean up to {search.lo:.2f}, stalls from {search.hi:.2f})"
    else:
        line = f"no result: {search.note or 'not finished'} (clean up to {search.lo}, stalls from {search.hi})"
    return res, line


def run(args, m, kinds):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    lab = "".join(c if c.isalnum() else "_" for c in args.label)[:40]
    base = OUT / f"brake_{m['tag']}_{stamp}_{lab}"
    cruise = args.cruise_ms / 1000
    sim = {"speedup": args.simulate_stall_at, "brake": args.simulate_brake_stall_at or args.simulate_stall_at}
    simulate = args.simulate_stall_at is not None
    print(f"\nBrake vs speed-up test, method {m['name']} ({m['text']}), speed {build(kinds[0], m, args.speed, 10, cruise)[1]['top']:.3f} m/s, "
          f"searches: {', '.join(NAMES[k] for k in kinds)}.")
    print("Arms OFF. Bare cart on the tape mark, >= 0.3 m from each limit switch. Hands off the rail.")
    if not simulate:
        ask("Press Enter to start (Ctrl+C stops at any time): ")
    OUT.mkdir(exist_ok=True)
    cart = Cart(args.port).open()
    link = link_check(cart)
    print(f"Mega link: round trip median {link[0]:.1f} ms, 95 % {link[1]:.1f} ms. Results -> {base.name}.csv")
    searches = {k: Search(args.start_accel, args.max_accel) for k in kinds}
    rows, stop_reason, k = [], "", 0
    try:
        while not all(s.done for s in searches.values()):
            for kind in kinds:
                search = searches[kind]
                if search.done:
                    continue
                if k >= MAX_TRIALS:
                    for s in searches.values():
                        if not s.done:
                            s.done, s.note = True, f"{MAX_TRIALS} trials without a consistent edge: no result"
                    break
                k += 1
                a = search.next()
                cmds, d = build(kind, m, args.speed, a, cruise)
                if max(-d["xmin"], d["xmax"]) > TRAVEL_LIMIT_M:
                    search.done, search.note = True, f"travel over {TRAVEL_LIMIT_M} m: use a lower --speed"
                    continue
                print(f"\n[{k}] {NAMES[kind]:8s} {search.phase:6s} a = {a:5.2f} m/s^2  ({d['text']})")
                g_mean, g_max = run_trial(cart, m, cmds, f"{kind}{k}")
                if simulate:
                    ans = "b" if a >= sim[kind] else ""
                    print(f"  (simulated answer: {ans or 'clean'})")
                else:
                    ans = ask("  Enter = clean, b = stutter/buzz/clunk, r = red LED, o = off the mark, q = stop: ", "").lower()[:1]
                if ans == "q":
                    raise KeyboardInterrupt
                stalled = ans in ("b", "r", "o")
                rows.append([datetime.now().isoformat(timespec="seconds"), args.label, m["name"], m["tag"], kind,
                             f"{d['top']:.4f}", k, search.phase, f"{a:.2f}", f"{d['mean']:.2f}", f"{d['hard_ms']:.0f}",
                             f"{g_mean:.2f}", f"{g_max:.1f}", "stall" if stalled else "clean", ans])
                search.report(a, stalled)
                if stalled:
                    motor_back_on(cart, simulate)
                lo = f"{search.lo:.2f}" if search.lo is not None else "-"
                hi = f"{search.hi:.2f}" if search.hi is not None else "-"
                print(f"  {NAMES[kind]}: clean up to {lo}, stalls from {hi} m/s^2")
    except KeyboardInterrupt:
        stop_reason = "stopped by user"
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
        w.writerow(["time", "label", "method", "method_tag", "kind", "top_speed_m_s", "trial", "phase",
                    "a_target_m_s2", "a_mean_m_s2", "hard_phase_ms", "sent_every_ms", "longest_gap_ms", "result", "answer"])
        w.writerows(rows)
    with open(f"{base}_cycles.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "timer", "steps", "v_cmd_m_s", "trial"])
        w.writerows(cart.log)
    out = dict(label=args.label, method=m["name"], method_tag=m["tag"], method_text=m["text"], speed_target=args.speed,
               top_speed=build(kinds[0], m, args.speed, 10, cruise)[1]["top"], cruise_ms=args.cruise_ms,
               trials=len(rows), note=stop_reason, time=stamp,
               link_round_trip_ms=dict(median=link[0], p95=link[1], max=link[2]))
    print()
    results = {}
    for kind in kinds:
        res, line = finish(kind, searches[kind], m, args)
        results[kind] = res
        out[kind] = dict(a_stall=res[0] if res else None, a_err=res[1] if res else None,
                         clean_up_to=searches[kind].lo, stalls_from=searches[kind].hi, note=searches[kind].note)
        print(f"RESULT {NAMES[kind]:8s}: {line}")
    Path(f"{base}.json").write_text(json.dumps(out, indent=1))
    if results.get("speedup") and results.get("brake"):
        (au, eu), (ab, eb) = results["speedup"], results["brake"]
        diff, err = ab - au, math.hypot(eu, eb)
        if abs(diff) <= max(2 * err, 1.5):
            verdict = "about the same: braking is no stronger than speeding up at this speed"
        elif diff > 0:
            verdict = f"BRAKING takes {diff:.1f} m/s^2 more ({ab / au:.2f}x) than speeding up"
        else:
            verdict = f"SPEEDING UP takes {-diff:.1f} m/s^2 more than braking"
        print(f"  -> {verdict}. (One run each; repeat to see the run-to-run spread.)")
    print(f"Saved {base.name}.csv, .json and _cycles.csv in results/. Paste this output in the thread.")
    return 0


def summary_table():
    files = sorted(OUT.glob("brake_*.json"))
    if not files:
        print("No finished runs in results/ yet.")
        return 1
    print(f"{'method':8s} {'speed':>6s} {'speed-up stall':>16s} {'brake stall':>16s}  label / file")
    for f in files:
        r = json.loads(f.read_text())
        cols = []
        for kind in KINDS:
            x = r.get(kind) or {}
            cols.append(f"{x['a_stall']:.2f} +/- {x['a_err']:.2f}" if x.get("a_stall") else
                        ("-" if not x else f"clean to {x.get('clean_up_to')}"))
        print(f"{r['method']:8s} {r['top_speed']:6.3f} {cols[0]:>16s} {cols[1]:>16s}  {r.get('label', '')} ({f.name})")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test", choices=["both", "brake", "speedup"], default="both")
    ap.add_argument("--method", default="D", help="how commands are sent: A..F (default D = every 5 ms, no waiting)")
    ap.add_argument("--speed", type=float, default=0.8, help="speed the cart reaches, m/s")
    ap.add_argument("--cruise-ms", type=float, default=60.0, help="steady speed between the two phases, ms")
    ap.add_argument("--start-accel", type=float, default=6.0)
    ap.add_argument("--max-accel", type=float, default=40.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--list", action="store_true", help="show what it would send, no motion")
    ap.add_argument("--summary", action="store_true", help="table of every finished run in results/")
    ap.add_argument("--simulate-stall-at", type=float, help=argparse.SUPPRESS)        # testing without a person
    ap.add_argument("--simulate-brake-stall-at", type=float, help=argparse.SUPPRESS)
    args = ap.parse_args()
    # make_method() reads these reversal_method_test options; this test uses the letter as is
    args.period = args.smooth = args.send = None
    args.hyst, args.mega = False, "slave"

    if args.summary:
        return summary_table()
    letter = args.method.strip().upper()
    if letter not in TECHNIQUES or letter == "G":
        sys.exit("--method: use one letter A..F (G's S-curve is not supported in this test)")
    m = make_method(letter, args)
    kinds = KINDS if args.test == "both" else (args.test,)
    cruise = args.cruise_ms / 1000

    accels = [x for x in COARSE if args.start_accel <= x <= args.max_accel]
    print(f"Method {m['name']}: {m['text']}; speed {args.speed} m/s, cruise {args.cruise_ms:g} ms, gentle phases {A_GENTLE} m/s^2")
    for kind in kinds:
        for a in accels:
            cmds, d = build(kind, m, args.speed, a, cruise)
            far = max(-d["xmin"], d["xmax"]) > TRAVEL_LIMIT_M
            if args.list or far:
                print(f"  {NAMES[kind]:8s} a {a:5.1f}: {d['text']}; {d['n']} commands, biggest speed jump {d['jump']:.3f} m/s, "
                      f"travel {d['xmin'] * 1000:+4.0f}..{d['xmax'] * 1000:+4.0f} mm, ends {d['end'] * 1000:+.0f} mm, {d['dur']:.1f} s")
            if far and not args.list:
                sys.exit(f"That would travel more than {TRAVEL_LIMIT_M} m from the start. Use a lower --speed.")
    if args.list:
        return 0
    return run(args, m, kinds)


if __name__ == "__main__":
    sys.exit(main())
