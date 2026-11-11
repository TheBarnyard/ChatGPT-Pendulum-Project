#!/usr/bin/env python3
"""Command style test: does HOW the Pi sends speed commands change the cart's stutter?

The same cart move is sent several ways, in a shuffled, blind order. After each trial you score
what you saw and heard; at the end the script ranks the techniques. Nothing needs reflashing:
everything here is on the Pi side, with the team's ArduinoSlave on the Mega.

Techniques (letters are what you see in the summary):
  A  10 ms steps        what the swing-up does now: a new speed every 10 ms (a staircase)
  B  20 ms steps        half the command rate, bigger speed jumps
  C  40 ms steps        quarter rate, much bigger jumps
  D   5 ms steps        double rate, half-size jumps
  E   2 ms steps        5x rate, nearly a smooth ramp (what a ramping Mega sketch would give)
  F  10 ms + hysteresis at high speed the timer can only make speeds about 5 % apart, so the
                        speed flips between two timer values; F holds the old value until the
                        request is a full step away
  G  10 ms + smoothed   the same move with its corners rounded over 30 ms (less jerk; the top
                        speed of sharp reversals drops a little)

How to read it:
  B/C worse, D/E better  -> the speed staircase is the trouble: send faster, or flash a ramping sketch
  B/C better, D/E worse  -> commands really are "too fast" for the Mega/driver: slow the loop
  F better               -> timer rounding flip-flop at speed causes it: add hysteresis in the scripts
  G better               -> jerk (sudden accel changes) causes it: smooth the paths
  all the same           -> it is torque / driver tuning, not how the commands are sent

The Pi does not wait for the Mega's reply each cycle here (so 2 ms is possible); it still reads the
replies and stops if the Mega goes silent. A link check at the start prints the round-trip time.

Moves (--motion):
  reversal  gentle to -v, then back-to-back reversals between -v and +v at --accel (default)
  sine      cart speed v*sin(2 pi f t) for a few cycles (a smooth shake)
  cruise    out and back at a steady 0.5 m/s: only the command rate changes, not the speed,
            so any difference is the command stream itself
  replay    a logged swing attempt (--replay-csv RUN.csv --attempt N), arms off

Scores after each trial: 0 = smooth, 1 = slight stutter, 2 = clear stutter / skip / clunk,
r = red LED, x = missed it (not counted), q = stop. After a red LED the motor turns off: slide the
cart back to the tape mark and press Enter.

Safety: arms OFF. Cart on a tape mark, >= 0.3 m from each limit switch. Hands off the rail.

Usage (on the Pi, in ~/Desktop/CartJolt):
  PY=$HOME/PythonLibraries/bin/python
  $PY command_style_test.py --list                         # what each technique sends, no motion
  $PY command_style_test.py                                # reversals +/-0.7 m/s at 12 m/s^2, 2 repeats
  $PY command_style_test.py --accel 16                     # pick an accel where it stutters now
  $PY command_style_test.py --motion cruise
  $PY command_style_test.py --motion replay --replay-csv ~/Desktop/DoubleSwingUpV2/run_logs/swingup_double_20261104_162911_974833.csv --attempt 1
Options: --techniques A,C,E   --repeats 3   --show (not blind)   --speed 0.7   --hz 2   --cycles 4
"""
import argparse
import bisect
import csv
import math
import random
import statistics
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

from cart_link import Cart, HOLD, M_PER_STEP, TIMER_STEP_HZ, T, speed_of_timer, timer_for_speed

HERE = Path(__file__).resolve().parent
A_GENTLE = 4.0
TRAVEL_LIMIT_M = 0.30

TECHNIQUES = {
    "A": ("10 ms steps (as now)", dict(period=0.010)),
    "B": ("20 ms steps", dict(period=0.020)),
    "C": ("40 ms steps", dict(period=0.040)),
    "D": ("5 ms steps", dict(period=0.005)),
    "E": ("2 ms steps (near ramp)", dict(period=0.002)),
    "F": ("10 ms + timer hysteresis", dict(period=0.010, hyst=True)),
    "G": ("10 ms + smoothed 30 ms", dict(period=0.010, smooth=0.030)),
}


# ---------------------------------------------------------------- moves as speed-vs-time knots
class Knots:
    """Piecewise-linear cart speed v(t) through (t, v) points; 0 outside."""

    def __init__(self, pts):
        self.t = [p[0] for p in pts]
        self.v = [p[1] for p in pts]
        self.dur = self.t[-1]

    def __call__(self, t):
        if t <= self.t[0] or t >= self.t[-1]:
            return 0.0
        i = bisect.bisect_right(self.t, t) - 1
        t0, t1 = self.t[i], self.t[i + 1]
        return self.v[i] + (self.v[i + 1] - self.v[i]) * (t - t0) / (t1 - t0) if t1 > t0 else self.v[i + 1]


def move_reversal(v, a, n_rev):
    pts = [(0.0, 0.0)]
    t = v / A_GENTLE
    pts.append((t, -v))
    cur = -v
    for _ in range(n_rev):
        t += 2 * v / a
        cur = -cur
        pts.append((t, cur))
    t += v / A_GENTLE
    pts.append((t, 0.0))
    return Knots(pts)


def move_sine(v, hz, cycles):
    w = 2 * math.pi * hz
    n = int(cycles / hz / 0.001)
    return Knots([(k * 0.001, v * math.sin(w * k * 0.001)) for k in range(n + 1)] + [((n + 1) * 0.001, 0.0)])


def move_cruise(v=0.5, dist=0.20, a=3.0):
    ta = v / a
    tc = max(0.0, (dist - v * ta) / v)
    pts, t = [(0.0, 0.0)], 0.0
    for s in (1, -1):
        pts += [(t + ta, s * v), (t + ta + tc, s * v), (t + 2 * ta + tc, 0.0)]
        t += 2 * ta + tc
        pts.append((t + 0.3, 0.0))      # pause before coming back
        t += 0.3
    return Knots(pts)


def move_replay(path, attempt, scale):
    from replay_run_motor import load, attempts
    rows = load(path)
    att = attempts(rows)
    if not att:
        sys.exit("No SWING attempts found in that CSV.")
    if not 1 <= attempt <= len(att):
        sys.exit(f"--attempt must be 1..{len(att)}")
    i0, i1 = att[attempt - 1]
    sp = [scale * speed_of_timer(n) for _, n, _ in rows[i0:i1]]
    pts = [(0.0, 0.0)] + [((k + 0.5) * T, s) for k, s in enumerate(sp)] + [((len(sp) + 0.5) * T, 0.0)]
    # gentle stop if the attempt ended moving
    if abs(sp[-1]) > 0.02:
        pts[-1] = ((len(sp) - 0.5) * T + abs(sp[-1]) / A_GENTLE, 0.0)
    return Knots(pts)


# ---------------------------------------------------------------- techniques -> command lists
def sample(move, period, smooth=0.0):
    """One speed per period: the move's average over the period (midpoint of a linear piece),
    or its average over a centred window of 'smooth' seconds."""
    t0 = -smooth / 2
    n = int(math.ceil((move.dur + smooth) / period)) + 1
    out = []
    for k in range(n):
        tm = t0 + (k + 0.5) * period
        if smooth > 0:
            m = max(3, int(smooth / 0.001))
            out.append(sum(move(tm - smooth / 2 + (j + 0.5) * smooth / m) for j in range(m)) / m)
        else:
            out.append(move(tm))
    out.append(0.0)
    return out


def timers(speeds, hyst=False):
    out, prev = [], None
    for v in speeds:
        n = timer_for_speed(v)
        if hyst and n is not None and prev is not None and abs(prev) < HOLD and (n > 0) == (prev > 0):
            nm = abs(prev)
            fast = TIMER_STEP_HZ / nm * M_PER_STEP          # speed of |N| - 1
            slow = TIMER_STEP_HZ / (nm + 2) * M_PER_STEP    # speed of |N| + 1
            if slow < abs(v) < fast:
                n = prev
        if n is None:
            n = -HOLD
        out.append(n)
        prev = n
    return out


def profile_stats(cmds, period):
    v = [speed_of_timer(n) for n in cmds]
    acc = [abs(b - a) / period for a, b in zip([0.0] + v[:-1], v)]
    jumps = [abs(b - a) for a, b in zip([0.0] + v[:-1], v)]
    x, xmin, xmax = 0.0, 0.0, 0.0
    for s in v:
        x += s * period
        xmin, xmax = min(xmin, x), max(xmax, x)
    flips = sum(1 for a, b, c in zip(cmds, cmds[1:], cmds[2:]) if a == c and a != b)
    return dict(n=len(cmds), dur=len(cmds) * period, vmax=max(map(abs, v)), jump=max(jumps),
                acc=max(acc), xmin=xmin, xmax=xmax, xend=x, flips=flips)


def build(move, key):
    name, opt = TECHNIQUES[key]
    p = opt["period"]
    cmds = timers(sample(move, p, opt.get("smooth", 0.0)), opt.get("hyst", False))
    return cmds, p


# ---------------------------------------------------------------- sending
def link_check(cart, n=150):
    rt = []
    for _ in range(n):
        t = time.monotonic()
        cart.send(-HOLD)
        rt.append((time.monotonic() - t) * 1000)
        time.sleep(0.004)
    rt.sort()
    return rt[len(rt) // 2], rt[int(len(rt) * 0.95)], rt[-1]


def stream(cart, cmds, period, tag):
    """Send one timer value per period without waiting for replies; read replies as they come.
    Returns the actual send intervals (ms)."""
    ser = cart.ser
    ser.reset_input_buffer()
    sent = []
    last_rx = time.monotonic()
    next_t = time.monotonic()
    for n in cmds:
        now = time.monotonic()
        ser.write(struct.pack("<i", int(n)))
        sent.append(now)
        cart.log.append((now - cart.t0, n, cart.steps, speed_of_timer(n), tag))
        k = ser.in_waiting
        if k:
            ser.read(k)
            last_rx = now
        elif now - last_rx > 0.15:
            raise TimeoutError("the Mega stopped replying (limit switch or stop button?)")
        next_t += period
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        elif dt < -5 * period:
            next_t = time.monotonic()
    time.sleep(0.03)
    ser.reset_input_buffer()
    cart.send(-HOLD)                  # waits for a fresh reply: step count is current again
    return [(b - a) * 1000 for a, b in zip(sent, sent[1:])]


def ask(prompt, default=""):
    try:
        s = input(prompt).strip()
    except EOFError:
        s = ""
    return s or default


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motion", choices=["reversal", "sine", "cruise", "replay"], default="reversal")
    ap.add_argument("--speed", type=float, default=0.7, help="m/s (reversal, sine)")
    ap.add_argument("--accel", type=float, default=12.0, help="m/s^2 (reversal)")
    ap.add_argument("--reversals", type=int, default=3)
    ap.add_argument("--hz", type=float, default=2.0, help="sine frequency")
    ap.add_argument("--cycles", type=float, default=4)
    ap.add_argument("--replay-csv")
    ap.add_argument("--attempt", type=int, default=1)
    ap.add_argument("--speed-scale", type=float, default=1.0, help="replay only")
    ap.add_argument("--techniques", default="ABCDEFG")
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--show", action="store_true", help="say which technique runs before each trial")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--label", default="")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--simulate", action="store_true", help=argparse.SUPPRESS)   # random answers, for testing
    args = ap.parse_args()

    if args.motion == "reversal":
        move = move_reversal(args.speed, args.accel, args.reversals)
        mdesc = f"{args.reversals} reversals +/-{args.speed} m/s at {args.accel} m/s^2"
    elif args.motion == "sine":
        move = move_sine(args.speed, args.hz, args.cycles)
        mdesc = f"sine {args.speed} m/s peak at {args.hz} Hz x {args.cycles} (peak accel {args.speed * 2 * math.pi * args.hz:.1f} m/s^2)"
    elif args.motion == "cruise":
        move = move_cruise()
        mdesc = "out and back at a steady 0.5 m/s"
    else:
        if not args.replay_csv:
            sys.exit("--motion replay needs --replay-csv RUN.csv")
        move = move_replay(args.replay_csv, args.attempt, args.speed_scale)
        mdesc = f"replay {Path(args.replay_csv).name} attempt {args.attempt} x{args.speed_scale}"
    keys = [k for k in args.techniques.upper() if k in TECHNIQUES]
    if not keys:
        sys.exit("--techniques: pick letters from ABCDEFG")
    built = {k: build(move, k) for k in keys}
    limit = 0.40 if args.motion == "replay" else TRAVEL_LIMIT_M
    print(f"Move: {mdesc}")
    for k in keys:
        cmds, p = built[k]
        s = profile_stats(cmds, p)
        print(f"  {k} {TECHNIQUES[k][0]:26s} {s['n']:4d} commands, biggest speed jump {s['jump']:.3f} m/s "
              f"({s['acc']:5.1f} m/s^2 over one step), top {s['vmax']:.2f} m/s, timer flip-flops {s['flips']:3d}, "
              f"travel {s['xmin'] * 1000:+4.0f}..{s['xmax'] * 1000:+4.0f} mm")
        if max(abs(s["xmin"]), abs(s["xmax"])) > limit:
            sys.exit(f"Technique {k} would travel more than {limit} m from the start. Use a smaller move.")
    if args.list:
        return 0

    rnd = random.Random(args.seed)
    order = [k for k in keys for _ in range(args.repeats)]
    rnd.shuffle(order)
    if "A" in keys:
        order = ["A"] + order          # reference first, shown
    out_dir = HERE / "results"
    out_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = "".join(c if c.isalnum() else "_" for c in (args.label or args.motion))[:40]
    path = out_dir / f"style_{stamp}_{tag}.csv"

    print(f"\n{len(order)} trials, {'shown' if args.show else 'blind (the letter shows after you score)'}.")
    print("Arms OFF. Cart on the tape mark, >= 0.3 m from each limit switch. Hands off the rail.")
    if not args.simulate:
        ask("Press Enter to start (Ctrl+C stops at any time): ")
    cart = Cart(args.port).open()
    med, p95, mx = link_check(cart)
    print(f"Mega link: round trip median {med:.1f} ms, 95 % {p95:.1f} ms, max {mx:.1f} ms "
          f"(the swing loop has 10 ms; a 2 ms period does not wait for replies)")
    rows = []
    try:
        for i, k in enumerate(order, 1):
            cmds, p = built[k]
            shown = args.show or (i == 1 and k == "A" and "A" in keys)
            head = f"[{i}/{len(order)}] " + (f"{k}: {TECHNIQUES[k][0]}" + (" (reference)" if i == 1 else "") if shown else "trial")
            print(f"\n{head}")
            time.sleep(0.6)
            gaps = stream(cart, cmds, p, f"{i}{k}")
            cart.hold(0.3, f"{i}{k}")
            cart.go_to(0.0)
            g_mean = statistics.mean(gaps) if gaps else 0.0
            g_max = max(gaps) if gaps else 0.0
            if args.simulate:
                ans = rnd.choice(["0", "1", "2", "0"])
                print(f"  (simulated score {ans})")
            else:
                ans = ask("  Score: 0 smooth, 1 slight stutter, 2 clear stutter/skip, r red LED, x missed, q stop: ", "x").lower()
            code = ans[:1]
            if code == "q":
                break
            if not shown:
                print(f"  that was {k}: {TECHNIQUES[k][0]}")
            print(f"  sent every {g_mean:.2f} ms on average, longest gap {g_max:.1f} ms (target {p * 1000:.0f} ms)")
            score = {"0": 0, "1": 1, "2": 2, "r": 3}.get(code)
            rows.append([datetime.now().isoformat(timespec="seconds"), args.label, args.motion, i, k, TECHNIQUES[k][0],
                         p * 1000, f"{g_mean:.2f}", f"{g_max:.1f}", code, "" if score is None else score, ans[1:].strip()])
            if code == "r":
                cart.close(motor_off=True)
                if not args.simulate:
                    print("  Motor OFF. Slide the cart back to the tape mark by hand.")
                    ask("  Press Enter to switch the motor back on and go on: ")
                cart.reenable()
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as exc:
        print(f"\nFAULT: {type(exc).__name__}: {exc}")
    finally:
        try:
            cart.close(motor_off=True)
        except Exception:
            pass

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "label", "motion", "trial", "technique", "name", "period_ms", "sent_every_ms",
                    "longest_gap_ms", "answer", "score", "notes"])
        w.writerows(rows)
    with open(str(path).replace(".csv", "_cycles.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "timer", "steps", "v_cmd_m_s", "trial"])
        w.writerows(cart.log)

    print(f"\nSummary ({mdesc}); score 0 smooth .. 2 clear stutter, red LED = 3:")
    res = []
    for k in keys:
        sc = [r[10] for r in rows if r[4] == k and r[10] != ""]
        reds = sum(1 for r in rows if r[4] == k and r[9] == "r")
        if sc:
            res.append((statistics.mean(sc), k, len(sc), reds, sc))
    for m, k, n, reds, sc in sorted(res):
        print(f"  {k} {TECHNIQUES[k][0]:26s} mean {m:.2f}  ({n} trials: {' '.join(map(str, sc))}; red LED {reds})")
    print(f"Saved {path.name} (and _cycles.csv) in results/. Paste this summary in the thread.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
