#!/usr/bin/env python3
"""Reversal test with a choice of control method: the same back-to-back hard reversals as
two_load_reversal_test.py, but you pick HOW the Pi sends the speed commands with -- options.
Arms OFF.

Each trial: move gently (4 m/s^2) to -0.8 m/s, reverse at the test acceleration to +0.8, back to
-0.8, back to +0.8 (3 reversals), then slow down gently and return to the start.

Control methods (--method, letters match command_style_test.py):
  A  new speed every 10 ms, wait for the Mega's reply each time   (what the swing-up does now)
  B  every 20 ms, wait                    C  every 40 ms, wait
  D  every 5 ms, stream (no waiting)      E  every 2 ms, stream (nearly a smooth ramp)
  F  10 ms + timer hysteresis (stops the timer flip-flopping between two values at speed)
  G  10 ms + S-curve: the acceleration flips over 30 ms instead of at once (less jerk; same top
     speed and same acceleration through the middle; each reversal takes 15 ms longer)
Build your own with --period MS, --smooth MS, --hyst, --send wait|stream (these also change a
letter: --method D --hyst = 5 ms stream with hysteresis). --method A,E,G runs one after another.
--mega ramp only labels the results if you flashed ArduinoSlaveRamp (use it with 10 ms, wait).

Two ways to run:
  search (default): climb then bisect the acceleration until the motor stalls; gives a stall
                    acceleration per method. Answer after each trial:
                    Enter = clean, r = red LED, b = buzz/grind/clunk, o = off the tape mark, q = stop
  --accel 16      : the same acceleration --repeats times per method (methods alternate);
                    score each: 0 smooth, 1 slight stutter, 2 clear stutter/skip, r red LED, x missed
After a stall or red LED the motor turns off: slide the cart back to the tape mark, press Enter.

Read it: a method that stalls at a clearly higher acceleration (more than about 1 m/s^2, the
search's resolution) or scores lower is better for the cart. All the same = torque / driver
settings, not how the commands are sent.

Safety: arms OFF. Cart on a tape mark, >= 0.3 m from each limit switch. Hands off the rail.
Ctrl+C stops at any time (motor off).

Usage (on the Pi, in ~/Desktop/CartJolt):
  PY=$HOME/PythonLibraries/bin/python
  $PY reversal_method_test.py --method E --list               # what it would send, no motion
  $PY reversal_method_test.py --method A                      # stall search, method A
  $PY reversal_method_test.py --method A,E,F,G                # four searches, one after another
  $PY reversal_method_test.py --period 5 --hyst --send wait   # your own method
  $PY reversal_method_test.py --method A,E --accel 16 --repeats 4
  $PY reversal_method_test.py --summary                       # table of every finished search
  $PY reversal_method_test.py --method E --compute            # mass + force (needs --added-kg 0 and a loaded run, same method)
Options: --end-speed 0.8, --reversals 3, --start-accel 6, --max-accel 40, --added-kg 0, --label TEXT.
"""
import argparse
import csv
import json
import math
import statistics
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

from cart_link import Cart, HOLD, speed_of_timer
from command_style_test import TECHNIQUES, timers, link_check
from two_load_stall_test import A_GENTLE, COARSE, OUT, Search, ask, compute

TRAVEL_LIMIT_M = 0.30
SEND_AUTO_BELOW_MS = 10.0       # periods under this stream by default (no time to wait for replies)


# ---------------------------------------------------------------- methods
def make_method(letter, args):
    """Preset letter plus any --period/--smooth/--hyst/--send overrides -> method dict."""
    base = dict(TECHNIQUES[letter][1])
    p_ms = args.period if args.period is not None else base["period"] * 1000
    smooth_ms = args.smooth if args.smooth is not None else base.get("smooth", 0.0) * 1000
    hyst = args.hyst or base.get("hyst", False)
    send = args.send or ("stream" if p_ms < SEND_AUTO_BELOW_MS else "wait")
    if p_ms < 1.0 or p_ms > 100.0:
        sys.exit("--period: use 1 to 100 ms")
    tag = f"p{p_ms:g}{'h' if hyst else ''}{f'm{smooth_ms:g}' if smooth_ms > 0 else ''}-{send}"
    if args.mega == "ramp":
        tag += "-ramp"
    text = f"every {p_ms:g} ms, {send}" + (", hysteresis" if hyst else "") + \
           (f", S-curve {smooth_ms:g} ms" if smooth_ms > 0 else "") + (", Mega ramp sketch" if args.mega == "ramp" else "")
    custom = any(x is not None for x in (args.period, args.smooth, args.send)) or (args.hyst and not base.get("hyst"))
    name = (f"{letter}+custom" if custom else letter)
    return dict(name=name, tag=tag, text=text, period=p_ms / 1000, smooth=smooth_ms / 1000, hyst=hyst, send=send)


# ---------------------------------------------------------------- profile at any command period
def ramp_p(v0, v1, a, p):
    out, v = [], v0
    step = math.copysign(a * p, v1 - v0)
    while abs(v1 - v) > abs(step) + 1e-12:
        v += step
        out.append(v)
    out.append(v1)
    return out


def reverse_p(v_from, v_to, a, p):
    """One hard reversal: a leftover first step, then whole steps of a*p ending exactly at v_to.
    Returns (speeds, index where the whole steps start)."""
    s = math.copysign(1.0, v_to - v_from)
    dv = abs(v_to - v_from)
    n = int(dv / (a * p) + 1e-9)
    if n == 0:                                  # slower command rate than the reversal: one jump
        return [v_to], 0
    rest = dv - n * a * p
    out, v = [], v_from
    if rest > 1e-6:
        v += s * rest
        out.append(v)
    first_whole = len(out)
    for _ in range(n):
        v += s * a * p
        out.append(v)
    out[-1] = v_to
    return out, first_whole


def raw_profile(v, a, n_rev, p):
    seq = ramp_p(0.0, -v, A_GENTLE, p)
    parts, cur = [], -v
    for _ in range(n_rev):
        r, w = reverse_p(cur, -cur, a, p)
        parts.append((len(seq) + w, len(seq) + len(r)))
        seq += r
        cur = -cur
    seq += ramp_p(cur, 0.0, A_GENTLE, p)
    return seq, parts


def scurve(v, a, n_rev, p, tc):
    """Same moves, but every change of acceleration is spread over tc seconds instead of a jump
    (an S-curve: the acceleration flips from +a to -a over tc at each reversal). Every corner still
    reaches the full speed and the middle of each reversal is still at a; each reversal takes tc/2
    longer. Sampled like the sharp profile (speed at the end of each command period).
    Returns (speeds, index of each reversal's corner)."""
    tj = tc / 2
    segs = [(-v, A_GENTLE)]
    cur = -v
    for _ in range(n_rev):
        cur = -cur
        segs.append((cur, a))
    segs.append((0.0, A_GENTLE))
    pieces, t, vnow, corners = [], 0.0, 0.0, []      # (t0, t1, acc0, acc1)
    for k, (vt, acc) in enumerate(segs):
        dv = vt - vnow
        sg = math.copysign(1.0, dv)
        if abs(dv) >= acc * tj:
            tcst = abs(dv) / acc - tj
            pieces += [(t, t + tj, 0.0, sg * acc), (t + tj, t + tj + tcst, sg * acc, sg * acc),
                       (t + tj + tcst, t + 2 * tj + tcst, sg * acc, 0.0)]
            t += 2 * tj + tcst
        else:
            apk = math.sqrt(abs(dv) * acc / tj)
            tr = apk / (acc / tj)
            pieces += [(t, t + tr, 0.0, sg * apk), (t + tr, t + 2 * tr, sg * apk, 0.0)]
            t += 2 * tr
        vnow = vt
        if 1 <= k <= n_rev:
            corners.append(t)
    dt = min(p / 20, 0.0005)
    n_fine = int(math.ceil(t / dt))
    vel, vv, j = [0.0], 0.0, 0
    for i in range(n_fine):
        tm = (i + 0.5) * dt
        while j < len(pieces) - 1 and tm > pieces[j][1]:
            j += 1
        t0, t1, a0, a1 = pieces[j]
        vv += (a0 + (a1 - a0) * (tm - t0) / (t1 - t0) if t1 > t0 else a1) * dt
        vel.append(vv)
    out = []
    k = 1
    while k * p < t:
        out.append(vel[min(int(round(k * p / dt)), len(vel) - 1)])
        k += 1
    out.append(0.0)
    idx = [min(max(int(round(tcn / p)) - 1, 0), len(out) - 1) for tcn in corners]
    return out, idx


def build(m, v, a, n_rev):
    """Timer commands for one trial with method m, and what they do."""
    p = m["period"]
    raw, parts = raw_profile(v, a, n_rev, p)
    q_raw = [speed_of_timer(n) for n in timers(raw, m["hyst"])]
    means = []
    for i0, i1 in parts:
        if i1 > i0:
            prev = q_raw[i0 - 1] if i0 > 0 else 0.0
            means.append(abs(q_raw[i1 - 1] - prev) / ((i1 - i0) * p))
    if m["smooth"] > 0:
        sp, corners = scurve(v, a, n_rev, p, m["smooth"])
    else:
        sp, corners = raw, [i1 - 1 for _, i1 in parts]
    cmds = timers(sp, m["hyst"])
    cmds.append(-HOLD)
    q = [speed_of_timer(n) for n in cmds]
    jumps = [abs(b - c) for c, b in zip([0.0] + q[:-1], q)]
    x, xmin, xmax = 0.0, 0.0, 0.0
    for sp in q:
        x += sp * p
        xmin, xmax = min(xmin, x), max(xmax, x)
    flips = sum(1 for c0, c1, c2 in zip(cmds, cmds[1:], cmds[2:]) if c0 == c2 and c0 != c1)
    mean = sum(means) / len(means) if means else 0.0
    top = min(max(abs(x) for x in q[max(c - 1, 0):c + 2]) for c in corners)   # lowest corner speed
    d = dict(top=top, mean=mean, jump=max(jumps), peak=max(jumps) / p, xmin=xmin, xmax=xmax,
             dur=len(cmds) * p, n=len(cmds), flips=flips,
             text=f"{n_rev} reversals +/-{top:.2f} m/s, mean {mean:.1f}")
    return cmds, d


# ---------------------------------------------------------------- sending
def send_wait(cart, cmds, p, tag):
    """Like the swing-up: send, wait for the Mega's step count, then wait out the period."""
    sent = []
    next_t = time.monotonic()
    for n in cmds:
        sent.append(time.monotonic())
        cart.send(n)
        cart.log.append((time.monotonic() - cart.t0, n, cart.steps, speed_of_timer(n), tag))
        next_t += p
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        elif dt < -5 * p:
            next_t = time.monotonic()           # fell behind (USB hiccup): do not burst to catch up
    return [(b - a) * 1000 for a, b in zip(sent, sent[1:])]


def send_stream(cart, cmds, p, tag):
    """Send one timer value per period without waiting for replies; read the replies as they come
    (keeps the step count current) and stop if the Mega goes silent."""
    ser = cart.ser
    ser.reset_input_buffer()
    sent, buf = [], bytearray()
    last_rx = next_t = time.monotonic()
    for n in cmds:
        now = time.monotonic()
        ser.write(struct.pack("<i", int(n)))
        sent.append(now)
        k = ser.in_waiting
        if k:
            buf.extend(ser.read(k))
            last_rx = now
            if len(buf) >= 2:
                end = len(buf) - len(buf) % 2
                cart.steps = struct.unpack(">h", bytes(buf[end - 2:end]))[0]
                del buf[:end]
        elif now - last_rx > 0.15:
            raise TimeoutError("the Mega stopped replying (limit switch or stop button?)")
        cart.log.append((now - cart.t0, n, cart.steps, speed_of_timer(n), tag))
        next_t += p
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        elif dt < -5 * p:
            next_t = time.monotonic()
    time.sleep(0.03)
    ser.reset_input_buffer()
    cart.send(-HOLD)                  # waits for a fresh reply: step count is current again
    return [(b - a) * 1000 for a, b in zip(sent, sent[1:])]


def run_trial(cart, m, cmds, tag):
    time.sleep(0.4)
    if m["send"] == "stream":
        gaps = send_stream(cart, cmds, m["period"], tag)
    else:
        gaps = send_wait(cart, cmds, m["period"], tag)
    cart.hold(0.3, tag)
    cart.go_to(0.0)
    if gaps:
        print(f"  sent every {statistics.mean(gaps):.2f} ms on average, longest gap {max(gaps):.1f} ms "
              f"(target {m['period'] * 1000:g} ms)")
        return statistics.mean(gaps), max(gaps)
    return 0.0, 0.0


def motor_back_on(cart, simulate):
    cart.close(motor_off=True)
    if not simulate:
        print("  Motor OFF. Slide the cart back to the tape mark by hand.")
        ask("  Press Enter to switch the motor back on and go on: ")
    cart.reenable()


# ---------------------------------------------------------------- search mode
def search_one(cart, m, args, link, simulate):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    lab = "".join(c if c.isalnum() else "_" for c in args.label)[:40]
    base = OUT / f"revm_{m['tag']}_{stamp}_{lab}"
    top = build(m, args.end_speed, 10, args.reversals)[1]["top"]
    print(f"\n==== Method {m['name']}: {m['text']}  (results -> {base.name}.csv)")
    search = Search(args.start_accel, args.max_accel)
    log0 = len(cart.log)
    rows, stop_reason, k = [], "", 0
    try:
        while not search.done:
            if k >= 30:
                search.done, search.note = True, "30 trials without a consistent edge (answers flip): no result"
                break
            k += 1
            a = search.next()
            cmds, d = build(m, args.end_speed, a, args.reversals)
            if max(-d["xmin"], d["xmax"]) > TRAVEL_LIMIT_M:
                search.done, search.note = True, f"travel over {TRAVEL_LIMIT_M} m at {a} m/s^2: use a lower --end-speed"
                break
            print(f"\n[{m['name']} {k}] {search.phase:6s} a = {a:5.2f} m/s^2 ({d['text']}, biggest speed jump {d['jump']:.3f} m/s)")
            g_mean, g_max = run_trial(cart, m, cmds, f"{m['name']}{k}")
            if simulate is not None:
                ans = "r" if a >= simulate else ""
                print(f"  (simulated answer: {ans or 'clean'})")
            else:
                ans = ask("  Enter = clean, r = red LED, b = buzz/grind/clunk, o = off the mark, q = stop: ", "").lower()[:1]
            if ans == "q":
                stop_reason = "stopped by user"
                break
            stalled = ans in ("r", "b", "o")
            rows.append([datetime.now().isoformat(timespec="seconds"), args.label, m["name"], m["tag"], args.added_kg,
                         f"{d['top']:.4f}", k, search.phase, f"{a:.2f}", f"{d['mean']:.2f}", f"{d['peak']:.2f}",
                         f"{g_mean:.2f}", f"{g_max:.1f}", "stall" if stalled else "clean", ans])
            search.report(a, stalled)
            if stalled:
                motor_back_on(cart, simulate is not None)
            lo = f"{search.lo:.2f}" if search.lo is not None else "-"
            hi = f"{search.hi:.2f}" if search.hi is not None else "-"
            print(f"  clean up to {lo}, stalls from {hi} m/s^2")
    except KeyboardInterrupt:
        stop_reason = "Ctrl+C"
        print("\nStopped.")
    except Exception as exc:
        stop_reason = f"{type(exc).__name__}: {exc}"
        print(f"\nFAULT: {stop_reason}")

    with open(f"{base}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "label", "method", "method_tag", "added_kg", "top_speed_m_s", "trial", "phase",
                    "a_target_m_s2", "a_mean_m_s2", "a_biggest_jump_m_s2", "sent_every_ms", "longest_gap_ms",
                    "result", "answer"])
        w.writerows(rows)
    with open(f"{base}_cycles.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "timer", "steps", "v_cmd_m_s", "trial"])
        w.writerows(cart.log[log0:])
    res = None
    if search.done and not search.note and search.result():
        a_lo = build(m, args.end_speed, search.lo, args.reversals)[1]["mean"]
        a_hi = build(m, args.end_speed, search.hi, args.reversals)[1]["mean"]
        res = ((a_lo + a_hi) / 2, max(abs(a_hi - a_lo) / 2, 0.25))
    summary = dict(label=args.label, method=m["name"], method_tag=m["tag"], method_text=m["text"],
                   period_ms=m["period"] * 1000, hyst=m["hyst"], smooth_ms=m["smooth"] * 1000, send=m["send"],
                   mega=args.mega, reversals=args.reversals, added_kg=args.added_kg, speed_target=args.end_speed,
                   top_speed=top, a_stall=res[0] if res else None, a_err=res[1] if res else None,
                   clean_up_to=search.lo, stalls_from=search.hi, trials=len(rows),
                   note=search.note or stop_reason, time=stamp,
                   link_round_trip_ms=dict(median=link[0], p95=link[1], max=link[2]))
    Path(f"{base}.json").write_text(json.dumps(summary, indent=1))
    if res:
        line = f"stalls at {res[0]:.2f} +/- {res[1]:.2f} m/s^2 (clean up to {search.lo:.2f}, stalls from {search.hi:.2f})"
    else:
        line = f"no result: {search.note or stop_reason} (clean up to {search.lo}, stalls from {search.hi})"
    print(f"RESULT method {m['name']} ({m['text']}): {line} at {top:.3f} m/s, {args.added_kg:.3f} kg added.")
    print(f"Saved {base.name}.csv, .json and _cycles.csv in results/")
    return (m, res, line), bool(stop_reason)          # q, Ctrl+C or a fault stops the remaining methods


# ---------------------------------------------------------------- fixed-acceleration mode
def fixed_mode(cart, methods, args, simulate):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    lab = "".join(c if c.isalnum() else "_" for c in args.label)[:40]
    path = OUT / f"revfix_{stamp}_{lab}.csv"
    built = {m["tag"]: build(m, args.end_speed, args.accel, args.reversals) for m in methods}
    rows = []
    try:
        for rep in range(1, args.repeats + 1):
            for m in methods:
                cmds, d = built[m["tag"]]
                print(f"\n[{rep}/{args.repeats}] method {m['name']}: {m['text']} at {args.accel} m/s^2 ({d['text']})")
                g_mean, g_max = run_trial(cart, m, cmds, f"{m['name']}{rep}")
                if simulate is not None:
                    ans = "r" if args.accel >= simulate else "0"
                    print(f"  (simulated score {ans})")
                else:
                    ans = ask("  Score: 0 smooth, 1 slight stutter, 2 clear stutter/skip, r red LED, x missed, q stop: ", "x").lower()
                code = ans[:1]
                if code == "q":
                    raise KeyboardInterrupt
                score = {"0": 0, "1": 1, "2": 2, "r": 3}.get(code)
                rows.append([datetime.now().isoformat(timespec="seconds"), args.label, m["name"], m["tag"], m["text"],
                             args.added_kg, f"{args.accel:.2f}", f"{d['mean']:.2f}", f"{d['top']:.4f}", rep,
                             f"{g_mean:.2f}", f"{g_max:.1f}", code, "" if score is None else score])
                if code == "r":
                    motor_back_on(cart, simulate is not None)
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as exc:
        print(f"\nFAULT: {type(exc).__name__}: {exc}")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "label", "method", "method_tag", "method_text", "added_kg", "a_target_m_s2", "a_mean_m_s2",
                    "top_speed_m_s", "repeat", "sent_every_ms", "longest_gap_ms", "answer", "score"])
        w.writerows(rows)
    with open(str(path).replace(".csv", "_cycles.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "timer", "steps", "v_cmd_m_s", "trial"])
        w.writerows(cart.log)
    print(f"\nSummary at {args.accel} m/s^2, {args.reversals} reversals +/-{args.end_speed} m/s "
          f"(score 0 smooth .. 2 clear stutter, red LED = 3):")
    for m in methods:
        sc = [r[13] for r in rows if r[3] == m["tag"] and r[13] != ""]
        reds = sum(1 for r in rows if r[3] == m["tag"] and r[12] == "r")
        if sc:
            print(f"  {m['name']:9s} {m['text']:42s} mean {statistics.mean(sc):.2f}  ({' '.join(map(str, sc))}; red LED {reds})")
    print(f"Saved {path.name} (and _cycles.csv) in results/. Paste this summary in the thread.")


# ---------------------------------------------------------------- summary of saved searches
def summary_table():
    files = sorted(OUT.glob("revm_*.json")) + sorted(OUT.glob("rev_*.json"))
    if not files:
        print("No finished searches in results/ yet.")
        return 1
    print(f"{'method':10s} {'how':44s} {'kg':>6s} {'speed':>6s} {'stall m/s^2':>14s}  label / file")
    for f in files:
        r = json.loads(f.read_text())
        name = r.get("method", "A (rev test)")
        how = r.get("method_text", "every 10 ms, wait (two_load_reversal_test)")
        st = f"{r['a_stall']:.2f} +/- {r['a_err']:.2f}" if r.get("a_stall") else "no result"
        print(f"{name:10s} {how:44s} {r.get('added_kg', 0):6.3f} {r.get('top_speed', 0):6.3f} {st:>14s}  "
              f"{r.get('label', '')} ({f.name})")
    print("Compare stall accelerations at the same added mass; a gap above about 1 m/s^2 is a real difference.")
    return 0


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--method", default="A", help="A..G, a list like A,E,G, or 'all'")
    ap.add_argument("--period", type=float, help="ms between commands (overrides the letter)")
    ap.add_argument("--smooth", type=float, help="ms the acceleration takes to flip (S-curve), 0 = instant (overrides the letter)")
    ap.add_argument("--hyst", action="store_true", help="add timer hysteresis")
    ap.add_argument("--send", choices=["wait", "stream"], help="wait for each Mega reply, or stream (default: wait at 10 ms and slower)")
    ap.add_argument("--mega", choices=["slave", "ramp"], default="slave", help="label only: which sketch is on the Mega")
    ap.add_argument("--accel", type=float, help="fixed acceleration instead of the stall search")
    ap.add_argument("--repeats", type=int, default=3, help="with --accel: trials per method")
    ap.add_argument("--added-kg", type=float, default=0.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--end-speed", type=float, default=0.8, help="reversal speed, m/s")
    ap.add_argument("--reversals", type=int, default=3)
    ap.add_argument("--start-accel", type=float, default=6.0)
    ap.add_argument("--max-accel", type=float, default=40.0)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--list", action="store_true", help="show what each method sends, no motion")
    ap.add_argument("--summary", action="store_true", help="table of every finished search in results/")
    ap.add_argument("--compute", action="store_true", help="mass + force from a bare and a loaded run of the same method")
    ap.add_argument("--simulate-stall-at", type=float, help=argparse.SUPPRESS)   # testing without a person
    args = ap.parse_args()

    if args.summary:
        return summary_table()
    letters = "ABCDEFG" if args.method.lower() == "all" else [s.strip().upper() for s in args.method.split(",") if s.strip()]
    bad = [s for s in letters if s not in TECHNIQUES]
    if bad or not letters:
        sys.exit(f"--method: use letters A..G (got {','.join(bad) or 'nothing'})")
    methods, seen = [], set()
    for s in letters:
        m = make_method(s, args)
        if m["tag"] not in seen:
            methods.append(m)
            seen.add(m["tag"])

    if args.compute:
        rc = 0
        for m in methods:
            print(f"==== Method {m['name']}: {m['text']}")
            rc |= compute(None, prefix=f"revm_{m['tag']}", where=f"after back-to-back reversals, method {m['name']}")
        return rc

    accels = [args.accel] if args.accel else [x for x in COARSE if args.start_accel <= x <= args.max_accel]
    for m in methods:
        print(f"Method {m['name']}: {m['text']}")
        for a in accels:
            cmds, d = build(m, args.end_speed, a, args.reversals)
            far = max(-d["xmin"], d["xmax"]) > TRAVEL_LIMIT_M
            if args.list or far:
                print(f"  a {a:5.1f} m/s^2: +/-{d['top']:.3f} m/s, mean {d['mean']:5.1f}, {d['n']:4d} commands, biggest jump "
                      f"{d['jump']:.3f} m/s ({d['peak']:5.1f} m/s^2 over one command), timer flip-flops {d['flips']:3d}, "
                      f"travel {d['xmin'] * 1000:+4.0f}..{d['xmax'] * 1000:+4.0f} mm, {d['dur']:.1f} s")
            if far and not args.list:
                sys.exit(f"That would travel more than {TRAVEL_LIMIT_M} m from the start. Use a lower --end-speed.")
    if args.list:
        return 0

    sim = args.simulate_stall_at
    mode = f"fixed {args.accel} m/s^2 x {args.repeats}" if args.accel else "stall search"
    print(f"\nReversal method test ({mode}), {args.reversals} reversals +/-{args.end_speed} m/s, "
          f"added mass {args.added_kg:.3f} kg.")
    print("Arms OFF. Cart on the tape mark, >= 0.3 m from each limit switch. Hands off the rail.")
    if sim is None:
        ask("Press Enter to start (Ctrl+C stops at any time): ")
    OUT.mkdir(exist_ok=True)
    cart = Cart(args.port).open()
    link = link_check(cart)
    print(f"Mega link: round trip median {link[0]:.1f} ms, 95 % {link[1]:.1f} ms, max {link[2]:.1f} ms")
    slow = [m for m in methods if m["send"] == "wait" and m["period"] * 1000 < link[1] * 1.5]
    for m in slow:
        print(f"  WARNING: method {m['name']} waits for replies every {m['period'] * 1000:g} ms but the round trip is "
              f"up to {link[1]:.1f} ms: it will run slower than asked (see 'sent every' after each trial).")
    results = []
    try:
        if args.accel:
            fixed_mode(cart, methods, args, sim)
        else:
            for m in methods:
                r, stop_all = search_one(cart, m, args, link, sim)
                results.append(r)
                if stop_all:
                    break
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        try:
            cart.close(motor_off=True)
        except Exception:
            pass
    if len(results) > 1:
        print("\nComparison:")
        for m, res, line in results:
            print(f"  {m['name']:9s} {m['text']:42s} {line}")
    if results:
        print("Paste this output in the thread. All searches so far: --summary")
    return 0


if __name__ == "__main__":
    sys.exit(main())
