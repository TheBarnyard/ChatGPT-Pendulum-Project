"""usage: python3 make_v2b_d.py PythonMaster2_swingup_V2b.py PythonMaster2_swingup_V2b_D.py
Make the V2b_D script from PythonMaster2_swingup_V2b.py (which stays unchanged): cart speed commands
sent the "method D" way from the cart-jolt bench tests (every 5 ms, without waiting for the reply)."""
import re, sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()


def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (s.count(old), old[:80])
    s = s.replace(old, new)


# ---------------------------------------------------------------- header
rep('''# PythonMaster2_swingup_V2b.py: PythonMaster2_swingup_V2.py with a radio-aware swing observer
''', '''# PythonMaster2_swingup_V2b_D.py: PythonMaster2_swingup_V2b.py with the cart commands sent the way
# Barney found smoothest on the bench (2026-11-12, CartJolt/reversal_method_test.py method D): every
# 5 ms, without waiting for the Arduino's reply. PythonMaster2_swingup_V2b.py is not changed.
# V2b_D changes only:
#   - each 10 ms cycle sends its speed change in SENDS_PER_CYCLE equal parts (2 = 5 ms apart), so
#     the cart speed changes in steps half as big; --sends picks 1, 2 or 5 (every 10, 5 or 2 ms),
#   - --accel-max sets CART_ACCEL_MAX_M_S2 from the command line (default unchanged, 16),
#   - the Arduino's replies (step counts) are read whenever they have arrived (class CartStream);
#     no reply for REPLY_TIMEOUT_S stops the run, as before,
#   - the model, the swing tracking gains and the balance gains know the speed lands in parts,
#   - the stop brake is sent the same way.
# Same plans, observer, balance weights, recovery, limits and logging as V2b.
# PythonMaster2_swingup_V2b.py: PythonMaster2_swingup_V2.py with a radio-aware swing observer
''')

# ---------------------------------------------------------------- settings
rep('''# Used for the swing only; the balance uses BALANCE_LATENCY_MS.
CONTROL_LATENCY_MS = 8.0
''', '''# Used for the swing only; the balance uses BALANCE_LATENCY_MS.
# V2b_D: this is the delay of the FIRST part of each speed change; the later parts follow it
# SAMPLE_PERIOD_S / SENDS_PER_CYCLE apart.
CONTROL_LATENCY_MS = 8.0

# V2b_D (2026-11-12): how the speed commands go to the Arduino. On the cart-jolt bench tests
# (reversal_method_test.py, 2026-11-11/12) sending every 5 ms without waiting for the reply
# ("method D") felt the smoothest and most consistent. Each 10 ms control cycle sends the first part
# of its speed change at once and the rest in equal parts, 10 ms / SENDS_PER_CYCLE apart.
#   2 = method D (every 5 ms).  5 = method E (every 2 ms).  1 = every 10 ms, still without waiting.
# Also settable with --sends. The model and all gains are worked out for the value used.
SENDS_PER_CYCLE = 2
# The Arduino answers every command with its step count. If the oldest unanswered command is older
# than this, s, the run stops (limit switch or stop button pressed, or the Mega reset).
REPLY_TIMEOUT_S = 0.15
# Time from writing a command to the Arduino acting on it, ms (half the 4.1 ms USB round trip
# measured on 2026-11-12). Dates each step count, so the cart position can be carried forward.
REPLY_HALF_TRIP_MS = 2.0
''')

# ---------------------------------------------------------------- model: the change lands in parts
rep('''def step(z, u_prev, u, r, T=SAMPLE_PERIOD_S, tau=None):
    """One control period with the stepper's staircase speed. u is the speed change decided at this
    sample, divided by T (an equivalent acceleration); it reaches the cart tau after the sample. If tau
    is longer than one period, the previous sample's change lands in this period instead."""
    tau = CONTROL_LATENCY_MS / 1000.0 if tau is None else tau
    if tau < T:
        z = coast(z, tau, r)
        z = kick(z, u * T, r)
        return coast(z, T - tau, r)
    t1 = tau - T
    z = coast(z, t1, r)
    z = kick(z, u_prev * T, r)
    return coast(z, T - t1, r)
''', '''def step(z, u_prev, u, r, T=SAMPLE_PERIOD_S, tau=None):
    """One control period with the stepper's staircase speed. u is the speed change decided at this
    sample, divided by T (an equivalent acceleration). V2b_D: it is sent in SENDS_PER_CYCLE equal
    parts, T / SENDS_PER_CYCLE apart, and each part reaches the cart tau after it is sent. Parts that
    land after this period's end land in the next one (there, as u_prev)."""
    tau = CONTROL_LATENCY_MS / 1000.0 if tau is None else tau
    n = max(1, int(SENDS_PER_CYCLE))
    events = []
    for i in range(n):
        t = tau + i * T / n
        if t < T:
            events.append((t, u))
        else:
            events.append((min(t - T, T), u_prev))
    events.sort(key=lambda e: e[0])
    t0 = 0.0
    for t, uu in events:
        z = coast(z, t - t0, r)
        z = kick(z, uu * T / n, r)
        t0 = t
    return coast(z, T - t0, r)
''')

# the plan's speed changes must land at the parts' average time, not the first part's
rep('''    N = len(z) - 1
    tau = CONTROL_LATENCY_MS / 1000.0
    T = d["dt"]
''', '''    N = len(z) - 1
    T = d["dt"]
    # V2b_D: the parts of each change land on average (n - 1) T / 2n after the first one
    n = max(1, int(SENDS_PER_CYCLE))
    tau = CONTROL_LATENCY_MS / 1000.0 + (n - 1) * T / (2 * n)
''')

# ---------------------------------------------------------------- the streaming link
rep('''def signed_count_delta(raw, reference):
''', '''def speed_of_timer(timer):
    """Cart speed, m/s, that a timer command gives (0 for the two hold commands)."""
    if timer in (65535, -65535):
        return 0.0
    return -math.copysign(TIMER_STEP_HZ / (abs(timer) + 1), timer) * M_PER_STEP


class CartStream:
    """V2b_D: speed commands to the Arduino without waiting for its replies (method D). The Arduino
    answers every command with 2 bytes, its step count when the command arrived; they are read
    whenever they have come in, and the newest one gives the cart position."""

    def __init__(self, port, steps):
        self.port = port
        self.buf = bytearray()
        self.pending = deque()           # (time sent, speed) of commands not answered yet
        self.steps = steps
        self.t_count = time.monotonic()  # when self.steps was counted
        self.v_count = 0.0               # speed the cart ran at from then on
        self.sent = 0
        self.late_ms = 0.0               # how late the worst timed send was, ms
        self.late_count = 0              # timed sends more than 2 ms late

    def send(self, timer):
        now = time.monotonic()
        self.port.write(struct.pack("<i", int(timer)))
        self.pending.append((now, speed_of_timer(timer)))
        self.sent += 1
        self.poll()
        return now

    def send_at(self, t, timer):
        dt = t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        else:
            self.late_ms = max(self.late_ms, -dt * 1000.0)
            self.late_count += dt < -0.002
        return self.send(timer)

    def poll(self):
        """Read the replies that have arrived; returns the newest step count."""
        k = self.port.in_waiting
        if k:
            self.buf.extend(self.port.read(k))
        n = len(self.buf) // 2
        if n:
            self.steps = struct.unpack(">h", bytes(self.buf[2 * n - 2:2 * n]))[0]
            del self.buf[:2 * n]
            answered = None
            for _ in range(n):
                if self.pending:
                    answered = self.pending.popleft()
            if answered is not None:
                self.t_count = answered[0] + REPLY_HALF_TRIP_MS / 1000.0
                self.v_count = answered[1]
        elif self.pending and time.monotonic() - self.pending[0][0] > REPLY_TIMEOUT_S:
            raise TimeoutError(f"{self.port.port}: received no reply to a speed command for "
                               f"{REPLY_TIMEOUT_S * 1000:.0f} ms (limit switch or stop button? reset the Arduino)")
        return self.steps

    def travel_since_count(self, now):
        """Cart travel since self.steps was counted, from the speeds sent after it, m."""
        t, v, dx = self.t_count, self.v_count, 0.0
        for ts, vs in self.pending:
            ta = ts + REPLY_HALF_TRIP_MS / 1000.0
            if ta >= now:
                break
            if ta > t:
                dx += v * (ta - t)
                t = ta
            v = vs
        return dx + v * max(0.0, now - t)

    def drain(self):
        """Wait for the replies still on their way and drop them (before going back to send-and-wait)."""
        time.sleep(0.02)
        self.port.reset_input_buffer()
        self.buf.clear()
        self.pending.clear()


def signed_count_delta(raw, reference):
''')

# ---------------------------------------------------------------- run loop
rep('''    esp32 = arduino = None
    run_logger = None
''', '''    esp32 = arduino = None
    link = None                    # V2b_D: CartStream once the Arduino has answered
    run_logger = None
''')

rep('''        steps, t_reply, cart_m = steps0, time.monotonic(), 0.0
        log("START", f"Arduino replied: step count {steps0} is the cart's zero")
''', '''        steps, cart_m = steps0, 0.0
        link = CartStream(arduino, steps0)
        log("START", f"Arduino replied: step count {steps0} is the cart's zero")
        log("START", f"cart commands: {SENDS_PER_CYCLE} per {SAMPLE_PERIOD_S * 1000:.0f} ms cycle, every "
                     f"{SAMPLE_PERIOD_S * 1000 / SENDS_PER_CYCLE:.1f} ms, without waiting for replies (V2b_D)")
''')

rep('''            cart_now = cart_m + v_sent * max(0.0, time.monotonic() - t_reply)
''', '''            # V2b_D: newest step count from the replies that have arrived, carried forward with the
            # speeds sent since it was counted
            steps_before = steps
            steps = link.poll()
            jump = ((steps - steps_before + 32768) & 0xFFFF) - 32768
            if abs(jump) > MAX_STEPS_PER_REPLY:
                raise RuntimeError(f"step count jumped {jump:+d} steps in one cycle ({steps_before} -> {steps}); the cart "
                                   "cannot move that fast, so the count is wrong. Check the D5 -> D18 wire, make sure "
                                   "ModemManager is off, press the Mega's RESET button and run again.")
            cart_m = (steps - steps0) * M_PER_STEP
            cart_now = cart_m + link.travel_since_count(time.monotonic())
''')

rep('''            timer, step_hz = timer_for_speed(v_cmd, cart_now - hold_x, deadband)
            v_sent = 0.0 if timer in (65535, -65535) else -math.copysign(TIMER_STEP_HZ / (abs(timer) + 1), timer) * M_PER_STEP
            arduino.write(struct.pack("<i", int(timer)))
            brake["v"] = v_sent
            steps_before = steps if steps is not None else steps0
            steps = struct.unpack(">h", read_exact(arduino, 2))[0]
            arduino.reset_input_buffer()
            t_reply = time.monotonic()
            jump = ((steps - steps_before + 32768) & 0xFFFF) - 32768
            if abs(jump) > MAX_STEPS_PER_REPLY:
                raise RuntimeError(f"step count jumped {jump:+d} steps in one cycle ({steps_before} -> {steps}); the cart "
                                   "cannot move that fast, so the count is wrong. Check the D5 -> D18 wire, make sure "
                                   "ModemManager is off, press the Mega's RESET button and run again.")
            cart_m = (steps - steps0) * M_PER_STEP
''', '''            timer, step_hz = timer_for_speed(v_cmd, cart_now - hold_x, deadband)
            v_sent = speed_of_timer(timer)
            # V2b_D: the change goes out in SENDS_PER_CYCLE equal parts; the first one now, the rest
            # at the end of this cycle, SAMPLE_PERIOD_S / SENDS_PER_CYCLE apart (method D)
            parts = [timer_for_speed(v_state + (v_cmd - v_state) * i / SENDS_PER_CYCLE, cart_now - hold_x, deadband)[0]
                     for i in range(1, SENDS_PER_CYCLE)] + [timer]
            t_first = link.send(parts[0])
            brake["v"] = speed_of_timer(parts[0])
''')

rep('''                log("RUN", f"{mode:<7} arm1={math.degrees(wrap(th1)):+7.1f}deg arm2={math.degrees(wrap(th2)):+7.1f}deg "
                           f"cart={cart_m * 1000:+6.0f}mm v={v_cmd:+.2f}m/s a={a:+6.1f}m/s^2{extra}")
                next_status = now + STATUS_INTERVAL_S
''', '''                log("RUN", f"{mode:<7} arm1={math.degrees(wrap(th1)):+7.1f}deg arm2={math.degrees(wrap(th2)):+7.1f}deg "
                           f"cart={cart_m * 1000:+6.0f}mm v={v_cmd:+.2f}m/s a={a:+6.1f}m/s^2{extra}")
                next_status = now + STATUS_INTERVAL_S

            # ---- V2b_D: the rest of this cycle's speed change, on time
            for i, part in enumerate(parts[1:], 1):
                link.send_at(t_first + i * SAMPLE_PERIOD_S / SENDS_PER_CYCLE, part)
                brake["v"] = speed_of_timer(part)
''')

# ---------------------------------------------------------------- stop: brake streamed, then drain
rep('''                    try:
                        v = brake["v"]
                        while abs(v) > 0.005:
                            v -= math.copysign(min(abs(v), STOP_BRAKE_M_S2 * SAMPLE_PERIOD_S), v)
                            timer, _ = timer_for_speed(v, 0.0, 0.0)
                            arduino.write(struct.pack("<i", int(timer)))
                            read_exact(arduino, 2)
                            arduino.reset_input_buffer()
                            time.sleep(SAMPLE_PERIOD_S * 0.8)
                        log("STOP", f"cart braked to a stop from {brake['v']:+.2f} m/s")
                    except Exception as exc:
                        log("FAULT", f"could not brake the cart: {exc}")
''', '''                    try:
                        v = brake["v"]
                        if link is not None:
                            # V2b_D: brake with the same streamed commands, SENDS_PER_CYCLE per cycle
                            dt_b = SAMPLE_PERIOD_S / max(1, SENDS_PER_CYCLE)
                            t_next = time.monotonic()
                            while abs(v) > 0.005:
                                v -= math.copysign(min(abs(v), STOP_BRAKE_M_S2 * dt_b), v)
                                timer, _ = timer_for_speed(v, 0.0, 0.0)
                                t_next += dt_b
                                link.send_at(t_next, timer)
                        else:
                            while abs(v) > 0.005:
                                v -= math.copysign(min(abs(v), STOP_BRAKE_M_S2 * SAMPLE_PERIOD_S), v)
                                timer, _ = timer_for_speed(v, 0.0, 0.0)
                                arduino.write(struct.pack("<i", int(timer)))
                                read_exact(arduino, 2)
                                arduino.reset_input_buffer()
                                time.sleep(SAMPLE_PERIOD_S * 0.8)
                        log("STOP", f"cart braked to a stop from {brake['v']:+.2f} m/s")
                    except Exception as exc:
                        log("FAULT", f"could not brake the cart: {exc}")
                if link is not None and motor_command_sent and "received" not in run_stop_reason:
                    try:
                        link.drain()               # back to send-and-wait for the hold below
                        log("STOP", f"cart commands sent: {link.sent}; {link.late_count} timed sends more than 2 ms late "
                                    f"(worst {link.late_ms:.1f} ms)")
                    except Exception as exc:
                        log("FAULT", f"could not drain the Arduino replies: {exc}")
''')

# ---------------------------------------------------------------- command line
rep('''    ap = argparse.ArgumentParser(description="Double pendulum swing-up and balance (V2 arms).")
''', '''    ap = argparse.ArgumentParser(description="Double pendulum swing-up and balance (V2 arms), cart commands every 5 ms (method D).")
    ap.add_argument("--sends", type=int, choices=(1, 2, 5),
                    help="cart commands per 10 ms cycle: 2 = every 5 ms (method D, default), 5 = every 2 ms (E), 1 = every 10 ms")
    ap.add_argument("--accel-max", type=float,
                    help="hardest cart acceleration ever sent, m/s^2 (default CART_ACCEL_MAX_M_S2 = 16; the motor stalled at 16-18 in swing runs)")
''')
rep('''    args = ap.parse_args()
    if args.list:
''', '''    args = ap.parse_args()
    if args.sends:
        SENDS_PER_CYCLE = args.sends
    if args.accel_max:
        CART_ACCEL_MAX_M_S2 = args.accel_max
    if args.list:
''')

assert "read_exact(arduino, 2))[0]\n            arduino.reset_input_buffer()\n            t_reply" not in s
assert "t_reply" not in s, "t_reply left over"
open(dst, "w", encoding="utf-8").write(s)
print(f"wrote {dst} ({len(s.splitlines())} lines)")
