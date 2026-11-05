"""Shared serial link to the Mega running ArduinoSlave.ino (or ArduinoSlaveRamp.ino).

Protocol (same as the team's scripts): every cycle the Pi sends one signed 32-bit little-endian
timer value N; the Mega answers with its 16-bit big-endian step count. Step rate = 125000/(|N|+1)
steps/s; N = +/-65535 is the "stopped" creep (1.9 steps/s); N = -1 (0xFFFFFFFF) switches the timer
off and disables the motor. Positive N = negative cart direction (repo convention).

The step count is the Mega's own count of pulses it sent. It does NOT see skipped steps, belt slip
or a UIM344 lockout: that is why the tests ask you to check a tape mark by eye.
"""
import math
import struct
import time

PORT, BAUD = "/dev/ttyACM0", 115200
DISTANCE_PER_6400_STEPS_M = 0.638175   # same constant as the swing-up scripts
M_PER_STEP = DISTANCE_PER_6400_STEPS_M / 6400
TIMER_STEP_HZ = 125000.0
T = 0.01                               # control period, s (same as the swing-up)
HOLD = 65535


def timer_for_speed(v):
    """Speed in m/s -> signed timer value (same sign convention as PythonMaster2_swingup_V2b)."""
    if abs(v) < 2.0 * M_PER_STEP:
        return None
    step_hz = -v / M_PER_STEP
    icr = max(1, min(65535, int(round(TIMER_STEP_HZ / abs(step_hz))) - 1))
    return icr if step_hz > 0 else -icr


def speed_of_timer(n):
    if n is None or abs(n) >= 65535 or n == 0:
        return 0.0
    return -math.copysign(TIMER_STEP_HZ / (abs(n) + 1), n) * M_PER_STEP


class Cart:
    def __init__(self, port=PORT, baud=BAUD):
        self.port_name, self.baud = port, baud
        self.ser = None
        self.steps = 0
        self.steps0 = 0
        self.log = []          # (t, timer, steps, v_cmd)
        self.t0 = time.monotonic()

    # ------------------------------------------------------------------ connection
    def open(self):
        import serial
        self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05)
        time.sleep(2.5)                     # opening the port resets the Mega
        self.ser.reset_input_buffer()
        self.steps0 = self.send(-HOLD)      # first command also enables the motor (ArduinoSlave setup)
        return self

    def close(self, motor_off=True):
        if self.ser is None:
            return
        try:
            if motor_off:
                self.ser.write(struct.pack("<i", -1))
                self.ser.flush()
        finally:
            self.ser.close()
            self.ser = None

    def reenable(self):
        """Motor off, then reset the Mega by reopening the port: its setup() disables and then
        re-enables the driver, which clears a UIM344 position-error lockout (red LED)."""
        self.close(motor_off=True)
        time.sleep(0.5)
        return self.open()

    # ------------------------------------------------------------------ one cycle
    def send(self, timer):
        self.ser.write(struct.pack("<i", int(timer)))
        data = bytearray()
        deadline = time.monotonic() + 0.1
        while len(data) < 2 and time.monotonic() < deadline:
            data.extend(self.ser.read(2 - len(data)))
        if len(data) != 2:
            raise TimeoutError("no reply from the Mega (limit switch or stop button hit? press the Mega "
                               "RESET button, slide the cart back to the mark, run again)")
        self.ser.reset_input_buffer()
        self.steps = struct.unpack(">h", bytes(data))[0]
        return self.steps

    def pos_m(self):
        d = ((self.steps - self.steps0 + 32768) & 0xFFFF) - 32768
        return d * M_PER_STEP

    def run_speeds(self, speeds, label=""):
        """Send one speed per 10 ms cycle, paced on the monotonic clock. Returns the cycles sent."""
        next_t = time.monotonic()
        for v in speeds:
            n = timer_for_speed(v)
            if n is None:
                n = -HOLD
            self.send(n)
            self.log.append((time.monotonic() - self.t0, n, self.steps, v, label))
            next_t += T
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            elif dt < -0.05:
                next_t = time.monotonic()   # fell behind (USB hiccup): do not burst to catch up
        return len(speeds)

    def hold(self, seconds, label="hold"):
        """Hold still around the current step count (the swing-up scripts' trick: alternate the
        creep direction toward the target)."""
        target = self.steps
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(HOLD if self.steps > target else -HOLD)
            self.log.append((time.monotonic() - self.t0, 0, self.steps, 0.0, label))
            time.sleep(T * 0.8)

    def go_to(self, x_target_m, v_max=0.15, a=1.0, label="return"):
        """Gentle move to a cart position (relative to the start), on the commanded step count."""
        dist = x_target_m - self.pos_m()
        if abs(dist) < 2 * M_PER_STEP:
            return
        self.run_speeds(trapezoid(dist, v_max, a), label)
        self.hold(0.3, label)


# ---------------------------------------------------------------------- profiles
def trapezoid(dist_m, v_max, a):
    """Speeds (one per 10 ms) for a move of dist_m with top speed v_max and acceleration a."""
    s = math.copysign(1.0, dist_m)
    d = abs(dist_m)
    t_acc = v_max / a
    if a * t_acc ** 2 > d:                 # triangle: never reaches v_max
        t_acc = math.sqrt(d / a)
        v_max = a * t_acc
    t_flat = max(0.0, (d - a * t_acc ** 2) / v_max)
    t_tot = 2 * t_acc + t_flat
    out, x = [], 0.0
    n = int(math.ceil(t_tot / T))
    for k in range(n):
        t = (k + 0.5) * T
        if t < t_acc:
            v = a * t
        elif t < t_acc + t_flat:
            v = v_max
        else:
            v = max(0.0, v_max - a * (t - t_acc - t_flat))
        out.append(s * v)
        x += v * T
    # small correction so the commanded distance is right (staircase rounding)
    err = d - x
    while err > M_PER_STEP:
        v = min(err / T, 0.05)
        out.append(s * v)
        err -= v * T
    out.append(0.0)
    return out


def peak_accel(speeds):
    return max((abs(b - a) / T for a, b in zip([0.0] + list(speeds), speeds)), default=0.0)
