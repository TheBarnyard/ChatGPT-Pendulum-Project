"""Run the real PythonMaster3_triple.py against a simulated rig on a virtual clock.

The script's serial ports, time, sleep and input() are replaced:
  * plant: Math/shared/nlink_cart.py triple (true arm values may differ from the script's model),
    cart = speed source changed by the Arduino commands;
  * receiver: protocol v1 frames every 10 ms, per-transmitter radio trace (rig-fitted FIFO model of
    2026-11-04, optional), encoder quantisation + noise, slots and signs as on the rig guess;
  * Arduino: ArduinoSlave command/reply timing, step counter;
  * hands: hold the arms for the calibrations, hold them upright (manual start) or bring them up
    (auto-start) and let go after a reaction time.
"""
import binascii
import importlib.util
import io
import json
import math
import struct
import sys
import types
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

sys.path.insert(0, '/home/user/Project-Unified-Codex/Math/shared')
import nlink_cart as nl  # noqa: E402

HERE = Path(__file__).resolve().parent
SCRIPT_DIR = Path('/mnt/project-files/TripleBalance_NewArms')
sys.path.insert(0, str(SCRIPT_DIR))
import angle_protocol  # noqa: E402
import angle_packet_fallback  # noqa: E402

RAD = 2 * math.pi / 16384
M_STEP = 0.638175 / 6400
TRUTH = {
 'new': [(0.165400, 0.169095, 3.58603e-4, 0.2), (0.174025, 0.209476, 6.69226e-4, 0.25), (0.132450, 0.228848, 9.38529e-4, 0.3)],
 'old': [(0.098136, 0.147803, 3.39766e-4, 0.2), (0.161803, 0.111518, 2.02321e-3, 0.25), (0.169850, 0.135413, 3.05751e-3, 0.3)],
 'oldarms12_newarm3': [(0.098136, 0.147803, 3.39766e-4, 0.2), (0.161803, 0.111518, 2.02321e-3, 0.25), (0.18265, 0.16595, 2.845e-3, 0.3)],
}
DAMP = [1.62994e-4, 1.23589e-4, 1.23589e-4]
ARM3 = {'full': (0.169850, 0.135413, 3.05751e-3), 'bare': (0.138350, 0.116183, 2.46953e-3)}
RIG_RADIO = dict(lam=[8, 3, 3], dmin=4, dmean=10, qmax=8, retries=6, fail=30, a0=0.3, a1=1.8)


class StopSim(KeyboardInterrupt):
    pass


def true_rig(arm3='full', scale=None):
    """scale: {arm index: {'m': x, 'I': x, 'c': +dm}}"""
    links = []
    for i, a in enumerate(TRUTH[arm3]):
        m, c, I, Lk = a
        s = (scale or {}).get(i, {})
        m *= s.get('m', 1.0)
        I *= s.get('I', 1.0)
        c += s.get('c', 0.0)
        links.append(nl.Link(m, c, Lk, I, DAMP[i]))
    r = nl.Rig(links)
    H, p = r.h(), r.p()
    r.h = lambda: H        # freeze (speed)
    r.p = lambda: p
    return r


def radio_trace(rng, T_end, lam, dmin, dmean, qmax, retries, fail, a0, a1, period=0.005):
    """One transmitter: array of (receive time, sample time); rig-fitted FIFO model (twin_rig.patch_radio_fifo)."""
    blk = []
    t = rng.exponential(1 / lam) if lam > 0 else 1e9
    while t < T_end + 1:
        d = (dmin + rng.exponential(dmean)) / 1000
        blk.append((t, t + d))
        t = t + d + rng.exponential(1 / lam)
    blk = np.array(blk) if blk else np.zeros((0, 2))

    def blocked(t):
        i = np.searchsorted(blk[:, 0], t) - 1 if len(blk) else -1
        return i >= 0 and t < blk[i, 1]

    attempt = lambda: (a0 + rng.exponential(a1)) / 1000
    ts = rng.uniform(0, period)
    q, out, tries, t_end_att = [], [], 0, np.inf
    while ts < T_end or q:
        if ts <= t_end_att and ts < T_end:
            if len(q) < qmax:
                q.append(ts)
            if t_end_att == np.inf:
                tries, t_end_att = 0, ts + attempt()
            ts += period
        else:
            t = t_end_att
            if t == np.inf:
                break
            if not (blocked(t) or rng.random() < fail / 100):
                out.append((t, q.pop(0)))
                tries = 0
            else:
                tries += 1
                if tries > retries:
                    q.pop(0)
                    tries = 0
            t_end_att = t + attempt() if q else np.inf
    return np.array(out) if out else np.zeros((0, 2))


class Sim:
    def __init__(self, rig, seed=0, noise_counts=1.0, radio=None, T_end=30.0, enc_err_deg=(0, 0, 0),
                 slots=(1, 3, 2), signs=(1, 1, 1), ard_delay=0.0015, compute=0.0015, usb=0.0005,
                 extra_delay=0.0, start_lean_deg=(0, 0, 0), start_rate=(0, 0, 0), release_s=0.05,
                 auto_path_s=2.0, auto_from_deg=(8, -6, 5), auto_err_deg=(0.0, 0.0, 0.0), push=None,
                 stall_accel=None):
        self.rig, self.n = rig, 3
        self.rng = np.random.default_rng(seed)
        self.noise, self.T_end = noise_counts, T_end
        self.enc_err = np.radians(enc_err_deg)
        self.slots, self.signs = slots, signs
        self.ard_delay, self.compute, self.usb, self.extra = ard_delay, compute, usb, extra_delay
        self.zero = self.rng.integers(0, 16384, 3)
        self.t = 0.0
        self.z = np.zeros(8)
        self.z[0:6:2] = np.pi
        self.hold = ('pin', self.z[0:6:2].copy(), np.zeros(3), 0.0)   # hand: angles pinned
        self.hist_t, self.hist_rel = [0.0], [self.rel()]
        self.pending = []          # (time, speed)
        self.motor_enabled = False
        self.steps_offset = int(self.rng.integers(-3000, 3000))
        self.start_lean, self.start_rate = np.radians(start_lean_deg), np.array(start_rate, float)
        self.release_s = release_s
        self.auto_path_s, self.auto_from, self.auto_err = auto_path_s, np.radians(auto_from_deg), np.radians(auto_err_deg)
        self.prompts = 0
        self.control_started_at = None
        self.push = push or []
        self.stall_accel = stall_accel
        self.stalled = False
        self.max = dict(x=0.0, v=0.0, th=0.0)
        self.trace = []
        self.radio = None
        if radio:
            self.radio = [radio_trace(self.rng, T_end + 60, radio['lam'][i], radio['dmin'], radio['dmean'], radio['qmax'],
                                      radio['retries'], radio['fail'], radio['a0'], radio['a1']) for i in range(3)]
        self.frame_t = 0.0037       # next receiver frame time
        self.esp_buf = bytearray()
        self.seq = 0
        self.ard_buf = bytearray()
        self.ard_ready = []        # (time, bytes)
        self.v_now = 0.0
        self.last_v_cmd = 0.0

    # ---------------- plant
    def rel(self):
        th = self.z[0:6:2]
        return np.array([th[0], th[1] - th[0], th[2] - th[1]])

    def f(self, z):
        return nl.accel_dynamics(self.rig, z, 0.0)

    def hand_angles(self, t):
        kind = self.hold[0]
        if kind == 'pin':
            return self.hold[1], np.zeros(3)
        if kind == 'path':     # auto-start: bring the arms from auto_from to auto_err over auto_path_s
            t0 = self.hold[3]
            s = min(max((t - t0) / self.auto_path_s, 0.0), 1.0)
            sm = s * s * (3 - 2 * s)
            dsm = 6 * s * (1 - s) / self.auto_path_s if 0 < s < 1 else 0.0
            # overshoot slowly past the target: a human hunting for the balance point
            wob = np.radians(1.5) * np.sin(2 * np.pi * 0.7 * (t - t0)) * np.array([1, -1, 1]) * (1 - sm * 0.5)
            dwob = np.radians(1.5) * 2 * np.pi * 0.7 * np.cos(2 * np.pi * 0.7 * (t - t0)) * np.array([1, -1, 1]) * (1 - sm * 0.5)
            th = self.auto_from * (1 - sm) + self.auto_err * sm + wob
            w = (self.auto_err - self.auto_from) * dsm + dwob
            return th, w
        return None, None

    def advance(self, t_new):
        h = 0.0005
        while self.t < t_new - 1e-12:
            t_next = min(t_new, self.t + h)
            if self.pending and self.pending[0][0] < t_next:
                t_next = max(self.pending[0][0], self.t)
            dt = t_next - self.t
            th_h, w_h = self.hand_angles(t_next)
            if th_h is not None:
                self.z[0:6:2], self.z[1:6:2] = th_h, w_h
                self.z[6] += self.z[7] * dt
            elif dt > 0:
                self.z = nl.rk4(lambda zz, aa: nl.accel_dynamics(self.rig, zz, aa), self.z, 0.0, dt)
            self.t = t_next
            for (tp, kick) in self.push:
                if self.t - dt < tp <= self.t:
                    self.z[5] += kick
            while self.pending and self.pending[0][0] <= self.t + 1e-12:
                _, v = self.pending.pop(0)
                if self.stall_accel is not None and abs(v - self.z[7]) / 0.005 > self.stall_accel:
                    self.stalled = True
                if self.hold[0] is None:
                    self.z = nl.speed_jump(self.rig, self.z, v - self.z[7])
                else:
                    self.z[7] = v
            self.hist_t.append(self.t)
            self.hist_rel.append(self.rel())
            if len(self.hist_t) > 2000:
                del self.hist_t[:-1000]
                del self.hist_rel[:-1000]
            if self.hold[0] is None:
                self.max['x'] = max(self.max['x'], abs(self.z[6]))
                self.max['v'] = max(self.max['v'], abs(self.z[7]))
                self.max['th'] = max(self.max['th'], float(np.max(np.abs(nl.wrap(self.z[0:6:2])))))
        self.t = max(self.t, t_new)
        if self.release_at is not None and self.t >= self.release_at and self.hold[0] is not None:
            self.hold = (None,)
            self.release_at = None
        if self.t > self.T_end and not self.stopped:
            self.stopped = True
            raise StopSim()

    release_at = None
    stopped = False

    # ---------------- clock
    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.advance(self.t + max(dt, 0.0))

    # ---------------- receiver
    def make_frame(self, tf):
        Ht = np.array(self.hist_t)
        Hr = np.array(self.hist_rel)
        angles, statuses, ages = [0] * 3, [0] * 3, [0] * 3
        for phys in range(3):
            slot = self.slots[phys] - 1
            if self.radio is not None:
                tr = self.radio[phys]
                j = np.searchsorted(tr[:, 0], tf) - 1
                rx, ts = (tr[j] if j >= 0 else (-1.0, -1.0))
                age = int(round((tf - rx) * 1000)) if j >= 0 else 0xFFFFFFFF
            else:
                ts, age = tf - 0.003, 3
            if age == 0xFFFFFFFF or age > 50:
                angles[slot], statuses[slot], ages[slot] = 0xFFFF, (3 if age == 0xFFFFFFFF else 2), age
                continue
            val = np.interp(ts, Ht, Hr[:, phys]) if ts >= Ht[0] else Hr[0, phys]
            val = nl.wrap(val + self.enc_err[phys] * (1 if self.hold_calib else 0))
            cnt = int(np.round(self.zero[phys] - self.signs[phys] * val / RAD + self.noise * self.rng.standard_normal())) % 16384
            angles[slot], statuses[slot], ages[slot] = cnt, 0, age
        payload = struct.pack('>II', self.seq & 0xFFFFFFFF, int(tf * 1000) & 0xFFFFFFFF)
        for i in range(3):
            payload += struct.pack('>HBI', angles[i], statuses[i], ages[i])
        body = bytes((1, 29)) + payload
        crc = binascii.crc_hqx(body, 0xFFFF)
        self.seq += 1
        return angle_protocol.MAGIC + body + crc.to_bytes(2, 'big')

    hold_calib = False

    def esp_fill(self):
        while self.frame_t + self.usb <= self.t + 1e-9:
            self.esp_buf.extend(self.make_frame(self.frame_t))
            self.frame_t += 0.01

    def esp_read(self, n):
        self.esp_fill()
        if not self.esp_buf:
            self.advance(min(self.t + 0.003, self.frame_t + self.usb))
            self.esp_fill()
        out = bytes(self.esp_buf[:n])
        del self.esp_buf[:n]
        return out

    # ---------------- Arduino
    def ard_write(self, data):
        self.ard_buf.extend(data)
        while len(self.ard_buf) >= 4:
            cmd = struct.unpack('<i', bytes(self.ard_buf[:4]))[0]
            del self.ard_buf[:4]
            self.advance(self.t + self.compute)       # the script's computing time before the write
            if cmd == -1:
                v = 0.0
                self.motor_enabled = False
            else:
                self.motor_enabled = True
                v = -math.copysign(125000.0 / (abs(cmd) + 1), cmd) * M_STEP
                if cmd not in (65535, -65535) and self.control_started_at is None and self.mode == 'auto':
                    self.control_started_at = self.t
                    self.release_at = self.t + self.release_s
            self.pending.append((self.t + self.ard_delay + self.extra, v))
            self.pending.sort()
            steps = int(round(self.z[6] / M_STEP)) + self.steps_offset
            steps = (steps + 32768) % 65536 - 32768
            self.ard_ready.append((self.t + 0.0008, struct.pack('>h', steps)))
        return len(data)

    def ard_avail(self):
        out = bytearray()
        for tr, b in self.ard_ready:
            if tr <= self.t:
                out.extend(b)
        return out

    def ard_read(self, n):
        if not self.ard_ready:
            self.advance(self.t + 0.003)
            return b''
        tr, b = self.ard_ready[0]
        if tr > self.t:
            self.advance(min(tr, self.t + 0.003))
        if tr <= self.t:
            self.ard_ready.pop(0)
            return b[:n]
        return b''

    # ---------------- hands (input prompts)
    mode = 'manual'
    countdown = 0.0

    def on_input(self, prompt):
        self.prompts += 1
        if 'stopped swinging' in prompt:       # hanging calibration: arms hang (with the encoder-zero error)
            self.hold = ('pin', np.array([np.pi, np.pi, np.pi]), np.zeros(3), self.t)
            self.hold_calib = True
            return ''
        self.hold_calib = False
        if 'countdown' in prompt:              # countdown: arms held at start_lean, let go at GO + release_s
            th = self.start_lean.copy()
            self.hold = ('pin', th, np.zeros(3), self.t)
            self.release_at = self.t + self.countdown + self.release_s
            self._release_rate = self.start_rate
            return ''
        if 'lift the arms' in prompt:          # auto-start
            self.mode = 'auto'
            self.hold = ('path', None, None, self.t)
            return ''
        if 'upright' in prompt:                # manual: arms held upright, let go release_s after Enter
            th = self.start_lean.copy()
            self.hold = ('pin', th, np.zeros(3), self.t)
            self.release_at = self.t + self.release_s
            self._release_rate = self.start_rate
            return ''
        return ''


def run_case(arm3_model='full', arm3_true='full', scale=None, seed=0, noise=1.0, radio=None, T_end=25.0,
             enc_err_deg=(0, 0, 0), args=(), extra_delay=0.0, start_lean_deg=(0.3, -0.2, 0.2), release_s=0.05,
             auto=False, auto_err_deg=(0.0, 0.0, 0.0), push=None, stall_accel=None, keep_log=False):
    rig = true_rig(arm3_true, scale)
    sim = Sim(rig, seed=seed, noise_counts=noise, radio=radio, T_end=T_end, enc_err_deg=enc_err_deg,
              extra_delay=extra_delay, start_lean_deg=start_lean_deg, release_s=release_s,
              auto_err_deg=auto_err_deg, push=push, stall_accel=stall_accel)
    fake_time = types.SimpleNamespace(monotonic=sim.monotonic, sleep=sim.sleep, time=sim.monotonic)

    class Port:
        def __init__(self, port, baud, timeout=None, write_timeout=None):
            self.port, self.is_open = port, True
            self.kind = 'esp' if 'USB' in port else 'ard'

        @property
        def in_waiting(self):
            if self.kind == 'esp':
                sim.esp_fill()
                return len(sim.esp_buf)
            return len(sim.ard_avail())

        def read(self, n=1):
            return sim.esp_read(n) if self.kind == 'esp' else sim.ard_read(n)

        def write(self, data):
            return sim.ard_write(data)

        def reset_input_buffer(self):
            if self.kind == 'esp':
                sim.esp_fill()
                sim.esp_buf.clear()
            else:
                sim.ard_ready = [(t, b) for t, b in sim.ard_ready if t > sim.t]

        def flush(self):
            pass

        def close(self):
            self.is_open = False

    fake_serial = types.SimpleNamespace(Serial=Port, SerialException=OSError)
    spec = importlib.util.spec_from_file_location(f'pm3_{seed}_{id(sim)}', SCRIPT_DIR / 'PythonMaster3_triple_NewArms.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.time = fake_time
    angle_protocol.time = fake_time
    angle_packet_fallback.time = fake_time
    mod.serial = fake_serial
    mod.input = sim.on_input
    mod.RUN_LOG_DIRECTORY = Path('/tmp/claude-0/-home-user/f4568911-2e30-5d0a-8b4f-eb46450cab15/scratchpad/w/tri/simlogs')
    mod.SAVE_RUN_CSV = keep_log
    mod.started_at = 0.0

    orig_release = sim.advance

    def advance(t_new):
        # apply the hand's release rate at the moment of release (manual start)
        before = sim.hold[0]
        orig_release(t_new)
        if before is not None and sim.hold[0] is None and hasattr(sim, '_release_rate'):
            sim.z[1:6:2] = sim._release_rate
    sim.advance = advance

    argv = ['PythonMaster3_triple.py', *args]
    if '--countdown' in args:
        sim.countdown = float(args[list(args).index('--countdown') + 1])
    old_argv = sys.argv
    sys.argv = argv
    buf = io.StringIO()
    try:
        with redirect_stdout(buf):
            try:
                mod.main()
            except SystemExit:
                pass
            except Exception as exc:   # noqa: BLE001
                print(f'[HARNESS] exception {type(exc).__name__}: {exc}')
    finally:
        sys.argv = old_argv
    out = buf.getvalue()
    stop = [ln for ln in out.splitlines() if '[FAULT]' in ln or 'Ctrl+C' in ln or '[HARNESS]' in ln]
    bal = [ln for ln in out.splitlines() if 'balanced ' in ln]
    held = bool(bal) and not any('[FAULT]' in ln or '[HARNESS]' in ln for ln in stop)
    res = dict(held=held, stop=stop[:2], max_x_mm=sim.max['x'] * 1000, max_v=sim.max['v'],
               max_th_deg=math.degrees(sim.max['th']), stalled=sim.stalled, balanced=bal[-1][-200:] if bal else '')
    if keep_log:
        res['log'] = out
    return res


if __name__ == '__main__':
    r = run_case(keep_log=True, T_end=12.0)
    print('\n'.join(r['log'].splitlines()[:40]))
    print('...')
    print('\n'.join(r['log'].splitlines()[-12:]))
    print({k: v for k, v in r.items() if k != 'log'})
