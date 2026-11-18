"""PythonMaster3_triple.py - balance the TRIPLE pendulum (V2 arms + arm 3) on the cart.

A new script. It is PythonMaster2_auto_improved.py (the double balance that held 12.8 min on the rig)
written for three arms. No team file is changed. Same helpers (angle_protocol.py,
angle_packet_fallback.py, run_csv_logger.py), same ports, same prompts:
    1. motor power OFF, cart centred, ALL THREE arms hanging still  -> hanging calibration
    2. motor power ON, hold all three arms upright, press Enter       -> balancing starts
       (or --countdown 3: press Enter, hold the arms with both hands, let go on GO)
Ctrl+C stops the motor.

Other modes (no balancing):
    --check        motor stays off. Prints the three receiver slots and the arm/joint angles live,
                   so you can confirm which slot is joint 2 and joint 3, and joint 3's sign.
    --log 120      logs all angles (and the cart) for 120 s while you swing the arms by hand and let
                   them swing freely. The motor holds the cart still (or --no-motor: motor power off,
                   hold the cart by hand). Used to measure arm 3 for the model.
    --design-only  prints the model and the gains, opens no ports.

Differences from the double script
  * Three angles. Arm 1 from receiver slot 1; joint 2 and joint 3 from the slots in
    RECEIVER_SLOTS (on 2026-11-04 the joint-2 transmitter sent as slot 3). Check with --check.
  * The model is built from each arm's mass, centre of mass and inertia (physical form). A triple
    tolerates about +/-10-20 % error in real arm values, but under 1 % error in a freely fitted
    ratio, so ratios are not used here (PUC Math/Triple/03_balance.md, 3.6).
  * No start ramp by default: a triple falls about twice as fast as the double and the ramp cuts the
    largest lean it can catch to about a third.
  * The cart is braked to a stop before the motor is switched off at the end.

Arm values: arms 1 and 2 = V2 arms (the 2026-11-04 rig fit, run excite_20261104_093436).
Arm 3 = arm 2's design 50 mm longer [HYP, not weighed]. If you know arm 3's mass and centre of mass,
give them with --arm3-mass-g and --arm3-com-mm (and --arm3-period-s, the swing period of arm 3 alone
hanging from joint 3, if you timed it). In simulation the balance still holds with arm 3's mass
-20..+20 % and its centre of mass +/-20 mm off, and with arm 3's inertia -15..+40 %.

Quick tuning guide (same knobs as the double):
  cart jitters or buzzes while balanced   -> ENCODER_NOISE_COUNTS up, or LQR_CART_ACCEL_M_S2 down
  oscillation at a few Hz that grows       -> CONTROL_LATENCY_MS up 1-2 ms (the triple only tolerates
                                              about 5 ms more delay than designed)
  stepper skips or the driver LED goes red -> CART_ACCEL_MAX_M_S2 down (but the triple needs a lot)
  cart parks off centre at the start       -> pass the printed trims with --offsets

CSV: run_logs/balance_triple_*.csv (balance), run_logs/freeswing_triple_*.csv (--log).
The "force" columns hold the cart acceleration command in m/s^2.
"""

import argparse
import json
import math
import struct
import sys
import time
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
from scipy.linalg import expm, solve_discrete_are

from run_csv_logger import RunCsvLogger
from angle_protocol import TELEMETRY_COLUMNS, AnglePacketReader, AngleProtocolError, packet_csv_values
from angle_packet_fallback import MissingAnglePacketError, collect_valid_samples


# =====================================================================================
# 1. ARM MODEL (physical values; index 0 = arm 1 at the cart, 2 = arm 3 at the top)
# =====================================================================================
# Mass of each arm alone, kg: everything that swings with it except the arms above it (rod,
# weights, the transmitter board and battery on its hub, the next joint's shaft parts).
ARM_MASS_KG = [0.098136, 0.161803, 0.169850]
# Distance from the arm's own pivot to its centre of mass, m.
ARM_COM_M = [0.147803, 0.111518, 0.135413]
# Pivot to the next joint, m (arm 3: pivot to tip; it does not enter the dynamics).
ARM_LENGTH_M = [0.200, 0.250, 0.300]
# Moment of inertia of each arm about its OWN centre of mass, kg m^2.
ARM_INERTIA_KG_M2 = [3.39766e-4, 2.02321e-3, 3.05751e-3]
# Viscous friction at the main pivot, joint 2 and joint 3, N m s/rad (relative rates).
JOINT_FRICTION = [1.62994e-4, 1.23589e-4, 1.23589e-4]

# Arm 3 guesses [HYP]: "full" = arm 3 carries the same hub electronics as arm 2 (XIAO, LiPo,
# encoder board, switch) at its tip; "bare" = without them. (mass kg, CoM m, inertia about CoM kg m^2)
ARM3_PRESETS = {
    "full": (0.169850, 0.135413, 3.05751e-3),
    "bare": (0.138350, 0.116183, 2.46953e-3),
}
ARM3_PRESET = "full"

G = 9.81   # m/s^2

# =====================================================================================
# 2. LQR WEIGHTS (Bryson's rule, as in the double script)
# =====================================================================================
LQR_ARM_ANGLE_DEG = [2.0, 2.0, 2.0]   # arm 1, 2, 3 absolute angle from upright that counts as large
LQR_ARM_RATE_DEG_S = 60.0             # any arm's angular speed that counts as large
LQR_CART_M = 0.05                     # cart distance from centre that counts as large
LQR_CART_SPEED_M_S = 0.3              # cart speed that counts as large
LQR_CART_ACCEL_M_S2 = 1.3             # the calm-vs-stiff knob (cost on the acceleration command)

# Angle measured -> motor reacting, ms. The triple holds up to about 14 ms of real delay with 9 ms
# designed (simulation); the double held to 22 ms.
CONTROL_LATENCY_MS = 9.0
SAMPLE_PERIOD_S = 0.01

# =====================================================================================
# 3. ESTIMATOR
# =====================================================================================
# Encoder noise, counts. 6 here (double: 3): it also covers the radio's timing jitter. With the rig's
# radio timing (2026-11-04 fit) the simulated balance held a median 22 s with 3 and 60 s+ with 6.
ENCODER_NOISE_COUNTS = 6.0
# For the catch, the first CATCH_FILTER_S seconds use a quicker filter (the double's 3 counts): in
# simulation 6 from the start lost catches of arm 3 leaning 1.5 deg.
CATCH_ENCODER_NOISE_COUNTS = 3.0
CATCH_FILTER_S = 2.0
ARM_DISTURBANCE_RAD_S2 = 1.0
# Radio timing. Each frame carries how long ago the receiver got each transmitter's reading. On the
# rig about 1 in 9 arm-1 readings is 10 ms or older (2026-11-04), i.e. a repeat of the last frame's.
# The triple only tolerates about 5 ms of extra delay, so repeats are skipped (the prediction fills in)
# and fresh readings are moved forward by (age - NOMINAL_AGE_MS) times the estimated joint speed.
SKIP_STALE = True
STALE_AGE_MS = 10
AGE_CORRECTION = True
NOMINAL_AGE_MS = 3.0

# =====================================================================================
# 4. STANDSTILL: AUTO-TRIM AND STEP DITHER (as in the double script)
# =====================================================================================
AUTO_TRIM = True
TRIM_TIME_CONSTANT_S = 6.0
TRIM_START_DELAY_S = 1.0
TRIM_MAX_DEG = 3.0
# Starting offsets, degrees (positive subtracts from the reported angle): arm 1, joint 2, joint 3.
# The script prints the trims it found at the end; pass them with --offsets next run.
START_OFFSETS_DEG = [0.0, 0.0, 0.0]
SPEED_DEADBAND_M_S = 0.0003

# =====================================================================================
# 5. LIMITS, START AND SAFETY
# =====================================================================================
CART_SPEED_MAX_M_S = 0.8
# The motor stalled at 16-18 m/s^2 reversals on the rig (2026-11-05). A 1 degree lean of arm 3
# already asks for about 14 m/s^2, so this cannot go much lower without losing the catch.
CART_ACCEL_MAX_M_S2 = 15.0
# The motor stops if the cart gets this far from where it started, m. The track is about +/-540 mm
# between the limit switches (2026-11-04). The triple needs more room than the double: an encoder
# zero 0.15 deg off makes the cart drift 150-250 mm before the auto-trim finds it.
TRACK_SOFT_LIMIT_M = 0.40
MAX_RUNNING_ANGLE_DEG = 25.0
START_RAMP_S = 0.0                # 0 = full correction from the first cycle (see the top)
MAX_START_ANGLE_DEG = 10.0        # every arm and joint must be within this when balancing starts
COUNTDOWN_S = 0                   # --countdown 3: the motor holds the cart, counts down, starts at GO
STOP_BRAKE_M_S2 = 10.0            # deceleration used to stop the cart at the end
HANGING_SAMPLES = 50
UPRIGHT_SAMPLES = 3               # frames read after Enter before balancing starts (30 ms)
SAMPLE_SPACING_S = 0.02
# Arms must hang this still (counts) during the hanging calibration. Tighter than the double's 80:
# the triple only tolerates about 0.15 deg error in the joint references (simulation).
MAX_HANGING_DEVIATION_COUNTS = 25
MAX_RUNNING_CONSECUTIVE_MISSES = 8   # per angle; the receiver itself marks a reading missing after 50 ms
MAX_SETUP_CONSECUTIVE_MISSES = 5
MAX_HANGING_TOTAL_MISSES = 5
MAX_UPRIGHT_TOTAL_MISSES = 5

# =====================================================================================
# 6. HARDWARE AND OUTPUT
# =====================================================================================
ESP32_PORT, ESP32_BAUD = "/dev/ttyUSB0", 921600
ARDUINO_PORT, ARDUINO_BAUD = "/dev/ttyACM0", 115200
# Receiver slots (1-3) for: arm 1 encoder, joint 2 encoder (on arm 1's hub), joint 3 encoder (on
# arm 2's hub). Slot = transmitter SENDER_ID + 1. Override with --slots, check with --check.
RECEIVER_SLOTS = [1, 3, 2]
# Direction of each encoder: +1 or -1. Arm 1 and joint 2 are the double script's convention. If
# --check shows joint 3 counting the opposite way to joint 2 for the same bend, use --joint3-sign -1.
ENCODER_SIGNS = [1, 1, 1]
DISTANCE_PER_6400_STEPS_M = 0.638175
STATUS_INTERVAL_S = 0.5
SAVE_RUN_CSV = True
RUN_LOG_DIRECTORY = Path(__file__).resolve().parent / "run_logs"

# -------------------------------------------------------------------------------------
ENCODER_COUNTS = 16384
HALF_TURN = ENCODER_COUNTS // 2
RAD_PER_COUNT = 2 * math.pi / ENCODER_COUNTS
M_PER_STEP = DISTANCE_PER_6400_STEPS_M / 6400
TIMER_STEP_HZ = 125000.0
N_ARMS = 3

TRIPLE_COLUMNS = (
    "record_type", "elapsed_s", "cycle", "esp32_packet",
    "raw_slot_1", "raw_slot_2", "raw_slot_3",
    "raw_arm1", "raw_joint2", "raw_joint3",
    "theta1_deg", "theta2_deg", "theta3_deg", "joint2_deg", "joint3_deg",
    "cart_used_m", "cart_steps", "cart_m",
    "est_theta1_rad", "est_w1_rad_s", "est_theta2_rad", "est_w2_rad_s", "est_theta3_rad", "est_w3_rad_s",
    "est_cart_m", "est_cart_velocity_m_s",
    "accel_command_m_s2", "cart_velocity_command_m_s", "step_frequency_hz", "motor_timer_command",
    "trim_arm1_deg", "trim_joint2_deg", "trim_joint3_deg", "missing_channels",
    "sensor_read_ms", "position_read_ms", "loop_ms", "cycle_period_ms",
    *TELEMETRY_COLUMNS, "message",
)

started_at = time.monotonic()
esp32_packet_count = 0
angle_reader = AnglePacketReader()
latest_angle_packet = None
latest_angle_message = ""
latest_raw = (None, None, None)


def log(tag, message):
    print(f"[{time.monotonic() - started_at:7.2f}s] [{tag}] {message}", flush=True)


# ------------------------------------------------------------------ model and gains
def arm_values():
    """(mass, com, length, inertia, friction) lists for the three arms, arm 3 from ARM3_PRESET."""
    m, c, L, I = list(ARM_MASS_KG), list(ARM_COM_M), list(ARM_LENGTH_M), list(ARM_INERTIA_KG_M2)
    return m, c, L, I, list(JOINT_FRICTION)


def physical_matrices(m, c, L, I):
    """p (first moments) and H (inertia matrix at upright) of an n-arm chain on a cart."""
    n = len(m)
    p = np.array([m[j] * c[j] + L[j] * sum(m[j + 1:]) for j in range(n)])
    H = np.zeros((n, n))
    for i in range(n):
        H[i, i] = I[i] + m[i] * c[i] ** 2 + L[i] ** 2 * sum(m[i + 1:])
        for j in range(i + 1, n):
            H[i, j] = H[j, i] = L[i] * p[j]
    return p, H


def friction_matrix(b):
    n = len(b)
    Bf = np.zeros((n, n))
    for i in range(n):
        Bf[i, i] += b[i]
        if i > 0:
            Bf[i, i - 1] -= b[i]
            Bf[i - 1, i] -= b[i]
            Bf[i - 1, i - 1] += b[i]
    return Bf


def continuous_model(m, c, L, I, b):
    """x = [th1, w1, th2, w2, th3, w3, cart, cart_speed]; input = cart acceleration. Absolute angles
    from upright in the double script's sign convention: H dw = g diag(p) th - Bf w - p a."""
    n = len(m)
    p, H = physical_matrices(m, c, L, I)
    Hi = np.linalg.inv(H)
    Ath, Aw, Bth = G * Hi @ np.diag(p), -Hi @ friction_matrix(b), -Hi @ p
    A = np.zeros((2 * n + 2, 2 * n + 2))
    B = np.zeros((2 * n + 2, 1))
    for i in range(n):
        A[2 * i, 2 * i + 1] = 1.0
        for j in range(n):
            A[2 * i + 1, 2 * j] = Ath[i, j]
            A[2 * i + 1, 2 * j + 1] = Aw[i, j]
        B[2 * i + 1, 0] = Bth[i]
    A[2 * n, 2 * n + 1] = 1.0
    B[2 * n + 1, 0] = 1.0
    return A, B, p, H


def discretize_with_delay(A, B, T, tau):
    """x[k+1] = Phi x[k] + G0 u[k] + G1 u[k-1] for an input that takes effect tau after the sample."""
    n = A.shape[0]

    def zoh(t):
        Mx = np.zeros((n + 1, n + 1))
        Mx[:n, :n], Mx[:n, n:] = A, B
        E = expm(Mx * t)
        return E[:n, :n], E[:n, n:]
    tau = min(max(tau, 0.0), T)
    Phi, _ = zoh(T)
    E1, G0 = zoh(T - tau)
    _, Gt = zoh(tau)
    return Phi, G0, E1 @ Gt


def kalman_gain(Phi, n, T, noise_counts):
    """Steady-state Kalman filter gain: three absolute angles and the cart position measured."""
    nz = 2 * n + 2
    C = np.zeros((n + 1, nz))
    for i in range(n):
        C[i, 2 * i] = 1.0
    C[n, 2 * n] = 1.0
    w = ARM_DISTURBANCE_RAD_S2 * T
    qn = []
    for _ in range(n):
        qn += [(w * T / 2) ** 2, w ** 2]
    qn += [(M_PER_STEP / 4) ** 2, (M_PER_STEP / T / 4) ** 2]
    s_ang = noise_counts * RAD_PER_COUNT
    # absolute angle i is the sum of i encoder readings: noisier by about sqrt(i)
    rn = [(s_ang * math.sqrt(i + 1)) ** 2 for i in range(n)] + [(M_PER_STEP / math.sqrt(12)) ** 2]
    Pk = solve_discrete_are(Phi.T, C.T, np.diag(qn), np.diag(rn))
    return Pk @ C.T @ np.linalg.inv(C @ Pk @ C.T + np.diag(rn)), C


def design():
    m, c, L, I, b = arm_values()
    A, B, p, H = continuous_model(m, c, L, I, b)
    n = N_ARMS
    nz = 2 * n + 2
    T, tau = SAMPLE_PERIOD_S, CONTROL_LATENCY_MS / 1000.0
    Phi, G0, G1 = discretize_with_delay(A, B, T, tau)
    Az = np.block([[Phi, G1], [np.zeros((1, nz)), np.zeros((1, 1))]])
    Bz = np.vstack([G0, [[1.0]]])
    q = []
    for i in range(n):
        q += [1 / math.radians(LQR_ARM_ANGLE_DEG[i]) ** 2, 1 / math.radians(LQR_ARM_RATE_DEG_S) ** 2]
    q += [1 / LQR_CART_M ** 2, 1 / LQR_CART_SPEED_M_S ** 2, 0.0]
    Q = np.diag(q)
    R = np.array([[1 / LQR_CART_ACCEL_M_S2 ** 2]])
    P = solve_discrete_are(Az, Bz, Q, R)
    K = np.linalg.solve(R + Bz.T @ P @ Bz, Bz.T @ P @ Az)
    Lk, C = kalman_gain(Phi, n, T, ENCODER_NOISE_COUNTS)
    Lc, _ = kalman_gain(Phi, n, T, CATCH_ENCODER_NOISE_COUNTS)
    cl = np.abs(np.linalg.eigvals(Az - Bz @ K))
    return {"arms": {"mass_kg": m, "com_m": c, "length_m": L, "inertia_kg_m2": I, "friction": b},
            "p": p, "H": H, "A": A, "B": B, "Phi": Phi, "G0": G0, "G1": G1, "C": C, "K": K, "L": Lk, "L_catch": Lc,
            "closed_loop_max_abs_eig": float(cl.max()), "open_loop_eig": np.linalg.eigvals(A)}


def print_design(d):
    arms = d["arms"]
    for i in range(N_ARMS):
        log("MODEL", f"arm {i + 1}: mass {arms['mass_kg'][i] * 1000:.1f} g, CoM {arms['com_m'][i] * 1000:.1f} mm, "
                     f"length {arms['length_m'][i] * 1000:.0f} mm, inertia(CoM) {arms['inertia_kg_m2'][i]:.4e} kg m^2")
    fall = sorted([e.real for e in d["open_loop_eig"] if e.real > 1e-6], reverse=True)
    log("MODEL", "falls with time constants " + ", ".join(f"{1000 / e:.0f}" for e in fall) + " ms")
    K = d["K"][0]
    per_deg = [-math.radians(1) * K[2 * i] for i in range(N_ARMS)]
    log("GAINS", "cart accel per degree of lean (arm 1, 2, 3): " + ", ".join(f"{g:+.1f}" for g in per_deg) + " m/s^2")
    log("GAINS", f"K = {np.round(K, 3).tolist()} (for [th1, w1, th2, w2, th3, w3, x, v, a_prev]); "
                 f"closed-loop |eig| max {d['closed_loop_max_abs_eig']:.4f}")


# ------------------------------------------------------------------ sensors and motor
def read_exact(port, count, timeout_s=0.1):
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    if len(data) != count:
        raise TimeoutError(f"{port.port}: received {len(data)}/{count} bytes in {timeout_s:.2f}s "
                           "(limit switch or stop button pressed? reset the Arduino)")
    return bytes(data)


def read_frame(esp32):
    """Newest receiver frame -> (raw arm 1, raw joint 2, raw joint 3) in counts (0xFFFF = missing)."""
    global esp32_packet_count, latest_angle_packet, latest_angle_message, latest_raw
    latest_angle_packet = None
    try:
        packet = angle_reader.read_latest(esp32)
    except AngleProtocolError as exc:
        latest_raw = (None, None, None)
        latest_angle_message = str(exc)
        raise MissingAnglePacketError(str(exc)) from exc
    latest_angle_packet = packet
    esp32_packet_count += 1
    idx = [s - 1 for s in RECEIVER_SLOTS]
    latest_angle_message = packet.missing_message(tuple(idx))
    latest_raw = tuple(packet.angles[i] for i in idx)
    return latest_raw


def read_required(esp32):
    raw = read_frame(esp32)
    missing = [str(i + 1) for i in range(N_ARMS) if raw[i] == 0xFFFF]
    if missing:
        raise MissingAnglePacketError(f"angle(s) {', '.join(missing)} missing (arm1/joint2/joint3 = slots "
                                      f"{RECEIVER_SLOTS}); {latest_angle_message}")
    return raw


def signed_count_delta(raw, reference):
    return (raw - reference + HALF_TURN) % ENCODER_COUNTS - HALF_TURN


def circular_mean_counts(readings):
    phase = np.asarray(readings, dtype=float) * RAD_PER_COUNT
    return (np.angle(np.mean(np.exp(1j * phase))) % (2 * np.pi)) / RAD_PER_COUNT


def calibrate_hanging(esp32):
    """References in counts: arm 1 upright (hanging + half a turn), joint 2 and joint 3 straight."""
    log("CAL", f"Reading {HANGING_SAMPLES} frames with all three arms hanging")
    readings, misses = collect_valid_samples(
        lambda: read_required(esp32), HANGING_SAMPLES, interval_s=SAMPLE_SPACING_S,
        max_total_misses=MAX_HANGING_TOTAL_MISSES, max_consecutive_misses=MAX_SETUP_CONSECUTIVE_MISSES)
    if misses:
        log("CAL", f"Skipped {misses} unusable receiver frame(s)")
    readings = np.asarray(readings, dtype=float)
    refs = []
    for ch in range(N_ARMS):
        ref = circular_mean_counts(readings[:, ch])
        largest = float(np.max(np.abs(signed_count_delta(readings[:, ch], ref))))
        log("CAL", f"{('arm1', 'joint2', 'joint3')[ch]} (slot {RECEIVER_SLOTS[ch]}): mean={ref:.1f} "
                   f"first={int(readings[0, ch])} last={int(readings[-1, ch])} largest_deviation={largest:.1f} counts")
        if largest > MAX_HANGING_DEVIATION_COUNTS:
            raise RuntimeError(f"{('Arm 1', 'Joint 2', 'Joint 3')[ch]} moved during the hanging calibration "
                               f"({largest:.1f} counts, limit {MAX_HANGING_DEVIATION_COUNTS}); stop the swinging with a "
                               "light touch, wait until the arms are completely still, and restart")
        refs.append(ref)
    refs[0] = (refs[0] + HALF_TURN) % ENCODER_COUNTS
    log("CAL", f"arm1 upright={refs[0]:.1f}, joint2 straight={refs[1]:.1f}, joint3 straight={refs[2]:.1f} counts")
    return refs


def relative_angles(raw, refs, offs):
    """Arm 1 angle from upright and the two joint angles, radians (double script's convention)."""
    return [-ENCODER_SIGNS[i] * signed_count_delta(raw[i], refs[i]) * RAD_PER_COUNT - offs[i] for i in range(N_ARMS)]


def absolute_angles(rel):
    out, s = [], 0.0
    for r in rel:
        s += r
        out.append(s)
    return out


def check_upright(esp32, refs, offs):
    raw_samples, misses = collect_valid_samples(
        lambda: read_required(esp32), UPRIGHT_SAMPLES, interval_s=0.0,
        max_total_misses=MAX_UPRIGHT_TOTAL_MISSES, max_consecutive_misses=MAX_SETUP_CONSECUTIVE_MISSES)
    rel = np.median(np.asarray([relative_angles(r, refs, offs) for r in raw_samples]), axis=0)
    th = absolute_angles(rel)
    log("CHECK", f"arm1={math.degrees(th[0]):+.2f} arm2={math.degrees(th[1]):+.2f} arm3={math.degrees(th[2]):+.2f}deg "
                 f"(joint2={math.degrees(rel[1]):+.2f} joint3={math.degrees(rel[2]):+.2f})")
    if max(abs(v) for v in list(th) + list(rel)) > math.radians(MAX_START_ANGLE_DEG):
        raise RuntimeError(f"Upright check failed (limit {MAX_START_ANGLE_DEG} deg): hold all three arms upright "
                           "and restart. No motor command was sent.")
    return th


def send_motor_command(arduino, timer_command):
    payload = struct.pack("<i", int(timer_command))
    if arduino.write(payload) != len(payload):
        raise OSError(f"Arduino accepted fewer than {len(payload)} command bytes")


def read_steps(arduino):
    steps = struct.unpack(">h", read_exact(arduino, 2))[0]
    arduino.reset_input_buffer()
    return steps


def timer_for_speed(v, hold_error_m):
    """Timer value for cart speed v (m/s). Inside the deadband the cart holds a position with the
    slowest command pointed back toward it (as in the double script)."""
    if abs(v) < SPEED_DEADBAND_M_S:
        return (65535 if hold_error_m > 0 else -65535), 0.0
    step_hz = -v / M_PER_STEP
    icr = max(1, min(65535, int(round(TIMER_STEP_HZ / abs(step_hz))) - 1))
    return (icr if step_hz > 0 else -icr), step_hz


def sent_speed(timer_command):
    if timer_command in (65535, -65535):
        return 0.0
    return -math.copysign(TIMER_STEP_HZ / (abs(timer_command) + 1), timer_command) * M_PER_STEP


def open_ports(need_arduino=True):
    esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.003)
    arduino = None
    if need_arduino:
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.003, write_timeout=0.2)
        log("START", "receiver and Arduino ports open; waiting 3 s for the Arduino")
        time.sleep(3)
        arduino.reset_input_buffer()
    else:
        log("START", "receiver port open (motor not used)")
        time.sleep(0.5)
    esp32.reset_input_buffer()
    return esp32, arduino


def settings_snapshot():
    return {k: v for k, v in globals().items() if k.isupper() and isinstance(v, (int, float, str, bool, list, type(None)))}


# ------------------------------------------------------------------ --check
def run_check():
    """Live angle display, motor never commanded."""
    esp32, _ = open_ports(need_arduino=False)
    try:
        log("CHECK", "Motor is not used. Let all three arms hang still.")
        input("Press Enter when the arms have stopped swinging: ")
        angle_reader.discard_pending(esp32)
        refs = calibrate_hanging(esp32)
        log("CHECK", "Now move the arms by hand. Expected:")
        log("CHECK", "  - bend ONLY arm 3 (arm 2 still): only joint3 changes. If joint2 changes instead, swap the "
                     "slots: --slots " + ",".join(str(s) for s in (RECEIVER_SLOTS[0], RECEIVER_SLOTS[2], RECEIVER_SLOTS[1])))
        log("CHECK", "  - bend arm 2 relative to arm 1 one way (say clockwise seen from the front), then arm 3 relative "
                     "to arm 2 the same way: joint2 and joint3 must get the SAME sign. If not: --joint3-sign "
                     f"{-ENCODER_SIGNS[2]}")
        log("CHECK", "  - all arms straight up together: arm1, arm2, arm3 all near 0. Ctrl+C to end.")
        next_print = 0.0
        while True:
            try:
                raw = read_frame(esp32)
            except MissingAnglePacketError as exc:
                log("CHECK", f"no frame: {exc}")
                time.sleep(0.2)
                continue
            if time.monotonic() < next_print:
                continue
            next_print = time.monotonic() + 0.2
            pkt = latest_angle_packet
            slots = " ".join(f"s{i + 1}={'----' if a == 0xFFFF else a:>5}" for i, a in enumerate(pkt.angles))
            if any(r == 0xFFFF for r in raw):
                log("CHECK", f"{slots} | missing: {latest_angle_message}")
                continue
            rel = relative_angles(raw, refs, [0.0, 0.0, 0.0])
            hang = [rel[0] + math.pi] + rel[1:]          # arm 1 from hanging, joints from straight
            hang[0] = (hang[0] + math.pi) % (2 * math.pi) - math.pi
            th = absolute_angles(rel)
            log("CHECK", f"{slots} | from hanging: arm1={math.degrees(hang[0]):+7.1f} joint2={math.degrees(rel[1]):+7.1f} "
                         f"joint3={math.degrees(rel[2]):+7.1f} | from upright: arm1={math.degrees(th[0]):+7.1f} "
                         f"arm2={math.degrees(th[1]):+7.1f} arm3={math.degrees(th[2]):+7.1f} deg | "
                         f"ages {'/'.join(str(a) if a != 0xFFFFFFFF else '-' for a in pkt.ages_ms)} ms")
    except KeyboardInterrupt:
        log("STOP", "check ended")
    finally:
        esp32.close()


# ------------------------------------------------------------------ --log
def run_freeswing_log(seconds, use_motor):
    """Log all angles at 100 Hz while the arms swing freely (for fitting arm 3)."""
    esp32, arduino = open_ports(need_arduino=use_motor)
    run_logger = None
    stop_reason = "program exit"
    motor_on = False
    cycle = 0
    try:
        log("READY", "Motor power OFF; centre the cart and let ALL THREE arms hang still")
        input("Press Enter when the arms have stopped swinging: ")
        angle_reader.discard_pending(esp32)
        refs = calibrate_hanging(esp32)
        filename = datetime.now().astimezone().strftime("freeswing_triple_%Y%m%d_%H%M%S_%f.csv")
        run_logger = RunCsvLogger(RUN_LOG_DIRECTORY / filename, columns=TRIPLE_COLUMNS)
        run_logger.start()
        run_logger.event("config", json.dumps({"script": "PythonMaster3_triple.py", "mode": "freeswing_log",
                                               "references_counts": refs, "settings": settings_snapshot()},
                                              separators=(",", ":")), 0.0)
        if use_motor:
            log("READY", "Turn motor power ON (the motor will hold the cart still)")
        else:
            log("READY", "Motor not used: hold the cart still by hand or block it")
        input(f"Press Enter to start logging for {seconds:.0f} s: ")
        log("LOG", "Logging. Lift the arms (one, two or all three, 20-90 deg) and let go; let them swing "
                   "freely for 10-20 s each time. Do not touch the cart. Ctrl+C ends early.")
        steps = steps_zero = 0
        hold_x = 0.0
        if use_motor:
            send_motor_command(arduino, -65535)
            motor_on = True
            steps = steps_zero = read_steps(arduino)
        angle_reader.discard_pending(esp32)
        t0 = time.monotonic()
        next_status = t0
        while time.monotonic() - t0 < seconds:
            cycle += 1
            t_c = time.monotonic()
            try:
                raw = read_frame(esp32)
            except MissingAnglePacketError:
                raw = (None, None, None)
            cart_m = 0.0
            timer = ""
            if use_motor:
                cart_m = (steps - steps_zero) * M_PER_STEP
                timer, _ = timer_for_speed(0.0, cart_m - hold_x)
                send_motor_command(arduino, timer)
                steps = read_steps(arduino)
            ok = all(r is not None and r != 0xFFFF for r in raw)
            if ok:
                rel = relative_angles(raw, refs, [0.0, 0.0, 0.0])
                th = absolute_angles(rel)
                deg = [math.degrees(v) for v in th] + [math.degrees(rel[1]), math.degrees(rel[2])]
            else:
                deg = [""] * 5
            pkt = latest_angle_packet
            run_logger.sample((
                "sample", t_c - t0, cycle, esp32_packet_count,
                *(pkt.angles if pkt is not None else ("", "", "")), *[("" if r is None else r) for r in raw],
                *deg, cart_m, (steps - steps_zero) if use_motor else "", cart_m,
                *([""] * 8), 0.0, 0.0, 0.0, timer, 0.0, 0.0, 0.0,
                "" if ok else "missing", "", "", (time.monotonic() - t_c) * 1000, "",
                *packet_csv_values(pkt, angle_reader), "" if ok else latest_angle_message))
            if t_c >= next_status:
                if ok:
                    hang1 = (deg[0] + 360.0) % 360.0 - 180.0
                    log("LOG", f"{t_c - t0:5.0f} s  from hanging: arm1={hang1:+7.1f} joint2={deg[3]:+7.1f} "
                               f"joint3={deg[4]:+7.1f} deg")
                else:
                    log("LOG", f"{t_c - t0:5.0f} s  missing: {latest_angle_message}")
                next_status = t_c + 2.0
        stop_reason = "time up"
    except KeyboardInterrupt:
        stop_reason = "Ctrl+C"
        log("STOP", "logging ended by Ctrl+C")
    except Exception as exc:
        stop_reason = f"{type(exc).__name__}: {exc}"
        log("FAULT", stop_reason)
        traceback.print_exc()
    finally:
        if arduino is not None and arduino.is_open:
            if motor_on:
                try:
                    send_motor_command(arduino, -1)
                    arduino.flush()
                    log("STOP", "Sent Arduino stop command")
                except (serial.SerialException, OSError) as exc:
                    log("FAULT", f"Could not send stop command: {exc}")
            arduino.close()
        esp32.close()
        if run_logger is not None:
            warning = run_logger.stop(stop_reason)
            log("STOP", warning or f"CSV saved: {run_logger.path} ({cycle} cycles)")


# ------------------------------------------------------------------ balance
def run_balance(countdown):
    global angle_reader
    angle_reader = AnglePacketReader()
    d = design()
    print_design(d)
    Phi, G0, G1, C, K, L = d["Phi"], d["G0"], d["G1"], d["C"], d["K"], d["L"]
    L_catch = d["L_catch"]
    n, nz = N_ARMS, 2 * N_ARMS + 2
    T = SAMPLE_PERIOD_S

    esp32 = arduino = None
    motor_command_sent = False
    stage = "opening serial ports"
    cycle = 0
    run_logger = None
    run_started_at = None
    run_stop_reason = "program exit"
    steps = None
    last_command = None
    v_state = 0.0
    offs = [math.radians(v) for v in START_OFFSETS_DEG]
    try:
        esp32, arduino = open_ports(need_arduino=True)

        stage = "hanging calibration"
        log("READY", "Motor power OFF; centre the cart and let ALL THREE arms hang still")
        input("Press Enter when the arms have stopped swinging: ")
        angle_reader.discard_pending(esp32)
        refs = calibrate_hanging(esp32)
        log("CAL", "starting offsets (deg): arm1 {:+.3f} joint2 {:+.3f} joint3 {:+.3f}".format(*START_OFFSETS_DEG))

        if SAVE_RUN_CSV:
            filename = datetime.now().astimezone().strftime("balance_triple_%Y%m%d_%H%M%S_%f.csv")
            run_logger = RunCsvLogger(RUN_LOG_DIRECTORY / filename, columns=TRIPLE_COLUMNS)
            run_logger.start()
            run_logger.event("config", json.dumps({
                "schema_version": 1, "angle_protocol_version": 1, "pendulum_count": 3,
                "script": "PythonMaster3_triple.py", "countdown_s": countdown,
                "force_columns": "cart acceleration command, m/s^2",
                "settings": settings_snapshot(), "arms": d["arms"],
                "references_counts": [float(r) for r in refs],
                "discrete_A": Phi.tolist(), "discrete_B": G0.tolist(), "discrete_B_delayed": G1.tolist(),
                "discrete_C": C.tolist(), "controller_gain": K.tolist(), "estimator_gain": L.tolist(),
            }, separators=(",", ":")), 0.0)
            log("LOG", f"Recording every control cycle to {run_logger.path}")

        stage = "upright"
        if countdown > 0:
            log("READY", f"Turn motor power ON. After Enter the motor holds the cart and counts down {countdown:.0f} s. "
                         "Hold all three arms upright with both hands and LET GO on GO.")
            input("Press Enter to start the countdown: ")
        else:
            log("READY", "Turn motor power ON and hold ALL THREE arms upright")
            input("Press Enter when all three arms are upright: ")
        run_started_at = time.monotonic()
        if run_logger is not None:
            run_logger.event("start", "triple balancing requested", 0.0)
        angle_reader.discard_pending(esp32)

        stage = "initial Arduino command"
        if countdown <= 0:
            th0 = check_upright(esp32, refs, offs)
        motor_command_sent = True
        send_motor_command(arduino, -65535)
        last_command = -65535
        steps = read_steps(arduino)
        steps_zero = steps
        t_reply = time.monotonic()
        log("START", f"Arduino replied: step count {steps} is the cart's zero")

        x_wait = None
        if countdown > 0:
            # The motor holds the cart while the clock runs down; a Kalman filter (cart still) follows the
            # arms, so at GO the controller already knows their angles and speeds.
            stage = "countdown"
            hold_x, said = 0.0, None
            t_go = time.monotonic() + countdown
            while True:
                try:
                    raw = read_required(esp32)
                except MissingAnglePacketError:
                    raw = None
                cart_m = (steps - steps_zero) * M_PER_STEP
                timer, _ = timer_for_speed(0.0, cart_m - hold_x)
                send_motor_command(arduino, timer)
                last_command = timer
                steps = read_steps(arduino)
                t_reply = time.monotonic()
                if raw is not None:
                    rel0 = relative_angles(raw, refs, offs)
                    th = absolute_angles(rel0)
                    y = np.array([[v] for v in th] + [[cart_m]])
                    if x_wait is None or max(abs(th[i] - x_wait[2 * i, 0]) for i in range(n)) > 0.2:
                        x_wait = np.zeros((nz, 1))
                        for i in range(n):
                            x_wait[2 * i, 0] = th[i]
                        x_wait[2 * n, 0] = cart_m
                    x_wait = x_wait + L @ (y - C @ x_wait)
                    x_wait[2 * n + 1, 0] = 0.0
                remaining = t_go - time.monotonic()
                if remaining <= 0:
                    break
                sec = math.ceil(remaining)
                if sec != said:
                    angles = "" if x_wait is None else " ".join(
                        f"arm{i + 1}={math.degrees(x_wait[2 * i, 0]):+5.1f}" for i in range(n))
                    log("COUNT", f"{sec} ...   {angles}")
                    said = sec
                if x_wait is not None:
                    x_wait = Phi @ x_wait      # cart held still: no acceleration
                    x_wait[2 * n + 1, 0] = 0.0
            if raw is None or x_wait is None:
                raise RuntimeError("no usable angles at GO; nothing started")
            th0 = th
            worst = max(abs(v) for v in list(th) + list(rel0))
            if worst > math.radians(MAX_START_ANGLE_DEG):
                raise RuntimeError(f"arms not upright at GO ({math.degrees(worst):.1f} deg > {MAX_START_ANGLE_DEG}); "
                                   "cart stopped, nothing started")
            log("START", "GO  -  let go now")
            if run_logger is not None:
                run_logger.event("go", f"angles_deg={[round(math.degrees(v), 2) for v in th0]}",
                                 time.monotonic() - run_started_at)
        cart_m = (steps - steps_zero) * M_PER_STEP

        if x_wait is not None:
            x_pred = Phi @ x_wait               # filter state from the countdown, one cycle on
            x_pred[2 * n + 1, 0] = 0.0
        else:
            x_pred = np.zeros((nz, 1))
            for i in range(n):
                x_pred[2 * i, 0] = th0[i]
            x_pred[2 * n, 0] = cart_m
        a_prev = 0.0
        v_state = 0.0
        v_sent = 0.0
        hold_x = cart_m
        balance_t0 = time.monotonic()
        consecutive_misses = 0
        next_status = time.monotonic()
        last_status_at, last_status_cycle = next_status, 0
        previous_cycle_started = None
        recent_x = deque(maxlen=int(2.0 / T))
        max_cycle_ms = 0.0
        stale_count = 0
        miss_run = [0] * n
        log("RUN", "Triple pendulum control loop started")

        while True:
            stage = "control loop"
            cycle += 1
            cycle_started = time.monotonic()
            cycle_period_ms = "" if previous_cycle_started is None else (cycle_started - previous_cycle_started) * 1000
            previous_cycle_started = cycle_started
            t_run = cycle_started - balance_t0

            stage = "reading receiver angles"
            try:
                raw = read_frame(esp32)
            except MissingAnglePacketError:
                raw = (None, None, None)
            sensor_read_ms = (time.monotonic() - cycle_started) * 1000
            cart_now = cart_m + v_sent * max(0.0, time.monotonic() - t_reply)

            # ---- measurement: relative readings; a missing one is filled from the prediction
            # A reading the receiver already sent last frame (radio age >= STALE_AGE_MS) is old news: it is
            # skipped like a missing one. A fresh one is moved forward by its age beyond the designed age.
            pkt = latest_angle_packet
            ages = [pkt.ages_ms[sl - 1] for sl in RECEIVER_SLOTS] if pkt is not None else [None] * n
            pred_th = [float(x_pred[2 * i, 0]) for i in range(n)]
            pred_w = [float(x_pred[2 * i + 1, 0]) for i in range(n)]
            pred_rel = [pred_th[0]] + [pred_th[i] - pred_th[i - 1] for i in range(1, n)]
            pred_wrel = [pred_w[0]] + [pred_w[i] - pred_w[i - 1] for i in range(1, n)]
            rel, missing, stale = [], [], []
            for i in range(n):
                if raw[i] is None or raw[i] == 0xFFFF:
                    rel.append(pred_rel[i])
                    missing.append(i + 1)
                elif SKIP_STALE and ages[i] is not None and ages[i] >= STALE_AGE_MS:
                    rel.append(pred_rel[i])
                    stale.append(i + 1)
                else:
                    r = -ENCODER_SIGNS[i] * signed_count_delta(raw[i], refs[i]) * RAD_PER_COUNT - offs[i]
                    if AGE_CORRECTION and ages[i] is not None:
                        r += pred_wrel[i] * (ages[i] - NOMINAL_AGE_MS) / 1000.0
                    rel.append(r)
            th = absolute_angles(rel)
            stale_count += len(stale)
            message = ""
            unusable = missing + stale
            if missing:
                log("SENSOR", f"angle(s) {missing} missing; {latest_angle_message} at cycle={cycle}")
            for i in range(n):
                miss_run[i] = miss_run[i] + 1 if (i + 1) in unusable else 0
            consecutive_misses = max(miss_run)
            if unusable:
                message = (f"angle(s) missing {missing} stale {stale}; {latest_angle_message}; using prediction, "
                           f"longest run={consecutive_misses}")
                if consecutive_misses > MAX_RUNNING_CONSECUTIVE_MISSES:
                    raise RuntimeError(f"angle {miss_run.index(consecutive_misses) + 1} unusable for {consecutive_misses} "
                                       f"frames in a row (allowed {MAX_RUNNING_CONSECUTIVE_MISSES}); stopping motor")
            if max(abs(v) for v in th) > math.radians(MAX_RUNNING_ANGLE_DEG):
                raise RuntimeError("Angle limit: " + " ".join(f"arm{i + 1}={math.degrees(v):+.1f}deg" for i, v in enumerate(th)))
            if abs(cart_now) > TRACK_SOFT_LIMIT_M:
                raise RuntimeError(f"Cart at {cart_now:+.3f} m passed TRACK_SOFT_LIMIT_M={TRACK_SOFT_LIMIT_M}")

            # ---- Kalman correction
            y = np.array([[v] for v in th] + [[cart_now]])
            Lnow = L_catch if t_run < CATCH_FILTER_S else L
            x_hat = x_pred + Lnow @ (y - C @ x_pred)
            x_hat[2 * n + 1, 0] = v_state

            # ---- control: cart acceleration
            a = -float((K @ np.vstack([x_hat, [[a_prev]]]))[0, 0])
            if START_RAMP_S > 0 and t_run < START_RAMP_S:
                a *= t_run / START_RAMP_S
            a = max(-CART_ACCEL_MAX_M_S2, min(CART_ACCEL_MAX_M_S2, a))
            v_end = v_state + a * T
            if abs(v_end) > CART_SPEED_MAX_M_S:
                v_end = math.copysign(CART_SPEED_MAX_M_S, v_end)
                a = (v_end - v_state) / T
            v_cmd = v_state + 0.5 * a * T
            if abs(v_cmd) >= SPEED_DEADBAND_M_S:
                hold_x = cart_now + v_cmd * T
            timer_command, step_hz = timer_for_speed(v_cmd, cart_now - hold_x)
            v_sent = sent_speed(timer_command)

            stage = "sending motor command"
            send_motor_command(arduino, timer_command)
            last_command = timer_command
            stage = "reading cart position"
            position_read_started = time.monotonic()
            steps = read_steps(arduino)
            t_reply = time.monotonic()
            position_read_ms = (t_reply - position_read_started) * 1000
            cart_m = (steps - steps_zero) * M_PER_STEP

            # ---- predict the next cycle
            x_pred = Phi @ x_hat + G0 * a + G1 * a_prev
            a_prev = a
            v_state = v_end
            x_pred[2 * n + 1, 0] = v_state

            # ---- auto-trim: each encoder's reference follows its average reading
            trims_before = [math.degrees(v) for v in offs]
            if AUTO_TRIM and t_run > TRIM_START_DELAY_S and not unusable:
                kt = T / TRIM_TIME_CONSTANT_S
                lim = math.radians(TRIM_MAX_DEG)
                cum = 0.0
                for i in range(n):
                    di = max(-lim - offs[i], min(lim - offs[i], kt * rel[i]))
                    offs[i] += di
                    cum += di
                    x_hat[2 * i, 0] -= cum
                    x_pred[2 * i, 0] -= cum

            recent_x.append(cart_m)
            now = time.monotonic()
            loop_ms = (now - cycle_started) * 1000
            max_cycle_ms = max(max_cycle_ms, loop_ms)
            if run_logger is not None:
                pkt = latest_angle_packet
                run_logger.sample((
                    "sample", cycle_started - run_started_at, cycle, esp32_packet_count,
                    *(pkt.angles if pkt is not None else ("", "", "")),
                    *[("" if r is None else r) for r in raw],
                    *[math.degrees(v) for v in th], math.degrees(rel[1]), math.degrees(rel[2]),
                    cart_now, steps - steps_zero, cart_m,
                    *x_hat[:, 0].tolist(), a, v_cmd, step_hz, timer_command,
                    *trims_before, ",".join(str(mm) for mm in missing) + ("" if not stale else " stale " + ",".join(str(mm) for mm in stale)),
                    sensor_read_ms, position_read_ms, loop_ms, cycle_period_ms,
                    *packet_csv_values(pkt, angle_reader), message,
                ))
            if now >= next_status:
                rate = (cycle - last_status_cycle) / max(now - last_status_at, 1e-6)
                xs = np.asarray(recent_x)
                log("RUN", f"cycle={cycle} " + " ".join(f"arm{i + 1}={math.degrees(v):+6.2f}" for i, v in enumerate(th)) +
                           f"deg cart={cart_m * 1000:+7.1f}mm v={v_cmd * 1000:+6.1f}mm/s a={a:+6.1f} "
                           f"spread(2s)={np.ptp(xs) * 1000 if len(xs) else 0:5.1f}mm "
                           f"trim={'/'.join(f'{math.degrees(v):+.2f}' for v in offs)}deg "
                           f"stale={stale_count} rate={rate:.0f}Hz max_cycle={max_cycle_ms:.1f}ms")
                stale_count = 0
                next_status = now + STATUS_INTERVAL_S
                last_status_at, last_status_cycle = now, cycle
                max_cycle_ms = 0.0

    except KeyboardInterrupt:
        run_stop_reason = "Ctrl+C"
        log("STOP", f"Ctrl+C at {stage} after {cycle} cycles")
    except Exception as exc:
        run_stop_reason = f"{type(exc).__name__}: {exc}"
        if run_logger is not None:
            run_logger.event("fault", f"stage={stage}; cycle={cycle}; raw_angles={latest_raw}; cart_steps={steps}; "
                                      f"motor_command={last_command}; angle_details={latest_angle_message}; {run_stop_reason}",
                             "" if run_started_at is None else time.monotonic() - run_started_at)
        log("FAULT", f"stage={stage}, cycle={cycle}: {type(exc).__name__}: {exc}")
        if not isinstance(exc, RuntimeError):
            traceback.print_exc()
    finally:
        try:
            if arduino is not None and arduino.is_open:
                if motor_command_sent:
                    if abs(v_state) > 0.005 and "received" not in run_stop_reason:
                        # brake first: a switched-off stepper would let the cart coast
                        try:
                            v = v_state
                            while abs(v) > 0.005:
                                v -= math.copysign(min(abs(v), STOP_BRAKE_M_S2 * SAMPLE_PERIOD_S), v)
                                timer, _ = timer_for_speed(v, 0.0)
                                send_motor_command(arduino, timer)
                                read_exact(arduino, 2)
                                arduino.reset_input_buffer()
                                time.sleep(SAMPLE_PERIOD_S * 0.8)
                            log("STOP", f"cart braked to a stop from {v_state:+.2f} m/s")
                        except Exception as exc:
                            log("FAULT", f"could not brake the cart: {exc}")
                    try:
                        send_motor_command(arduino, -1)          # timer off, motor disabled
                        arduino.flush()
                        log("STOP", "Sent Arduino stop command")
                    except (serial.SerialException, OSError) as exc:
                        log("FAULT", f"Could not send stop command: {exc}")
                arduino.close()
            if esp32 is not None and esp32.is_open:
                esp32.close()
            log("STOP", "Serial ports closed")
            if cycle:
                tr = ",".join(f"{math.degrees(v):.3f}" for v in offs)
                log("STOP", f"balanced {cycle * SAMPLE_PERIOD_S:.1f} s. Trims found (deg, arm1/joint2/joint3): {tr}. "
                            f"Next run can start there with: --offsets {tr}")
        finally:
            if run_logger is not None:
                warning = run_logger.stop(run_stop_reason)
                log("STOP", warning or f"CSV saved: {run_logger.path}")


# ------------------------------------------------------------------ command line
def apply_arguments(args):
    global CART_ACCEL_MAX_M_S2, CONTROL_LATENCY_MS, START_RAMP_S, START_OFFSETS_DEG, ARM3_PRESET
    if args.slots:
        s = [int(v) for v in args.slots.split(",")]
        if len(s) != 3 or sorted(s) != [1, 2, 3]:
            sys.exit("--slots needs the three slots for arm1,joint2,joint3, e.g. --slots 1,3,2")
        RECEIVER_SLOTS[:] = s
    if args.joint3_sign is not None:
        ENCODER_SIGNS[2] = args.joint3_sign
    if args.joint2_sign is not None:
        ENCODER_SIGNS[1] = args.joint2_sign
    ARM3_PRESET = args.arm3
    m3, c3, I3 = ARM3_PRESETS[ARM3_PRESET]
    if args.arm3_mass_g is not None or args.arm3_com_mm is not None:
        # a measured mass or CoM: keep the radius of gyration of the preset unless a period is given
        k2 = I3 / m3
        m3 = m3 if args.arm3_mass_g is None else args.arm3_mass_g / 1000
        c3 = c3 if args.arm3_com_mm is None else args.arm3_com_mm / 1000
        I3 = m3 * k2
    if args.arm3_period_s is not None:
        # small swings of arm 3 alone about joint 3 (arm 2 held still): T = 2 pi sqrt(I_joint / (m g c))
        I_joint = m3 * G * c3 * (args.arm3_period_s / (2 * math.pi)) ** 2
        I3 = I_joint - m3 * c3 ** 2
        if I3 <= 0:
            sys.exit("--arm3-period-s is too short for that mass and CoM (inertia would be negative)")
    if args.arm3_inertia is not None:
        I3 = args.arm3_inertia
    ARM_MASS_KG[2], ARM_COM_M[2], ARM_INERTIA_KG_M2[2] = m3, c3, I3
    if args.accel_max is not None:
        CART_ACCEL_MAX_M_S2 = args.accel_max
    if args.latency_ms is not None:
        CONTROL_LATENCY_MS = args.latency_ms
    if args.ramp is not None:
        START_RAMP_S = args.ramp
    if args.offsets:
        o = [float(v) for v in args.offsets.split(",")]
        if len(o) != 3:
            sys.exit("--offsets needs three numbers: arm1,joint2,joint3 (degrees)")
        START_OFFSETS_DEG = o


def main():
    ap = argparse.ArgumentParser(description="Triple pendulum balance (V2 arms + arm 3). Default: balance.")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="motor off: show the three slots and angles live")
    mode.add_argument("--log", type=float, metavar="SECONDS", help="log free swings for this long (for fitting arm 3)")
    mode.add_argument("--design-only", action="store_true", help="print the model and gains, open no ports")
    ap.add_argument("--countdown", type=float, default=COUNTDOWN_S, metavar="SECONDS",
                    help="after Enter the motor holds the cart and counts down; let go of the arms on GO "
                         "(both hands free; e.g. --countdown 3)")
    ap.add_argument("--no-motor", action="store_true", help="with --log: do not use the Arduino (hold the cart by hand)")
    ap.add_argument("--slots", help="receiver slots for arm1,joint2,joint3 (default %s)" % ",".join(map(str, RECEIVER_SLOTS)))
    ap.add_argument("--joint3-sign", type=int, choices=(1, -1), help="flip joint 3's direction (-1)")
    ap.add_argument("--joint2-sign", type=int, choices=(1, -1), help="flip joint 2's direction (-1)")
    ap.add_argument("--arm3", choices=sorted(ARM3_PRESETS), default=ARM3_PRESET,
                    help="arm 3 guess: full = with hub electronics at its tip, bare = without (default %(default)s)")
    ap.add_argument("--arm3-mass-g", type=float, help="arm 3 mass, grams (weighed)")
    ap.add_argument("--arm3-com-mm", type=float, help="joint 3 axis to arm 3's centre of mass, mm")
    ap.add_argument("--arm3-period-s", type=float, help="period of small swings of arm 3 alone about joint 3, s")
    ap.add_argument("--arm3-inertia", type=float, help="arm 3 inertia about its own CoM, kg m^2")
    ap.add_argument("--accel-max", type=float, help=f"cart acceleration cap, m/s^2 (default {CART_ACCEL_MAX_M_S2})")
    ap.add_argument("--latency-ms", type=float, help=f"designed delay, ms (default {CONTROL_LATENCY_MS})")
    ap.add_argument("--ramp", type=float, help=f"start ramp, s (default {START_RAMP_S})")
    ap.add_argument("--offsets", help="starting trims arm1,joint2,joint3 in degrees (printed at the end of a run)")
    args = ap.parse_args()
    apply_arguments(args)
    log("START", f"PythonMaster3_triple.py: slots arm1/joint2/joint3 = {RECEIVER_SLOTS}, signs {ENCODER_SIGNS}, "
                 f"arm 3 = {ARM3_PRESET}{' + overrides' if any(v is not None for v in (args.arm3_mass_g, args.arm3_com_mm, args.arm3_period_s, args.arm3_inertia)) else ''}")
    if args.design_only:
        print_design(design())
    elif args.check:
        run_check()
    elif args.log is not None:
        run_freeswing_log(args.log, use_motor=not args.no_motor)
    else:
        run_balance(args.countdown)


if __name__ == "__main__":
    main()
