"""A/B test: does the cart motor/driver make the TX1 -> receiver radio drop angles?

Motor power ON, pendulum hanging, cart centred. Runs these phases back to back and
counts angle-1 radio losses in each, using every receiver frame (100 Hz):

  esp_only      Arduino port closed (same as AngleSensorDiagnostic.py)
  arduino_idle  Arduino port open and reset, no command yet (driver disabled)
  hold          driver enabled, cart held still (what the controller does when balanced)
  move          cart wiggles +/-MOVE_STEPS at MOVE_STEP_HZ (stepping like swing-up/LQG)
  hold, move    repeated, because Wi-Fi interference changes over time
  esp_only_end  motor stopped/disabled, Arduino port closed

Ctrl+C stops the motor and prints the summary so far.
Standalone: needs only pyserial. Writes run_logs/radio_test_<time>.csv next to this file.
"""

import binascii
import csv
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

ESP32_PORT, ESP32_BAUD = '/dev/ttyUSB0', 921600
ARDUINO_PORT, ARDUINO_BAUD = '/dev/ttyACM0', 115200
PHASE_S = 30.0
MOVE_STEPS = 200          # +/- steps from the start position (about +/-2 cm)
MOVE_STEP_HZ = 1000       # cart step rate while moving
PHASES = ['esp_only', 'arduino_idle', 'hold', 'move', 'hold', 'move', 'esp_only_end']

# Receiver protocol v1 (Code/Wireless/ANGLE_PROTOCOL.md)
MAGIC = b'\xa5\x5a\xc3\x3c'
HEADER = bytes((1, 29))
FRAME_SIZE = 37
VALID = 0
STATUS_NAMES = ('valid', 'encoder_read_failed', 'wireless_timeout', 'never_received')
TIMER_STEP_HZ = 125000.0  # ArduinoSlave.ino: step rate = 125000 / (|timer| + 1)


class FrameReader:
    """Returns every complete, CRC-checked frame (not just the newest)."""

    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = self.discarded = self.sequence_gaps = 0
        self.last_sequence = None

    def read_all(self, port):
        waiting = port.in_waiting
        if waiting:
            self.buffer.extend(port.read(waiting))
        frames = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                keep = len(MAGIC) - 1
                self.discarded += max(0, len(self.buffer) - keep)
                del self.buffer[:max(0, len(self.buffer) - keep)]
                break
            self.discarded += start
            del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break
            frame = bytes(self.buffer[:FRAME_SIZE])
            if frame[4:6] != HEADER or binascii.crc_hqx(frame[4:-2], 0xFFFF) != int.from_bytes(frame[-2:], 'big'):
                self.crc_errors += frame[4:6] == HEADER
                self.discarded += 1
                del self.buffer[:1]
                continue
            del self.buffer[:FRAME_SIZE]
            sequence, receiver_ms = struct.unpack_from('>II', frame, 6)
            angle, status, age = struct.unpack_from('>HBI', frame, 14)
            if self.last_sequence is not None and sequence > self.last_sequence + 1:
                self.sequence_gaps += sequence - self.last_sequence - 1
            self.last_sequence = sequence
            frames.append((sequence, receiver_ms, angle, status, age))
        return frames


class PhaseStats:
    def __init__(self, name):
        self.name = name
        self.frames = self.missing = self.episodes = self.longest = self.streak = 0
        self.over10 = self.over20 = self.over50 = 0
        self.max_age = 0
        self.gaps_start = None
        self.gaps = 0

    def add(self, status, age):
        self.frames += 1
        age_ms = age if age != 0xFFFFFFFF else 99999
        self.max_age = max(self.max_age, age_ms)
        self.over10 += age_ms > 10
        self.over20 += age_ms > 20
        self.over50 += age_ms > 50
        if status != VALID:
            self.missing += 1
            self.streak += 1
            self.episodes += self.streak == 1
            self.longest = max(self.longest, self.streak)
        else:
            self.streak = 0

    def line(self):
        n = max(1, self.frames)
        return (f'{self.name:<13} frames={self.frames:5d} missing={self.missing:4d} '
                f'episodes={self.episodes:3d} longest={self.longest:3d} '
                f'age>10ms={100 * self.over10 / n:5.2f}% age>20ms={100 * self.over20 / n:5.2f}% '
                f'max_age={self.max_age:5d}ms serial_seq_gaps={self.gaps}')


def read_exact(port, count, timeout_s=0.1):
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        data.extend(port.read(count - len(data)))
    if len(data) != count:
        raise TimeoutError(f'Arduino sent {len(data)}/{count} reply bytes (limit switch or button?)')
    return bytes(data)


def main():
    print('Motor power ON, pendulum hanging, cart centred. Keep hands clear of the cart.')
    input('Press Enter to start (Ctrl+C stops the motor at any time): ')
    log_dir = Path(__file__).resolve().parent / 'run_logs'
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f'radio_test_{datetime.now():%Y%m%d_%H%M%S}.csv'
    log_file = open(log_path, 'w', newline='')
    writer = csv.writer(log_file)
    writer.writerow(['t_s', 'phase', 'sequence', 'receiver_ms', 'raw_angle_1', 'status_1',
                     'radio_age_1_ms', 'cart_steps', 'motor_cmd'])

    esp32 = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.003)
    arduino = None
    motor_started = False
    reader = FrameReader()
    stats = []
    started = time.monotonic()
    try:
        time.sleep(0.2)
        esp32.reset_input_buffer()
        reference = position = None
        direction = 1
        command = 0
        for name in PHASES:
            if name == 'arduino_idle':
                arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.003, write_timeout=0.2)
                print('  (Arduino reset; 3 s startup wait counts toward this phase)')
            if name == 'esp_only_end' and arduino is not None:
                arduino.write(struct.pack('<I', 0xFFFFFFFF))
                arduino.flush()
                arduino.close()
                arduino = None
            phase = PhaseStats(name)
            phase.gaps_start = reader.sequence_gaps
            stats.append(phase)
            print(f'[{time.monotonic() - started:6.1f}s] phase {name} for {PHASE_S:.0f} s', flush=True)
            phase_end = time.monotonic() + PHASE_S
            next_status = time.monotonic() + 5
            while time.monotonic() < phase_end:
                frames = reader.read_all(esp32)
                if not frames:
                    time.sleep(0.001)
                    continue
                if name in ('hold', 'move') and arduino is not None:
                    if position is None:
                        command = -65535
                    elif name == 'hold':
                        command = 65535 if position > reference else -65535
                    else:
                        if position >= reference + MOVE_STEPS:
                            direction = -1
                        elif position <= reference - MOVE_STEPS:
                            direction = 1
                        icr = int(round(TIMER_STEP_HZ / MOVE_STEP_HZ)) - 1
                        command = -icr if direction > 0 else icr  # negative timer counts up
                    arduino.write(struct.pack('<i', command))
                    motor_started = True
                    position = struct.unpack('>h', read_exact(arduino, 2))[0]
                    arduino.reset_input_buffer()
                    if reference is None:
                        reference = position
                now = time.monotonic() - started
                for sequence, receiver_ms, angle, status, age in frames:
                    phase.add(status, age)
                    writer.writerow([f'{now:.3f}', name, sequence, receiver_ms, angle,
                                     STATUS_NAMES[status] if status < 4 else status,
                                     age if age != 0xFFFFFFFF else '', position, command])
                    if status != VALID and phase.streak == 1:
                        print(f'[{now:6.1f}s]   angle1 {STATUS_NAMES[status]} radio_age_ms={age}', flush=True)
                if time.monotonic() >= next_status:
                    print(f'[{now:6.1f}s]   {phase.line()}', flush=True)
                    next_status += 5
            phase.gaps = reader.sequence_gaps - phase.gaps_start
    except KeyboardInterrupt:
        print('\nStopped by Ctrl+C.')
        if stats:
            stats[-1].gaps = reader.sequence_gaps - stats[-1].gaps_start
    finally:
        if arduino is not None:
            if motor_started:
                arduino.write(struct.pack('<I', 0xFFFFFFFF))
                arduino.flush()
            arduino.close()
        esp32.close()
        log_file.close()
    print('\n==== SUMMARY (paste this back) ====')
    for phase in stats:
        print(phase.line())
    print(f'crc_errors={reader.crc_errors} discarded_bytes={reader.discarded}')
    print(f'CSV: {log_path}')


if __name__ == '__main__':
    main()
