"""Supply test: at what voltage/current does a transmitter start losing angles?

The XIAO ESP32-C3 under test is powered from the adjustable LM317 supply on the
breadboard, through an INA219 (on the Pi's I2C bus) that measures the voltage the
XIAO actually gets and the current it draws. The receiver (protocol v1, 100 Hz on
/dev/ttyUSB0) says, for every frame, whether that transmitter's angle was valid and
how old its last radio packet was.

Turn the pot slowly. Every second one line shows V, I and the losses. Typing a note
and Enter (for example "2.2 ohm series") tags the following rows. Ctrl+C prints a
table of losses by supply voltage.

  --slot 1|2   which receiver channel the test board sends on (TX1 = 1, TX2 = 2)
  --manual     no INA219: type the multimeter voltage and Enter whenever it changes

Standalone: needs only pyserial. Writes run_logs/supply_<time>_windows.csv (one row
per second) and _frames.csv (one row per receiver frame) next to this file.
"""

import argparse
import binascii
import csv
import fcntl
import os
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial

ESP32_PORT, ESP32_BAUD = '/dev/ttyUSB0', 921600
I2C_BUS, INA219_ADDR = '/dev/i2c-1', 0x40
SHUNT_OHMS = 0.1          # R100 on the usual INA219 breakouts
WINDOW_S = 1.0
OUTAGE_MS = 250           # a gap this long usually means the transmitter rebooted (brownout)
BIN_V = 0.1               # summary table bin width

# Receiver protocol v1 (Code/Wireless/ANGLE_PROTOCOL.md)
MAGIC = b'\xa5\x5a\xc3\x3c'
HEADER = bytes((1, 29))
FRAME_SIZE = 37
VALID, ENCODER_FAILED, TIMEOUT, NEVER = 0, 1, 2, 3
STATUS_NAMES = ('valid', 'encoder_read_failed', 'wireless_timeout', 'never_received')
NO_AGE = 0xFFFFFFFF


class FrameReader:
    """Returns every complete, CRC-checked frame (not just the newest)."""

    def __init__(self, slot):
        self.offset = 14 + 7 * (slot - 1)
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
            angle, status, age = struct.unpack_from('>HBI', frame, self.offset)
            if self.last_sequence is not None and sequence > self.last_sequence + 1:
                self.sequence_gaps += sequence - self.last_sequence - 1
            self.last_sequence = sequence
            frames.append((sequence, receiver_ms, angle, status, age))
        return frames


class INA219:
    """Bus voltage (at VIN-, i.e. what the XIAO gets) and current through the shunt."""

    I2C_SLAVE = 0x0703
    # 16 V range, PGA /4 (+-160 mV = +-1.6 A on 0.1 ohm), 12-bit 532 us conversions, shunt+bus continuous
    CONFIG = 0x119F

    def __init__(self, bus=I2C_BUS, address=INA219_ADDR):
        self.fd = os.open(bus, os.O_RDWR)
        fcntl.ioctl(self.fd, self.I2C_SLAVE, address)
        os.write(self.fd, bytes((0x00, self.CONFIG >> 8, self.CONFIG & 0xFF)))
        if self._read(0x00) != self.CONFIG:
            raise OSError(f'INA219 at 0x{address:02x} did not keep its configuration')

    def _read(self, register):
        os.write(self.fd, bytes((register,)))
        high, low = os.read(self.fd, 2)
        return (high << 8) | low

    def read(self):
        shunt = self._read(0x01)
        if shunt & 0x8000:
            shunt -= 0x10000
        volts = (self._read(0x02) >> 3) * 0.004
        return volts, shunt * 10e-6 / SHUNT_OHMS

    def close(self):
        os.close(self.fd)


class Supply:
    """Min/mean/max of V and I since the last take(), kept for two readers (window, frame)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.acc = {'window': self._empty(), 'frame': self._empty()}
        self.samples = 0

    @staticmethod
    def _empty():
        return {'n': 0, 'v_sum': 0.0, 'v_min': None, 'v_max': None,
                'i_sum': 0.0, 'i_min': None, 'i_max': None}

    def add(self, volts, amps):
        with self.lock:
            self.samples += 1
            for acc in self.acc.values():
                acc['n'] += 1
                acc['v_sum'] += volts
                acc['i_sum'] += amps
                acc['v_min'] = volts if acc['v_min'] is None else min(acc['v_min'], volts)
                acc['v_max'] = volts if acc['v_max'] is None else max(acc['v_max'], volts)
                acc['i_min'] = amps if acc['i_min'] is None else min(acc['i_min'], amps)
                acc['i_max'] = amps if acc['i_max'] is None else max(acc['i_max'], amps)

    def take(self, name):
        with self.lock:
            acc, self.acc[name] = self.acc[name], self._empty()
        n = acc['n']
        if not n:
            return None
        return {'v_mean': acc['v_sum'] / n, 'v_min': acc['v_min'], 'v_max': acc['v_max'],
                'i_mean': acc['i_sum'] / n, 'i_min': acc['i_min'], 'i_max': acc['i_max'], 'n': n}


def sample_forever(ina, supply, stop, errors):
    while not stop.is_set():
        try:
            supply.add(*ina.read())
        except OSError as error:
            errors.append(str(error))
            time.sleep(0.05)
        time.sleep(0.0003)


def read_notes(state):
    """Every typed line becomes the note for the following rows; its first number is the --manual voltage."""
    for line in sys.stdin:
        text = line.strip()
        with state['lock']:
            state['note'] = text
            try:
                state['manual_v'] = float(text.split()[0])
            except (IndexError, ValueError):
                pass
        print(f'  note set: {text!r}', flush=True)


class Window:
    def __init__(self):
        self.frames = self.valid = self.encoder_failed = self.timeout = self.never = 0
        self.over10 = self.outages = self.max_age = self.printed = 0

    def add(self, status, age, in_outage):
        self.frames += 1
        self.valid += status == VALID
        self.encoder_failed += status == ENCODER_FAILED
        self.timeout += status == TIMEOUT
        self.never += status == NEVER
        if age != NO_AGE:
            self.max_age = max(self.max_age, age)
            self.over10 += age > 10
        self.outages += age != NO_AGE and age >= OUTAGE_MS and not in_outage


def fmt(value, scale=1.0, digits=3):
    return '' if value is None else f'{value * scale:.{digits}f}'


def print_summary(rows, manual):
    print('\n==== SUMMARY by supply voltage (paste this back) ====')
    if not rows:
        print('no data')
        return
    bins = {}
    for row in rows:
        volts = row['v_mean']
        if volts is None:
            continue
        key = round(round(volts / BIN_V) * BIN_V, 2)
        b = bins.setdefault(key, {'s': 0, 'frames': 0, 'lost': 0, 'enc': 0, 'over10': 0, 'outages': 0,
                                  'max_age': 0, 'v_min': None, 'i_sum': 0.0, 'i_n': 0, 'i_max': None})
        b['s'] += 1
        b['frames'] += row['frames']
        b['lost'] += row['frames'] - row['valid']
        b['enc'] += row['encoder_failed']
        b['over10'] += row['over10']
        b['outages'] += row['outages']
        b['max_age'] = max(b['max_age'], row['max_age'])
        if row['v_min'] is not None:
            b['v_min'] = row['v_min'] if b['v_min'] is None else min(b['v_min'], row['v_min'])
        if row['i_mean'] is not None:
            b['i_sum'] += row['i_mean']
            b['i_n'] += 1
            b['i_max'] = row['i_max'] if b['i_max'] is None else max(b['i_max'], row['i_max'])
    print(f"{'V':>5} {'sec':>4} {'frames':>6} {'lost%':>6} {'enc_fail':>8} {'age>10ms%':>9} "
          f"{'max_age':>7} {'reboots?':>8} {'V_min':>6} {'I_mean':>7} {'I_peak':>7}")
    for key in sorted(bins, reverse=True):
        b = bins[key]
        n = max(1, b['frames'])
        i_mean = '' if manual or not b['i_n'] else f"{1000 * b['i_sum'] / b['i_n']:.0f}mA"
        i_peak = '' if manual or b['i_max'] is None else f"{1000 * b['i_max']:.0f}mA"
        v_min = '' if manual or b['v_min'] is None else f"{b['v_min']:.2f}"
        print(f"{key:5.2f} {b['s']:4d} {b['frames']:6d} {100 * b['lost'] / n:6.2f} {b['enc']:8d} "
              f"{100 * b['over10'] / n:9.2f} {b['max_age']:6d}ms {b['outages']:8d} {v_min:>6} "
              f"{i_mean:>7} {i_peak:>7}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--slot', type=int, choices=(1, 2, 3), default=2,
                        help='receiver channel of the board under test (TX1=1, TX2=2)')
    parser.add_argument('--manual', action='store_true', help='no INA219; type multimeter volts + Enter')
    parser.add_argument('--port', default=ESP32_PORT)
    args = parser.parse_args()

    supply = Supply()
    stop = threading.Event()
    ina_errors = []
    ina = None
    if not args.manual:
        try:
            ina = INA219()
        except OSError as error:
            print(f'INA219 not found on {I2C_BUS} at 0x{INA219_ADDR:02x}: {error}')
            print('Check: I2C enabled (sudo raspi-config nonint do_i2c 0), wiring SDA=pin 3, SCL=pin 5,')
            print('VCC=pin 1 (3.3 V), GND=pin 6. Or run with --manual to use a multimeter instead.')
            sys.exit(1)
        threading.Thread(target=sample_forever, args=(ina, supply, stop, ina_errors), daemon=True).start()

    state = {'lock': threading.Lock(), 'note': '', 'manual_v': None}
    threading.Thread(target=read_notes, args=(state,), daemon=True).start()

    log_dir = Path(__file__).resolve().parent / 'run_logs'
    log_dir.mkdir(exist_ok=True)
    stamp = f'supply_{datetime.now():%Y%m%d_%H%M%S}'
    windows_file = open(log_dir / f'{stamp}_windows.csv', 'w', newline='')
    frames_file = open(log_dir / f'{stamp}_frames.csv', 'w', newline='')
    windows_csv = csv.writer(windows_file)
    frames_csv = csv.writer(frames_file)
    windows_csv.writerow(['t_s', 'v_mean', 'v_min', 'v_max', 'i_mean_mA', 'i_min_mA', 'i_peak_mA',
                          'ina_samples', 'frames', 'valid', 'encoder_read_failed', 'wireless_timeout',
                          'never_received', 'age_over_10ms', 'max_age_ms', 'outages_ge_250ms',
                          'serial_seq_gaps', 'serial_crc_errors', 'note'])
    frames_csv.writerow(['t_s', 'sequence', 'receiver_ms', 'raw_angle', 'status', 'radio_age_ms',
                         'v_min_since_last_frame', 'i_peak_mA_since_last_frame', 'note'])

    print(f'Board under test on receiver channel {args.slot}. '
          + ('Type the multimeter voltage + Enter whenever you change it.' if args.manual
             else 'Turn the pot slowly (about 0.1 V per 20 s near the edge).'))
    print('Type a note + Enter to tag rows. Ctrl+C for the summary.\n')

    esp32 = serial.Serial(args.port, ESP32_BAUD, timeout=0.003)
    reader = FrameReader(args.slot)
    rows = []
    started = time.monotonic()
    next_window = started + WINDOW_S
    window = Window()
    in_outage = False
    previous_valid = True
    gaps_at_window = 0
    try:
        time.sleep(0.2)
        esp32.reset_input_buffer()
        while True:
            frames = reader.read_all(esp32)
            now = time.monotonic()
            if frames:
                with state['lock']:
                    note, manual_v = state['note'], state['manual_v']
                since = supply.take('frame')
                for sequence, receiver_ms, angle, status, age in frames:
                    window.add(status, age, in_outage)
                    in_outage = age != NO_AGE and age >= OUTAGE_MS
                    frames_csv.writerow([
                        f'{now - started:.3f}', sequence, receiver_ms, angle if angle != 0xFFFF else '',
                        STATUS_NAMES[status] if status < 4 else status, age if age != NO_AGE else '',
                        fmt(since and since['v_min']) if not args.manual else manual_v or '',
                        fmt(since and since['i_max'], 1000, 1), note])
                    if status != VALID and previous_valid and window.printed < 3:
                        window.printed += 1
                        print(f'[{now - started:7.1f}s]   LOST {STATUS_NAMES[status] if status < 4 else status}'
                              f' radio_age_ms={age if age != NO_AGE else "?"}', flush=True)
                    previous_valid = status == VALID
            else:
                time.sleep(0.001)
            if now < next_window:
                continue
            next_window += WINDOW_S
            stats = supply.take('window')
            with state['lock']:
                note, manual_v = state['note'], state['manual_v']
            if args.manual:
                stats = {'v_mean': manual_v, 'v_min': None, 'v_max': None, 'i_mean': None,
                         'i_min': None, 'i_max': None, 'n': 0}
            elif stats is None:
                stats = {'v_mean': None, 'v_min': None, 'v_max': None, 'i_mean': None,
                         'i_min': None, 'i_max': None, 'n': 0}
            gaps = reader.sequence_gaps - gaps_at_window
            gaps_at_window = reader.sequence_gaps
            row = dict(stats, frames=window.frames, valid=window.valid, encoder_failed=window.encoder_failed,
                       over10=window.over10, outages=window.outages, max_age=window.max_age)
            rows.append(row)
            windows_csv.writerow([
                f'{now - started:.1f}', fmt(stats['v_mean']), fmt(stats['v_min']), fmt(stats['v_max']),
                fmt(stats['i_mean'], 1000, 1), fmt(stats['i_min'], 1000, 1), fmt(stats['i_max'], 1000, 1),
                stats['n'], window.frames, window.valid, window.encoder_failed, window.timeout, window.never,
                window.over10, window.max_age, window.outages, gaps, reader.crc_errors, note])
            windows_file.flush()
            frames_file.flush()
            volts = '  ?   ' if stats['v_mean'] is None else f"{stats['v_mean']:.3f}"
            supply_text = f'V={volts}'
            if not args.manual and stats['n']:
                supply_text += (f" (min {stats['v_min']:.3f})  I={1000 * stats['i_mean']:4.0f}mA"
                                f" (peak {1000 * stats['i_max']:4.0f})")
                if stats['i_mean'] < -0.005:
                    supply_text += '  NEGATIVE: swap VIN+ / VIN-'
            lost = window.frames - window.valid
            print(f'[{now - started:7.1f}s] {supply_text}  frames={window.frames:3d} lost={lost:3d}'
                  f' enc_fail={window.encoder_failed} age>10ms={window.over10:3d} max_age={window.max_age:4d}ms'
                  + (f' REBOOT?x{window.outages}' if window.outages else '')
                  + (' never_received' if window.never else ''), flush=True)
            if ina_errors:
                print(f'  INA219 read errors: {len(ina_errors)} (last: {ina_errors[-1]})', flush=True)
                ina_errors.clear()
            window = Window()
    except KeyboardInterrupt:
        print('\nStopped by Ctrl+C.')
    finally:
        stop.set()
        esp32.close()
        windows_file.close()
        frames_file.close()
        if ina is not None:
            time.sleep(0.01)
            ina.close()
    print_summary(rows, args.manual)
    print(f'serial crc_errors={reader.crc_errors} discarded_bytes={reader.discarded} seq_gaps={reader.sequence_gaps}')
    print(f'CSV: {log_dir / stamp}_windows.csv and _frames.csv')


if __name__ == '__main__':
    main()
