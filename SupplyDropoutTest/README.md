# SupplyDropoutTest

Finds the supply voltage (and current) at which a XIAO ESP32-C3 transmitter starts losing angles.

The board under test (TX2 with its LiPo and USB unplugged) is powered from an LM317 adjustable supply on a breadboard, set with a potentiometer. An INA219 between the LM317 and the XIAO measures the voltage the XIAO gets and the current it draws, and reports to the Pi over I2C. The receiver (protocol v1, `/dev/ttyUSB0`) reports whether each angle arrived. The script logs both once a second and prints a table of losses by voltage.

## Circuit

```
9 V DC + ──┬── LM317 IN        LM317 OUT ──┬──────┬── INA219 VIN+   INA219 VIN- ── XIAO 5V   (Mode B: XIAO 3V3)
           C1 0.1 uF                       R1      C2 10 uF
           │             LM317 ADJ ────────┘       │                INA219 VCC ── Pi pin 1 (3V3)
           │                   │                   │                INA219 SDA ── Pi pin 3
           │                   R2                  │                INA219 SCL ── Pi pin 5
           │                   │                   │                INA219 GND ── Pi pin 6
           │             pot ∥ Rp                  │
9 V DC − ──┴───────────────────┴───────────────────┴──── GND (common with XIAO GND and Pi GND)
```

Pot wired as a variable resistor with the wiper tied to one end. Rp across it caps the top voltage even if the pot is turned fully or its wiper lifts.

| Part | Mode A: XIAO 5V pin | Mode B: XIAO 3V3 pin |
| --- | --- | --- |
| R1 (OUT to ADJ) | 240 Ω | 240 Ω |
| R2 (ADJ to pot) | 390 Ω | 240 Ω |
| Rp across a 10 kΩ pot | 390 Ω | 180 Ω |
| Rp across a 1 kΩ pot | 560 Ω | 220 Ω |
| Output range | about 3.3 to 5.2 V | about 2.5 to 3.4 V |

LM317T legs (text facing you, legs down): 1 ADJ, 2 OUT (tab is OUT too), 3 IN. Input: a 9 V adapter rated 0.5 A or more.

Always check the output range with a multimeter (pot fully both ways) before connecting the XIAO. Mode B must never exceed 3.6 V.

## Pi

```bash
sudo raspi-config nonint do_i2c 0
ls /dev/i2c-1
mkdir -p ~/Desktop/SupplyDropoutTest && cd ~/Desktop/SupplyDropoutTest
curl -fsSLO https://raw.githubusercontent.com/TheBarnyard/ChatGPT-Pendulum-Project/claude/project-thread-m06lor/SupplyDropoutTest/SupplyDropoutTest.py
~/PythonLibraries/bin/python SupplyDropoutTest.py --slot 2
```

- `--slot 2` for TX2 (sender 1), `--slot 1` for TX1. Only one board may send on that slot.
- `--manual` runs without the INA219: type the multimeter voltage and Enter whenever it changes.
- Typing any other text and Enter tags the next rows (for example `4.7 ohm series`).
- Ctrl+C prints the summary by 0.1 V bin. Logs: `run_logs/supply_<time>_windows.csv` (1 s rows) and `_frames.csv` (every receiver frame, with the minimum voltage and peak current since the previous frame).

## Columns

| Column | Meaning |
| --- | --- |
| lost | frames whose angle was not `valid` |
| enc_fail | the transmitter sent `0xFFFF` (its I2C read of the MT6701 failed) |
| age>10ms | at least one radio packet in a row missed (normal age is 0 to 5 ms) |
| REBOOT? / reboots? | a gap of 250 ms or more, which usually means the transmitter browned out and restarted |

The INA219 averages each reading over about 0.5 ms, so short dips and current spikes are smoothed. For the true minimum, put an oscilloscope on the XIAO 3V3 pin.
