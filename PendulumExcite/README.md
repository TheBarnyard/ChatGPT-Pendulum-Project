# PendulumExcite

`PendulumExcite.py` runs on the rig's Raspberry Pi. With the arm(s) hanging, it moves the cart in many different ways (steps, kicks, out-and-back pulses, sine dwells, chirps, multi-sines, PRBS, band-limited noise, constant-speed sweeps, resonance pumping, and rests for free swings). The motions come in a shuffled order with random sizes and run until Ctrl+C. Every 10 ms cycle is logged so a simulation (MuJoCo etc.) can be fitted and checked against the real arm.

It stands alone: it needs only numpy and pyserial, and it uses the same ports and protocols as the balance scripts (receiver protocol v1 on `/dev/ttyUSB0` at 921600 baud, ArduinoSlave on `/dev/ttyACM0` at 115200 baud).

## Run

```bash
cd ~/Desktop/PendulumExcite
~/PythonLibraries/bin/python PendulumExcite.py              # arms auto-detected
~/PythonLibraries/bin/python PendulumExcite.py --seed 7     # repeat a run's exact motions
~/PythonLibraries/bin/python PendulumExcite.py --only chirp,step,rest
~/PythonLibraries/bin/python PendulumExcite.py --arms 1     # force single arm (1,2 = double)
```

1. Motor power OFF, cart centred, arm(s) hanging still, Enter: hanging calibration.
2. Motor power ON, arm(s) still hanging, Enter: the motions start (5 s still baseline first).
3. Ctrl+C once: the cart brakes, the motor is disabled, and the files are saved. Ctrl+C twice: the motor is disabled at once.

## Safety

- The motions are planned inside ±0.15 m of the start position (`PLAN_LIMIT_M`), within 90% of 0.6 m/s and 6 m/s².
- The speed command brakes so the cart cannot pass ±0.20 m (`SOFT_LIMIT_M`). Past ±0.25 m by the step count, the motor is disabled and the run ends.
- If any arm swings more than 90° from hanging, or the angles are missing for 0.3 s, the cart stops and holds. It resumes once the swing is under 45°. Angles missing for 5 s end the run.
- The limit switches and the stop button stop the Arduino itself. The script sees that the replies stopped, saves everything, and exits. Rerunning resets the Arduino.

## Output

`run_logs/excite_<date>_<time>/`, next to the script:

| File | Contents |
|---|---|
| `samples.csv` | one row per cycle: timing, raw frame and statuses, angles, planned path, command, limiters, timer value, step reply, cart position, counters (meanings in `columns.txt`) |
| `segments.csv` | one row per motion: type, start and end, why it ended, peak planned position/speed/accel, largest swing, all parameters as JSON |
| `events.csv` | calibration, segment starts, governor on/off, missing frames, step-count glitches, faults, stop |
| `metadata.json` | settings, seed, calibration per channel (reference, noise), sign conventions, summary with the measured free-swing frequency |
| `esp32_raw.bin` | every byte the receiver sent |
| `terminal.log` | everything printed |
| `PendulumExcite_used.py` | the script as it ran |

samples.csv grows at about 2.5 MB per minute.
