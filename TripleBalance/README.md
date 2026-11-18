# Triple balance: PythonMaster3_triple.py

A new script for balancing the triple pendulum (V2 arms 1 and 2, arm 3 on top). It is the double balance script `PythonMaster2_auto_improved.py` (12.8 min on the rig) written for three arms. No team file is changed. The helpers `angle_protocol.py`, `angle_packet_fallback.py` and `run_csv_logger.py` are unchanged copies.

## Install on the Pi (copy-paste)

```
mkdir -p ~/Desktop/TripleBalance && cd ~/Desktop/TripleBalance
B=https://raw.githubusercontent.com/TheBarnyard/ChatGPT-Pendulum-Project/claude/project-thread-8o7nnj/TripleBalance
for f in $(curl -fsSL $B/FILES); do curl -fsSLO $B/$f; done
sha256sum -c SHA256SUMS
```

Every line of the check must say `OK`. The folder is outside the git checkouts, so the sync doesn't need pausing. (A branch URL can be up to 5 minutes stale after a push; the chat message gives a commit-pinned one.)

## At the rig, in this order

All commands run from the folder: `cd ~/Desktop/TripleBalance`. The Python is `$HOME/PythonLibraries/bin/python`.

**1. Check the angles (motor never used, about 1 min)**
```
$HOME/PythonLibraries/bin/python PythonMaster3_triple.py --check
```
Let the arms hang completely still and press Enter. Then move the arms by hand and watch the line:
- Bend only arm 3: only `joint3` should change. If `joint2` changes instead, add `--slots 1,2,3` to every command below.
- Bend arm 2 against arm 1 one way (for example clockwise seen from the front), then arm 3 against arm 2 the same way. `joint2` and `joint3` must get the same sign. If they don't, add `--joint3-sign -1` to every command below.
- If a slot shows `----`, that transmitter isn't heard. Fix that first.

**2. Log free swings for the arm 3 model (motor holds the cart, 2 min)**
```
$HOME/PythonLibraries/bin/python PythonMaster3_triple.py --log 120
```
Let the arms hang, press Enter, turn the motor power on, press Enter. Then lift the arms (arm 3 alone, arms 2+3 together, all three) 20-90 degrees and let go. Let each swing die down for 10-20 s. Don't touch the cart. Send me the CSV from `run_logs/freeswing_triple_*.csv`.

**3. Balance**

With a helper who presses Enter:
```
$HOME/PythonLibraries/bin/python PythonMaster3_triple.py
```
On your own (both hands on the arms, let go on GO):
```
$HOME/PythonLibraries/bin/python PythonMaster3_triple.py --countdown 3
```
Same prompts as the double: arms hanging and motor off, Enter; then motor on, arms up, Enter. Ctrl+C stops. At the end the script prints the trims it found. Start the next run from them with `--offsets a,b,c`.

The arms must be very close to straight up when balancing starts. In simulation it catches arm 3 or arm 2 leaning up to 1.5 degrees on its own, and arm 1 up to 4. That is much tighter than the double. With `--countdown`, let go on GO: 0.05 s early still caught, 0.1 s early or 0.05 s late fell.

Before the hanging calibration, stop the swinging with a light touch and wait until the arms are completely still. The triple only tolerates about 0.15-0.2 degrees of error in the joint references (simulation), so the script now refuses a calibration with more than 25 counts of movement.

If you weighed arm 3: add `--arm3-mass-g <grams> --arm3-com-mm <mm from joint 3 to its balance point>`. If arm 3 carries no electronics at its tip: add `--arm3 bare`.

## Settings you might change (top of the script, or on the command line)

| Problem | Change |
|---|---|
| Oscillation of a few Hz that grows | `--latency-ms 10` or `11` |
| Motor buzzes or the driver LED goes red | `--accel-max 12` (but the triple needs a lot of acceleration) |
| Stops with "unusable for N frames" | a transmitter dropped out for more than 80 ms: check radios and batteries |
| Cart drifts 0.2-0.4 m soon after the start, then the run stops at the track limit | the hanging calibration was off: let the arms settle longer; after a run that held, pass the printed trims with `--offsets` |

## Simulation check [SIM]

The real script was run against a simulated rig (virtual serial ports and clock, the V2 triple model, the receiver frames and the Arduino replies): see SIM_RESULTS.md. Short version: it holds with clean radio, with up to 6 ms extra delay and with arm 3's mass ±20 %, CoM ±20 mm and inertia -15 to +40 % off. With radio timing like the rig's on 2026-11-04 it held 60 s in 6 of 8 runs; the other 2 stopped at 43 s when TX1 went silent for more than 80 ms.

## Files

| File | What |
|---|---|
| `PythonMaster3_triple.py` | the script |
| `angle_protocol.py`, `angle_packet_fallback.py`, `run_csv_logger.py` | team helpers, unchanged |
| `SIM_RESULTS.md` | simulation results |
| `tools/fake_rig.py`, `tools/batch.py` | the simulated rig used for the check |
