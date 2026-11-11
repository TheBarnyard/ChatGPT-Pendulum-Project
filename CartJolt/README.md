# Cart jolt tests (2026-11-05)

Tools to find out why the cart jolts (UIM344 red LED) during double swing-up. The full step-by-step plan is in the project's Claude Doc "Cart jolt debug plan"; this folder holds the scripts it uses. Nothing here changes the team's files.

| File | What |
| --- | --- |
| `motor_stress_test.py` | 18 cart moves of rising acceleration / speed, no pendulum control. After each it asks: red LED? jolt? offset from a tape mark? Saves `results/stress_*.csv`. |
| `replay_run_motor.py` | Replays the exact motor commands of a logged swing-up attempt (`motor_timer_command` column). `--info` lists attempts with peak speed / acceleration. |
| `two_load_stall_test.py` | Arms off. Finds the acceleration where the motor stalls (ending at 0.8 m/s), once with the bare cart and once with a weighed item strapped on, then `--compute` gives the cart's effective mass and the motor's spare force. |
| `two_load_reversal_test.py` | Same as the stall test, but each trial is 3 back-to-back hard reversals between -0.8 and +0.8 m/s with no braking pause, like the swing-up paths. Imports the search and math from `two_load_stall_test.py`. |
| `command_style_test.py` | Arms off. Sends the same move 7 ways (command every 2/5/10/20/40 ms, timer hysteresis, smoothed corners) in a blind shuffled order; you score the stutter, it ranks the techniques. Moves: reversals, sine, steady cruise, or a replayed swing attempt. |
| `cart_link.py` | Shared serial link to the Mega (same protocol as the swing-up scripts). |
| `ArduinoSlaveRamp/ArduinoSlaveRamp.ino` | Copy of `ArduinoSlave.ino` that ramps the step rate over each 10 ms instead of jumping. Same pins, protocol and step counting. Compiles (avr-gcc syntax check); not yet run on the rig. |
| `tools/fake_mega.py` | Fake Mega on a pseudo-terminal, used to test the scripts off the rig. |

The Mega's step count is the number of pulses it sent, not where the cart is. Skipped teeth or a driver lockout never show in it, so the tests use a tape mark and your eyes (or a slow-motion video).

## Install on the Pi (copy-paste)
```
mkdir -p ~/Desktop/CartJolt && cd ~/Desktop/CartJolt
B=https://raw.githubusercontent.com/TheBarnyard/ChatGPT-Pendulum-Project/claude/project-thread-wk3tn6/CartJolt
for f in $(curl -fsSL $B/FILES); do mkdir -p "$(dirname "$f")"; curl -fsSL -o "$f" "$B/$f"; done
sha256sum -c SHA256SUMS
PY=$HOME/PythonLibraries/bin/python
$PY motor_stress_test.py --list
```
