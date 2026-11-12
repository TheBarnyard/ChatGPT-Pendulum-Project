# Double swing-up V2b_D: cart commands every 5 ms (method D)

`PythonMaster2_swingup_V2b_D.py` is a copy of `PythonMaster2_swingup_V2b.py` (DoubleSwingUp_V2, branch `claude/project-thread-74rdr9`). V2b is not changed. The only difference is how the cart speed commands go to the Arduino. They use method D from the cart-jolt bench tests (`CartJolt/reversal_method_test.py`, 2026-11-11/12), which Barney found the smoothest and most consistent:

- Each 10 ms control cycle sends half of its speed change at once and the other half 5 ms later, so the cart speed changes in steps half as big. `--sends 5` sends every 2 ms (method E), and `--sends 1` sends every 10 ms.
- The script does not wait for the Arduino's reply. Replies (step counts) are read whenever they have arrived, and the newest one gives the cart position. If no reply comes for 150 ms (`REPLY_TIMEOUT_S`), the run stops as before (limit switch or stop button).
- The model, the swing tracking gains and the balance gains are worked out for a speed change that lands in parts. The plan's speed changes are timed to the parts' average.
- The stop brake is sent the same way.
- `--accel-max` sets `CART_ACCEL_MAX_M_S2` from the command line. The default stays at 16.

Everything else is V2b's: plans, observer, balance weights, recovery, limits and CSV columns. `tools/make_v2b_d.py` makes the script from V2b, so the change can be checked line by line.

## Twin check [SIM]

`tools/twin_v2bd_check.py` runs `same_4.0s`. The runs are paired: V2b and V2b_D get the same radio, encoder noise and receiver timing, so only the command sending differs. Results are in `tools/twin_v2bd_check.json`.

| Case | V2b | V2b_D | V2b_D, `--accel-max 12` |
|---|---|---|---|
| Clean radio: nominal, arm-2 inertia +10 %, arm-2 mass -4 % (12 runs) | 12/12 caught first try | 12/12 | 12/12 |
| Rig-like radio, 54 runs: attempts caught | 42/142 (30 %) | 41/137 (30 %) | 27/144 (19 %) |
| Rig-like radio: caught on the first try | 18/54 | 15/54 | 10/54 |

V2b_D catches as often as V2b on the twin. The twin's motor never stalls, so it cannot show whether the 5 ms steps reduce the jolt. Only the rig can show that. A lower acceleration cap (12) costs catches when the radio is noisy.

## Install on the Pi (copy-paste)
This installs into a new folder, so the V2b folder is not touched.
```
mkdir -p ~/Desktop/DoubleSwingUpV2D && cd ~/Desktop/DoubleSwingUpV2D
B=https://raw.githubusercontent.com/TheBarnyard/ChatGPT-Pendulum-Project/claude/project-thread-wk3tn6/DoubleSwingUp_V2D
for f in $(curl -fsSL $B/FILES); do curl -fsSLO $B/$f; done
sha256sum -c SHA256SUMS
```

## Run
```
cd ~/Desktop/DoubleSwingUpV2D
$HOME/PythonLibraries/bin/python PythonMaster2_swingup_V2b_D.py --path same_4.0s
```
Optional settings:
- `--sends 5` sends every 2 ms.
- `--accel-max 12` caps the cart acceleration lower.
- `--list` shows the plans.

The CSV goes to `~/Desktop/DoubleSwingUpV2D/run_logs/`. At the stop, the script prints `cart commands sent: N; M timed sends more than 2 ms late (worst X ms)`. If M is more than a few percent of N, the Pi could not keep the 5 ms pace.
