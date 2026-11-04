# Double swing-up for the V2 arms

`PythonMaster2_swingup_V2.py` is a copy of the team's `PythonMaster2_swingup.py`, taken from the Pi's 2026-10-28 version at `etwan007/ENGT480` `experiment/legacy-pi-recovery` ba019f9. The original is not changed. The copy adds:
- `SYSID_RATIOS` set to the V2-arm fit from 2026-11-04 (`P1 5.2253, P2 2.0148, J2r 0.4506, H 0.4027, B1 0.0182, B2 0.0138`).
- Plan files `double_swingup_V2_<name>.json`. Choose one with `--path <name>` or with `SWING_PATH` at the top of the script. `--list` shows all of them.
- `ARM2_CHANNEL = "auto"`, which reads the joint encoder (TX2) from receiver slot 2 or slot 3. TX2 was on slot 3 on 2026-11-04.
- `PLAN_TRACK_M 0.30`. The swing still aborts at 0.36 m, and the soft limit stays at 0.40 m.

Everything else is unchanged, including tracking gains, balance, recovery, limits and CSV logging.

## Plans (same way up)

The digital-twin column runs this exact script against the emulated Arduino, receiver, stepper and arms. Each plan has 11 cases × 3 seeds = 33 runs. The cases are nominal, arm-2 mass ±4 %, arm-2 inertia ±10 %, arm-1 inertia +10 %, arm-2 CoM +3 %, joint or pivot friction ×3, TX2 on slot 3, and 2 % radio loss with 3 ms latency.

| `--path` | Time | Cart | Speed | Accel | Twin: caught first try |
|---|---|---|---|---|---|
| `same_4.0s` (default) | 4.0 s | ±300 mm | 0.68 m/s | 4.7 m/s² | 33/33 |
| `same_3.5s` | 3.5 s | ±300 mm | 0.71 m/s | 5.1 m/s² | 33/33 |
| `same_3.0s` | 3.0 s | ±290 mm | 0.75 m/s | 6.4 m/s² | 30/33 (fails at arm-2 inertia +10 %) |

All results are simulation only [SIM]. Start with `same_4.0s`.

## Before running
- **The cart travels up to ±300 mm**, and an attempt aborts at ±360 mm. Make sure the limit switches are farther from the centre than that on both sides.
- Centre the cart before starting. Both arms must hang still.

## Install on the Pi (copy-paste)
```
mkdir -p ~/Desktop/DoubleSwingUpV2 && cd ~/Desktop/DoubleSwingUpV2
B=https://raw.githubusercontent.com/TheBarnyard/ChatGPT-Pendulum-Project/claude/project-thread-74rdr9/DoubleSwingUp_V2
for f in $(curl -fsSL $B/FILES); do curl -fsSLO $B/$f; done
sha256sum -c SHA256SUMS
```
## Run
```
cd ~/Desktop/DoubleSwingUpV2
$HOME/PythonLibraries/bin/python PythonMaster2_swingup_V2.py --list
$HOME/PythonLibraries/bin/python PythonMaster2_swingup_V2.py --path same_4.0s
```
The prompts are the same as the original script's. CSV logs go to `~/Desktop/DoubleSwingUpV2/run_logs/`.
