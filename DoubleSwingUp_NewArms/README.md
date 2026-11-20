# Double swing-up V2b_D with the 2026-11-20 arm values

`PythonMaster2_swingup_V2b_D_NewArms.py` is a copy of `PythonMaster2_swingup_V2b_D.py` (DoubleSwingUp_V2D). V2b_D is not changed. `tools/make_newarms.py` makes the copy from V2b_D, so every change can be checked. Changes only:

- Arm values = Barney's 2026-11-20 estimates [HYP]: arm 1 200 mm, 165.4 g, CoM 169.1 mm, inertia about CoM 3.586e-4 kg m²; arm 2 250 mm, 174.0 g, CoM 209.5 mm, 6.692e-4 kg m² (joint-3 shaft + 2 collars on its tip). Friction = the 11-04 fit's, same as the triple script. Worked out in `tools/arm_values.py`.
- `--model new` (default) builds the model from those values. `--model fit` runs exactly like V2b_D (the 2026-11-04 rig fit and its V2 plans).
- New plans `double_swingup_NA_<name>.json`: `same_3.0s`, `same_3.5s` (default), `whip_3.0s`, `whip_3.5s`. No `same_4.0s`, `whip_4.5s` or `folded_4.5s` plan was found for the new values within ±0.30 m, 0.8 m/s, 8 m/s². `PLAN_SPEED_M_S` is 0.8 (was 0.75) because the heavier arm 2 has no "same" plan at 0.75.

Note: in V2b_D the old arm numbers (207 g, 87 mm, ...) were never used. `SYSID_RATIOS` (the rig fit) replaced them.

## Which model fits the rig?

| Check | Fit (V2b_D) | New values |
|---|---|---|
| Arm 1 hanging period (rig 0.84-0.89 s) | | 0.856 s |
| P1 / P2 / J2r / H | 5.225 / 2.015 / 0.451 / 0.403 | 5.210 / 3.026 / 0.689 / 0.605 |
| Rig run excite_20261104_093436, median RMS error over 3 s, arm 1 / arm 2 (`tools/rigdata_check.py`) | 1.7 / 2.7 deg | 13.7 / 19.0 deg |
| Same, arm 2 without the joint-3 shaft and collars (123.8 g, CoM 193 mm) | | 9.7 / 13.4 deg |
| Both arms locked straight, small swings about the main pivot (prediction) | 1.12 s | 1.19 s |

Arm 1 agrees. Arm 2's first moment is about 50 % higher than what the arms showed on 2026-11-04. If arm 2 has gained the joint-3 shaft and collars since then (triple build), the new values may now be closer. Otherwise the fit is better.

Quick check, no motor: tape joint 2 so the arms hang straight, time 10 small swings. About 11.2 s means the fit is right (`--model fit`), about 11.9 s means the new values are (`--model new`).

## In-between models (added 2026-11-20 20:15 UTC)

`--model b10` ... `b50`: the 11-04 fit's ratios moved 10 ... 50 % of the way toward the new values (P2 2.015 -> 2.520 at b50). Plans `double_swingup_B<n>_<name>.json`, made by stepping from the V2 plan 10 % at a time: `same_3.5s` (default) and `whip_3.5s` for every step, `same_4.0s` for b10 and b20 only (b30 found none). Twin, clean radio, true arms = the same step: b10, b30, b50 each 2/2 caught first try [SIM].

## Twin check [SIM]

The real script on the digital twin (`tools/twin_newarms_check.py`), 40 s per run. Results: `tools/res_*.json`.

| Script model, plan | True arms | Result |
|---|---|---|
| new, same_3.5s | new values, clean radio (4 runs) | 4/4 caught first try |
| new, same_3.5s | new, arm 2 inertia +15 % or mass -10 % (2+2) | 4/4 first try |
| new, same_3.5s | new, rig-like radio (12 runs) | 6 of 26 attempts caught (23 %), 3/12 first try (V2b_D on the fit: 30 %) |
| new, same_3.0s / whip_3.0s / whip_3.5s | new, clean (2 each) | all caught first try |
| new, same_3.5s | the 11-04 fit (4 runs) | **0 of 8** caught |
| fit, same_4.0s | new values (4 runs) | **0 of 16** caught |

The plan must match the real arms. Each model fails on the other's arms.

## Install on the Pi (copy-paste)
New folder; the V2D folder is not touched.
```
mkdir -p ~/Desktop/DoubleSwingUpNewArms && cd ~/Desktop/DoubleSwingUpNewArms
B=https://raw.githubusercontent.com/TheBarnyard/ChatGPT-Pendulum-Project/f704065642591446c82ae3831dd410f05cedf170/DoubleSwingUp_NewArms
for f in $(curl -fsSL $B/FILES); do curl -fsSLO $B/$f; done
sha256sum -c SHA256SUMS
```

## Run
```
cd ~/Desktop/DoubleSwingUpNewArms
$HOME/PythonLibraries/bin/python PythonMaster2_swingup_V2b_D_NewArms.py
```
- `--path same_3.0s` (or `whip_3.0s`, `whip_3.5s`) picks another plan; `--list` shows them.
- `--model fit` runs the 11-04 fit with its plans (default `same_4.0s`), same as V2b_D.
- `--model b10` ... `b50` runs an in-between model (see above).
- `--sends`, `--accel-max` as in V2b_D. CSVs go to `run_logs/`.
