# Triple balance with the 2026-11-20 arm values

`PythonMaster3_triple_NewArms.py` is a copy of `PythonMaster3_triple.py` (TripleBalance, 2026-11-18). The original is not changed. Only the arm values differ (plus `--arms old` to go back to the 2026-11-18 values). Helpers are unchanged copies.

## Arm values (Barney's estimates, 2026-11-20) [HYP]

Each arm = PLA bar 34.5 g per 200 mm + parts placed from its END shaft: 2x608 bearings 23.6 g on the end axis, 8x100 shaft 39.2 g + 2 collars 11.0 g on the end axis (arms 1 and 2), rest of the hub (XIAO, LiPo, MT6701, switch) 57.1 g centred 29.1 mm inboard (radius of gyration 12 mm assumed). Worked out in `../DoubleSwingUp_NewArms/tools/arm_values.py`.

| Arm | Length | Mass | CoM from own pivot | Inertia about CoM | Was (11-18) |
|---|---|---|---|---|---|
| 1 | 200 mm | 165.4 g | 169.1 mm | 3.586e-4 kg m² | 98.1 g, 147.8 mm |
| 2 | 250 mm | 174.0 g | 209.5 mm | 6.692e-4 kg m² | 161.8 g, 111.5 mm |
| 3 (`full`) | 300 mm | 132.5 g | 228.8 mm | 9.385e-4 kg m² | 169.9 g, 135.4 mm |
| 3 (`bare`: bar + bearings only) | 300 mm | 75.4 g | 197.0 mm | 7.528e-4 kg m² | 138.4 g, 116.2 mm |

Convention: each joint's shaft and collars are counted in the arm below (at its end), as Barney framed it. The 11-18 script counted them in the arm above (shaft clamped there). A mass sitting on a joint axis moves the arms the same either way, so only the bookkeeping differs. Friction unchanged.

Arm 1 check: these values give a hanging period of 0.856 s; the rig measured 0.84-0.89 s [HW].

## Install on the Pi (copy-paste)
```
mkdir -p ~/Desktop/TripleBalanceNewArms && cd ~/Desktop/TripleBalanceNewArms
B=https://raw.githubusercontent.com/TheBarnyard/ChatGPT-Pendulum-Project/claude/project-thread-evv7sw/TripleBalance_NewArms
for f in $(curl -fsSL $B/FILES); do curl -fsSLO $B/$f; done
sha256sum -c SHA256SUMS
```

## Run (same modes and prompts as before)
```
cd ~/Desktop/TripleBalanceNewArms
$HOME/PythonLibraries/bin/python PythonMaster3_triple_NewArms.py --check
$HOME/PythonLibraries/bin/python PythonMaster3_triple_NewArms.py --countdown 3
```
- `--arms old` runs the 11-18 values (same as PythonMaster3_triple.py).
- `--log 120`, `--slots`, `--joint3-sign`, `--offsets`, `--arm3 bare`, `--arm3-mass-g/--arm3-com-mm` work as before.

## Simulation check [SIM]

The real script on the simulated rig (`tools/fake_rig.py`, from TripleBalance, true arms set per case). 25 s runs unless noted. Results: `tools/batch_results.json`, `tools/batch_old_results.json`.

| True arms | New script | Old script |
|---|---|---|
| New values, clean radio (2 seeds), `--countdown 3` | held | held |
| New values, rig-like radio, 40 s (4 seeds) | held 4/4 | |
| 11-18 values | held | held |
| 11-18 arms 1-2 + new arm 3 | **fell** 5.6 s | |
| Arm 3 mass ±15 %, CoM ±20 mm, inertia x0.8 / x1.4 | held | |
| Arm 1 or 2 inertia x0.8 / x1.4, CoM -20 mm; arm 2 CoM +20 mm | held | |
| Arm 1 mass x0.85 / x1.15, arm 1 CoM +20 mm | **fell** | held (old arms) |
| Arm 2 mass x0.85 / x1.15 | **fell** | x0.85 fell, x1.15 held (old arms) |

With these heavier-ended arms the triple is less forgiving of errors in arms 1 and 2 (mass about ±10 %, arm 1 CoM about +15 mm) than with the 11-18 values. Arm 3 errors matter little. Weighing arms 1 and 2 and finding their balance points would settle it.
