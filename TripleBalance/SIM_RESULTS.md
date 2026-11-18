# Simulation check of PythonMaster3_triple.py [SIM]

2026-11-18, Ryan (Claude). The unchanged script ran against a simulated rig (`tools/fake_rig.py`): virtual serial ports and clock, the V2 triple model (Math/shared/nlink_cart.py, arms from RL_Triple_V2 `model/params.json`), receiver protocol v1 frames every 10 ms, Arduino command/reply timing, about 8 ms from sample to motor. A simulated hand holds the arms for the calibration and the start. Nothing here has run on the rig yet.

"Held" means still balancing at the end (21 s, or 60 s for the radio cases). Full list: `tools/batch_results.txt`.

| Case | Result |
|---|---|
| Clean radio, encoder noise 1 or 3 counts | held, cart within 70-85 mm |
| Extra motor delay +2, +4, +5, +6 ms | held |
| Arm 3 mass x0.8 / x1.2, CoM -20 / +20 mm, "bare" arm 3 | held |
| Arm 3 inertia x0.85, x1.2, x1.4 | held |
| Arm 3 inertia x0.8 | **fell** at 3 s |
| Arm 2 mass +15 %, CoM +10 mm | held |
| Start lean, others upright: arm 1 up to 4 deg, arm 2 or arm 3 up to 1.5 deg, all three +1 deg | held |
| Start lean: arm 2 or arm 3 at 2 deg, or 1/-1/1 deg | **fell** within 0.7 s |
| `--countdown 3`, arms let go 0.05 s before GO or on GO | held |
| `--countdown 3`, let go 0.1 s early or 0.05 s late | **fell** |
| Rig-like radio (fitted to the 2026-11-04 excite run: TX1 blackouts 8/s, retries, queue) | held 60 s in 6 of 8; 2 stopped at 43 s by a TX1 gap over 80 ms (safety stop) |
| Encoder zeros off (arm1/joint2/joint3 deg): 0.1/-0.1/0.1, 0/0.15/-0.15, 0/-0.2/0, 0/0/0.25, 1/0/0, 0.2/0.2/0.2, 0.5/-0.1/0.1 | held |
| Encoder zeros off -0.2/0.2/-0.2, 0.3/-0.2/0.2, 0.5/-0.5/0.5 | **fell** (cart past 0.40 m, or arms) |

What was changed because of these runs (all in the script):
- Radio: a reading the receiver already sent last frame (radio age 10 ms or more) is skipped and the prediction fills in; fresh readings are moved forward by (age - 3 ms) x the estimated joint speed. Encoder noise 6 counts in the filter after the first 2 s (3 counts during the catch). With the double's settings the rig-like radio knocked it over after a median 22 s (0 of 8 held 60 s).
- Stops only when one angle is unusable for more than 8 frames in a row (80 ms), not 5 frames of any angle.
- Track soft limit 0.40 m (double 0.30 m): an encoder zero 0.15 deg off drifts the cart 150-250 mm before the auto-trim finds it.
- Hanging calibration needs the arms within 25 counts (0.55 deg), not 80.
- No start ramp; the upright check reads 3 frames (30 ms) instead of 10.
- Tried and not used: stiffer cart weight (0.02-0.03 m: loses arm 3 catches and inertia margin), faster auto-trim (1 s destabilises, 2 s no help), designed delay 10 ms (no help with the radio).

Limits of this check: the hand model is crude (a late release holds the arms while the cart moves), the motor never stalls in the sim (it stalled at 16-18 m/s² reversals on the rig; the script caps at 15), and arm 3's real values are a guess.
