# ENGT481 triple pendulum RL (V2 arms + 300 mm arm 3)

MuJoCo + Stable-Baselines3 training for the **triple balance** and **triple swing-up**.
Built 2026-11-13 from `engt481_rl_double_newarms` (same SAC settings, reward, action, tooling); only the model and the task list are new.

## Install (once, Windows PowerShell)

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```
Linux/WSL: `python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt`, then `sudo apt install -y libegl1 libgl1` for videos.

## Train

```powershell
python run_all.py --dashboard
```
- `triple_balance` first; stops after **3** passing evals in a row (all 3 eval episodes last 10 s with >= 95 % upright). Budget 3M steps.
- `triple_swingup` warm-starts from that `best.zip`; stops after **5** passing evals in a row (12 s episodes: swing up within 6 s, then hold >= 6 s, >= 45 % upright overall). Budget 60M steps; Ctrl+C any time, `latest.zip` and `best.zip` are kept.
- Best replays: `runs\tri_best_videos\`. Report zip: `runs\tri_report_<time>.zip` (`python report.py --prefix tri`).

Other ways to run:
```powershell
python run_all.py --only triple_balance
python run_all.py --only triple_swingup --swingup-from runs\tri_1_triple_balance\best.zip
python train.py --task triple_balance --dashboard
python run_all.py --dashboard -- --randomize        # +-10 % masses, servo stiffness, 0-10 ms latency, encoder noise
```
Double policies cannot be warm-started here (12 inputs instead of 9). The double tasks still exist (`--task double_balance`) but use the old CAD double model.

## After training

```powershell
python watch.py --run tri_2_triple_swingup --policy best
python record.py --run tri_2_triple_swingup --seconds 12 --episodes 3
python watch.py --task triple_balance --lqr          # LQR reference on this triple model
python watch.py --task triple_swingup --passive      # physics only
```

## The triple model (`model/triple_pendulum.xml`, made by `model/build_triple_v2.py`)

| Link | Length | Mass | CoM from own pivot | I about CoM (swing) | Source |
|---|---|---|---|---|---|
| 1 = arm 1 | 200 mm | 98.1 g | 147.8 mm | 3.40e-4 kg m^2 | STL + measured weights (P1 matches the V2 rig fit within 0.2 %) |
| 2 = arm 2 + joint-2 shaft | 250 mm | 161.8 g | 111.5 mm | 2.02e-3 kg m^2 | mass from STL; CoM and inertia from the V2 rig fit (2026-11-04) |
| 3 = arm 3 + joint-3 shaft | 300 mm | 169.9 g | 135.4 mm | 3.06e-3 kg m^2 | arm 2 stretched 50 mm, then the same correction the fit found for arm 2 [HYP] |

Joint damping from the fit: b1 1.63e-4, b2 1.24e-4 N m s; b3 = b2 assumed.

Assumptions to check when arm 3 exists:
- Arm 3 carries a full copy of arm 2's hub hardware (bearing, mounts, XIAO, LiPo, encoder board). If it is built without electronics, run `python model\build_triple_v2.py --arm3-hub bare` and retrain.
- Printed bar mass linear in length (50.6 g at 300 mm). Arm 3 plane 25 mm out from arm 2 (assumed, like arm 2).
- Arm 3 is not weighed: weigh it and time arm 3's hanging period, then rebuild.

## Rig match and limits

- 100 Hz policy, 1 kHz physics, cart velocity command, 1.4 m/s limit, episode ends at |x| >= 0.50 m (track is about +-0.54 m).
- The cart is an ideal velocity servo: the real motor stalls at about 16-18 m/s^2 reversals (2026-11-05 cart jolt debug); not modelled here.
- Bearing Coulomb friction, belt stretch and radio timing noise are not modelled.
