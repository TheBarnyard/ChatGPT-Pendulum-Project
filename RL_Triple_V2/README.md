# ENGT481 triple pendulum RL (V2 arms + 300 mm arm 3)

MuJoCo + Stable-Baselines3 training for the **triple balance** and **triple swing-up**.
Built 2026-11-13 from `engt481_rl_double_newarms` (same SAC settings, reward, action, tooling); only the model and the task list are new.

## v2i (2026-11-15): pure RL with plan starts, with a progress check

Barney's choice after v2h: try plain RL again, with half of the training episodes started at a random point of the planned swing-up (`--variants plan`). Experimental: checked only for start-up, pause and resume, not for learning.
```powershell
python turbo.py --prefix turbo4 --variants plan,plan-quiet --layout cuda,64,8,4096
```
- **Two seeds, two exploration settings.** `plan` keeps SAC's default exploration target (about 0.09 command noise). `plan-quiet` lowers it (`train.py --target-entropy -2.1`, about 0.03), because the imitation policy only survives about 0.03.
- **Progress check ("plan from").** The eval from hanging stays at 0 % until the policy can do the whole swing-up. So each eval now also runs four noise-free episodes started on the plan 3.5, 3, 2 and 1 s into it, and reports the earliest one it finishes (upright for at least half of the episode): the line `plan starts: finishes from 3 s in (...)` in console.log and the `plan from` column in the turbo window. It should move from `-` to 3.5s, 3s, 2s, 1s as training works backwards. For reference: the imitation policy finishes from 1 s; the balance warm start finishes from none (it holds 21 % from 3.5 s, then runs into the rail).

## v2h (2026-11-15): the triple swing-up works: planned controller + imitation

Four RL runs of about 11M steps each (energy, energy-spin, accel18, smooth; 2026-11-14/15, about 8.5 h) did not solve the swing-up. Three settled into rolling the three arms round together about once a second (best score 583, 588, 467 of 1200; 0 % upright); the smooth one sits still from hanging in the noise-free test. v2h takes another route:

1. **Planned controller** (`plan_expert.py`, no learning). `model/triple_swingup_plan.json` is a 4.0 s swing-up found by trajectory optimisation for this model (the team's planning method from PUC Math/Triple; `model/make_plan.py`, which needs `pip install casadi` only to re-make the plan after a model change). A time-varying LQR keeps the arms on the plan, then the balance LQR holds. In this simulator it passes the triple swing-up test in 20 of 20 episodes: score 946, upright at 3.2 s, cart at most 1.2 m/s, 0.40 m of track and 6 to 13 m/s^2.
```powershell
python plan_expert.py                              # 20 test episodes, prints PASS/fail
python watch.py --task triple_swingup --plan       # watch it live
python record.py --task triple_swingup --plan      # mp4: runs\plan_triple_swingup.mp4
```
2. **Imitation** (`imitate.py`, a few minutes, CPU). Teaches the normal 256x256 SAC policy to copy the planned controller (DAgger: the policy shares the driving, the controller labels every visited state). In testing (two runs, seeds 0 and 1, 3 CPU threads): 20 of 20 episodes pass after 4 and 5 rounds (4 to 6 minutes), score 947 and 946, cart at most 8 to 11 m/s^2 in the test episodes. A single round can dip (seed 1: 0 of 10 in round 1); the best round is kept and it stops after three all-pass rounds in a row. With the 18 m/s^2 acceleration cap switched on it still passes (3 of 3 episodes, score 947). Result: `runs\imit_triple_swingup\best.zip`, a normal policy for `watch.py`, `record.py` and `train.py --init-from`.
```powershell
python imitate.py
python watch.py --run imit_triple_swingup --policy best
```

**RL on top of the imitation policy is not ready.** Tested: `train.py --init-from <imitation best.zip> --actor-freeze-steps 30000 --plan-start-prob 0.4`. The score stayed 940 while only the critic learned (to 30k steps) and fell to 27 within 5,000 steps after the policy was allowed to change. `best.zip` keeps the imitation score in that case, but nothing is gained. The policy tolerates little command noise (passes 6 of 6 at noise 0.03, 3 of 6 at 0.05, 0 of 6 at 0.10; SAC's default exploration target is about 0.09) [SIM], which is a likely reason plain SAC never held a catch [HYP].

Also new, experimental and untested beyond a start-up check: `turbo.py --variants plan` (energy reward from the balance warm start, half of the training episodes start at a random point of the plan, no imitation), `train.py --plan-start-prob`, `train.py --actor-freeze-steps`.

Dropped: choosing the plan point from the state instead of the clock in `plan_expert.py` (13 of 20 passes instead of 20 of 20); letting the policy drive alone while collecting imitation data (passes fell from 10/10 to 3/10 as its share went from 70 % to 100 %, because the controller's labels follow the clock).

## v2g (2026-11-15): resume several turbo groups in one window

```powershell
python turbo.py --prefix turbo2,turbo3 --resume   # turbo2_s1, turbo2_s2, turbo3_s1, turbo3_s2 together
```
All listed runs share the PC: CPU threads are split between them (10 usable cores / 4 runs = 2 each) and they share the GPU, so each run is slower than alone; the total st/s is about the same. Ctrl+C pauses all of them; the same command continues. The first run to solve stops the others. Only `--resume` takes a comma list.

## v2f (2026-11-13): exact pause and resume

Ctrl+C (in the turbo window or a train.py window) now saves each run's policy, optimisers and exploration level (`latest.zip`), its replay buffer (`replay_buffer.pkl`, about 110 MB per run) and its counters (`resume.json`: step, best, pass streak, next eval, elapsed time). Continue exactly where it stopped:
```powershell
python turbo.py --prefix turbo3 --resume        # all seeds of that turbo run
python train.py --resume turbo3_s1              # or a single run
```
Runs paused with an older version (no replay buffer) still resume; they refill 20k steps before learning again.

## v2e (2026-11-13): cart acceleration variants (turbo default)

`python turbo.py` now runs one seed of each:
- **accel18:** the cart acceleration is capped at 18 m/s^2 (the commanded speed ramps at most 18 m/s^2 per 1 ms physics step), like the stepper's stall limit. `train.py --cart-accel-limit 18`.
- **smooth:** no cap, but the reward is multiplied by a gentle-acceleration factor: 1 below 5 m/s^2 and below 5 m/s^2 change per 10 ms step, 0.5 at 20 m/s^2 (or 25 m/s^2 per step), falling further above. Because it only scales the positive reward, standing still (reward 0) can never pay more than moving; the factor bites hardest once the arms are up. `train.py --smooth-reward`.
Both use the v2d energy reward. Max cart acceleration (10 ms average) now shows next to max speed in the eval lines, dashboard, report, turbo window and a new HUD line (cart v and a, now and max) in the live view and videos.

## v2d (2026-11-13): energy reward for the triple swing-up (current default)

v2b's `h^2` height reward is flat near hanging, so in testing its evals from hanging still sat still at 100-150k steps. v2d's `triple_swingup` reward is `0.3*h + 0.2*e_term + 0.5*prod(up_i)`: linear height h, plus an energy term that peaks when the arms' energy (cart frame, potential + kinetic) equals exactly the upright energy, so pumping pays at once and spinning does not. In testing it swings hard from the first evals (return 70 to 293 by 125k) but fan-blades (arms up to 50 rad/s), not up yet. Optional anti fan-blading penalty: `--spin-penalty 0.01` (per rad/s above 25 rad/s). `turbo.py` runs one seed of each by default (`--variants energy,energy-spin`). The reward can be chosen with `train.py --reward energy|height|plain`.

## v2b (2026-11-13): triple swing-up fix

The first version's swing-up learned to sit still: the plain reward is about 0 unless all three arms are up, and the warm start from the balance policy left almost no exploration (SAC entropy coef 0.004). v2b changes only `triple_swingup`:
- **Height reward:** `0.5*h^2 + 0.5*prod(up_i)`, with h = the summed heights of the three link ends (0 = hanging, 1 = upright), so every bit of lift earns something. Holding upright still scores highest.
- **Random-state starts:** 30 % of training episodes start in a random pose (all angles, up to 2 rad/s); 10 % upright and 60 % hanging. Evaluations always start hanging.
- **Exploration reset:** with `--init-from`, the swing-up resets the entropy coef to 0.1 (`--ent-coef-init` to change it).

## Turbo (v2c): train the swing-up as fast as the PC allows

```powershell
python turbo.py --dashboard
```
Benchmarks CPU and GPU learner layouts on your PC (about 20 s each), then runs 2 independent swing-up seeds in parallel with the fastest one, warm-started from `runs\tri_1_triple_balance\best.zip`. The first seed to solve stops the other. Trainers run at below-normal priority, 2 cores stay free, and evals, videos and checkpoints happen every 200k steps. Every layout uses the same 512 samples per collected step as the normal run; only the number of updates changes (fewer, bigger ones). Options: `--seeds 3`, `--reserve-cores 4`, `--layout cuda,32,8,2048` (skip the benchmark), `--from <best.zip>`. Runs: `runs\turbo_s1`, `runs\turbo_s2`.

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
