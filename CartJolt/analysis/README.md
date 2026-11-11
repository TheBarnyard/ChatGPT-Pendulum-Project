# Log analysis: did the cart do what it was told? (2026-11-11)

The Pi logs the step pulses it sent, not where the cart went. These scripts use the arm angles as the sensor:
they fit the V2 arm model (each run's `model_ratios`) to the measured angles over short windows, once driven by the
commanded cart speed changes and once with no cart acceleration. A stalled or locked motor shows up as windows where
"no cart acceleration" fits far better. Needs numpy only. Results: Claude Doc "Cart jolt analysis: 2026-11-04 runs".

| File | What |
| --- | --- |
| `winfit.py` | Loader for V2 run CSVs, the V2 arm model, and vectorised Gauss-Newton window fits (`fit_kick`: modes none / dA / k / kick). |
| `deadscan.py` | 0.5 s windows (hop 0.1 s) over every run: fit error with the commanded motion vs with none. `python3 deadscan.py <run_logs_dir> dead.json` (about 3 min for 30 runs). |
| `stalls.py` | Prints a per-run timeline (a = cart alive, D = stalled, ? = undecided) and stall onsets with the commanded accel just before. `python3 stalls.py dead.json <run_logs_dir>` |
| `allruns.py` | 0.2 s windows, fits a scale k on the commanded speed changes (k = 1: cart followed). `python3 allruns.py <run_logs_dir> windows.json` |

Limits: about 10 ms radio timing jitter makes windows with arms faster than about 5 rad/s undecided; jolts shorter
than about 0.1 s can hide inside a window. Runs whose angle readings are frozen (dead TX) look like a stalled cart:
check `raw_angle_*` first.
