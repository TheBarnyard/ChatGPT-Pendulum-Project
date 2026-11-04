# V2 arms model fit (run excite_20261104_093436)

Run: PendulumExcite, 2026-11-04, 2139 s, both V2 (new PLA) arms, arm 1 on slot 1, arm 2 on slot 3 (TX2 on SENDER_ID 2).
Log: `excite_20261104_093436.tar.xz` (samples.csv etc., esp32_raw.bin left out).

## Result: use these ratios (controller form, J1-normalised, drop-in for SYSID_RATIOS)

```python
SYSID_RATIOS = {"P1": 5.2253, "P2": 2.0148, "J2r": 0.4506, "H": 0.4027, "B1": 0.0182, "B2": 0.0138}
```

Fuller model (adds Coulomb on joint 1 and quadratic air drag; only slightly better):
P1 5.2256, P2 2.0027, J2r 0.4477, H 0.4016, B1 0.0016, B2 0.0028, C1 0.0657 (Coulomb, joint 1), C2 0, D1 0.0016, D2 0.0034 (D*w*|w|).

Equations are the swing-up script's `f(z, a, r)`, the cart speed is the logged `v_sent_m_s` applied as instant jumps (`kick`). Best command latency 0 ms (scan -45..+30 ms). Angle sign: theta = pi + arm_abs (same as the controllers).

Sanity check: H/P2 = L1 = 0.4027/2.0148 = 0.200 m, the real arm 1 joint distance (200 mm), although L1 was not given to the fit.

## How good (held-out windows, simulated from a measured start state with the logged cart command)

Median RMS error arm 1 / arm 2, deg:

| horizon | new fit | sysid 2026-10-27 | swingfit 2026-10-27 | STL CAD 2026-10-29 |
|---|---|---|---|---|
| 0.5 s | 1.2 / 2.0 | 1.7 / 3.2 | 2.1 / 3.3 | 3.1 / 3.8 |
| 1 s | 1.3 / 2.2 | 2.6 / 3.8 | 3.4 / 4.3 | 4.7 / 6.0 |
| 2 s | 1.5 / 2.4 | 3.8 / 5.9 | 4.8 / 6.8 | 7.4 / 9.8 |
| 3 s | 1.7 / 2.7 | 5.0 / 7.9 | 5.8 / 8.4 | 9.1 / 12.5 |

By swing size (2 s): arm 2 < 45 deg: 0.8 / 1.2; 45-120 deg: 1.9 / 3.0; > 120 deg: 4.1 / 9.3 (only 8 windows). Windows where arm 2 spins over the top are chaotic and not predictable over 2 s by any model.

Limits: excitation stayed within +/-0.15 m, 0.54 m/s, 5.4 m/s^2; the swing-up uses more (0.75-0.83 m/s, up to 16 m/s^2). Few windows above 120 deg.

## Files
- fitlib.py: model, simulator, windowing, simulation-error fit (scipy least_squares, per-window start-rate correction).
- final.py: fit + validation; final_results.json: all parameter sets and validation numbers.
- breakdown.py: error by swing size and fit_examples.png.
