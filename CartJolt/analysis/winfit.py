"""Windowed check of how much of the commanded cart acceleration the arms actually felt.

For each window of N cycles inside SWING (and optionally BALANCE/RECOVER) stretches, fit the arms'
start state (4) plus one extra parameter to the measured angles, simulating the V2 model with the
logged speed commands (staircase kicks tau after each sample, like the script):
  mode 'dA': extra constant cart acceleration dA on top of the commands
  mode 'k' : all commanded speed changes scaled by k (k = 1: cart did what it was told)
Vectorized over all windows; Gauss-Newton with finite-difference Jacobians.
"""
import csv, json, math, sys
import numpy as np

G = 9.81
T = 0.01
M_PER_STEP = 0.638175 / 6400
TIMER_HZ = 125000.0


def load(path):
    rows = list(csv.reader(open(path)))
    H = {h: i for i, h in enumerate(rows[0])}
    cfg = json.loads(rows[1][H['message']]) if rows[1][0] == 'config' else {}
    S = [r for r in rows if r[0] == 'sample']

    def col(k, f=float):
        out = []
        for r in S:
            try:
                out.append(f(r[H[k]]))
            except Exception:
                out.append(np.nan)
        return np.array(out)
    d = {}
    d['t'] = col('elapsed_s')
    d['th1'] = col('measured_angle_1_rad')
    d['th2'] = col('measured_angle_2_rad')
    a1 = col('radio_age_1_ms')
    a2 = np.where(np.isfinite(col('radio_age_3_ms')), col('radio_age_3_ms'), col('radio_age_2_ms'))
    d['age1'], d['age2'] = a1, a2
    d['e1'], d['w1'], d['e2'], d['w2'] = col('est_theta1_rad'), col('est_theta1_rate_rad_s'), col('est_theta2_rad'), col('est_theta2_rate_rad_s')
    tim = col('motor_timer_command')
    v = np.zeros_like(tim)
    ok = np.isfinite(tim) & (np.abs(tim) < 65535) & (tim != 0)
    v[ok] = -np.sign(tim[ok]) * TIMER_HZ / (np.abs(tim[ok]) + 1) * M_PER_STEP
    d['v'] = v
    d['vcmd'] = col('cart_velocity_command_m_s')
    d['mode'] = np.array([r[H['message']].split('mode=')[1].split(';')[0] if 'mode=' in r[H['message']] else '' for r in S])
    d['cart'] = col('cart_m')
    d['ratios'] = cfg.get('model_ratios')
    d['cfg'] = cfg
    return d


def f(z, a, r):
    th1, w1, th2, w2 = z[:, 0], z[:, 1], z[:, 2], z[:, 3]
    dd = th1 - th2
    c, s = np.cos(dd), np.sin(dd)
    rel = w2 - w1
    R1 = G * r['P1'] * np.sin(th1) - r['P1'] * np.cos(th1) * a - r['H'] * s * w2 * w2 - r['B1'] * w1 + r['B2'] * rel
    R2 = G * r['P2'] * np.sin(th2) - r['P2'] * np.cos(th2) * a + r['H'] * s * w1 * w1 - r['B2'] * rel
    M12 = r['H'] * c
    det = r['J2r'] - M12 * M12
    return np.stack([w1, (r['J2r'] * R1 - M12 * R2) / det, w2, (R2 - M12 * R1) / det], axis=1)


def rk4(z, a, dt, r, n=2):
    h = dt / n
    for _ in range(n):
        k1 = f(z, a, r); k2 = f(z + h / 2 * k1, a, r); k3 = f(z + h / 2 * k2, a, r); k4 = f(z + h * k3, a, r)
        z = z + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return z


def kick(z, dv, r):
    th1, th2 = z[:, 0], z[:, 2]
    M12 = r['H'] * np.cos(th1 - th2)
    det = r['J2r'] - M12 * M12
    q1, q2 = -r['P1'] * np.cos(th1) * dv, -r['P2'] * np.cos(th2) * dv
    z = z.copy()
    z[:, 1] += (r['J2r'] * q1 - M12 * q2) / det
    z[:, 3] += (q2 - M12 * q1) / det
    return z


def simulate(p, dvs, r, mode, tau):
    """p (W,5); dvs (W,N) speed change decided at sample j (lands tau later). Returns angles (W,N,2)
    and rates at the sample instants j=0..N-1 (window starts at sample 0)."""
    W, N = dvs.shape
    z = p[:, :4].copy()
    extra = p[:, 4]
    dA = extra if mode == 'dA' else np.zeros(W)
    k = extra if mode == 'k' else np.ones(W)
    out = np.zeros((W, N, 4))
    for j in range(N):
        out[:, j] = z
        z = rk4(z, dA, tau, r, 1)
        z = kick(z, k * dvs[:, j], r)
        z = rk4(z, dA, T - tau, r, 2)
    return out


def wrap(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def fit_windows(d, starts, N, mode, tau=0.008, iters=8, extra_delay=0.0015, fixed=False):
    r = d['ratios']
    W = len(starts)
    idx = starts[:, None] + np.arange(N)[None, :]
    dv_all = np.diff(np.concatenate([[0.0], d['v']]))
    dvs = dv_all[idx]
    m1, m2 = d['th1'][idx], d['th2'][idx]
    # measurement delay: radio age + extra; model angle at t_j - delay ~ angle - rate*delay
    del1 = np.nan_to_num(d['age1'][idx], nan=5.0) / 1000 + extra_delay
    del2 = np.nan_to_num(d['age2'][idx], nan=5.0) / 1000 + extra_delay
    valid = np.isfinite(m1) & np.isfinite(m2)
    p = np.zeros((W, 5))
    p[:, 0], p[:, 1], p[:, 2], p[:, 3] = d['e1'][starts], d['w1'][starts], d['e2'][starts], d['w2'][starts]
    p[:, 4] = 0.0 if mode == 'dA' else 1.0
    # weights like V2b: encoder noise + 3 ms timing noise x rate
    wr1 = np.abs(np.nan_to_num(d['w1'][idx])); wr2 = np.abs(np.nan_to_num(d['w2'][idx]))
    sig1 = np.sqrt((3 * 2 * np.pi / 16384) ** 2 + (0.003 * wr1) ** 2)
    sig2 = np.sqrt((3 * 2 * np.pi / 16384) ** 2 + (0.003 * wr2) ** 2)

    def resid(pp):
        o = simulate(pp, dvs, r, mode, tau)
        a1 = o[:, :, 0] - o[:, :, 1] * del1
        a2 = o[:, :, 2] - o[:, :, 3] * del2
        e = np.concatenate([wrap(m1 - a1) / sig1, wrap(m2 - a2) / sig2], axis=1)
        return np.where(np.concatenate([valid, valid], axis=1), e, 0.0)

    lam = np.full(W, 1e-3)
    e = resid(p)
    cost = (e ** 2).sum(1)
    steps = np.array([1e-4, 1e-3, 1e-4, 1e-3, 0.01 if mode == 'dA' else 0.001])
    for it in range(iters):
        J = np.zeros((W, e.shape[1], 5))
        for i in range(4 if fixed else 5):
            pp = p.copy(); pp[:, i] += steps[i]
            J[:, :, i] = (resid(pp) - e) / steps[i]
        JtJ = np.einsum('wni,wnj->wij', J, J)
        Jte = np.einsum('wni,wn->wi', J, e)
        A = JtJ + lam[:, None, None] * np.eye(5)[None] * (np.einsum('wii->wi', JtJ)[:, :, None] + 1e-9)
        if fixed:
            A[:, 4, :] = 0; A[:, :, 4] = 0; A[:, 4, 4] = 1; Jte[:, 4] = 0
        dp = -np.linalg.solve(A, Jte[:, :, None])[:, :, 0]
        pn = p + dp
        en = resid(pn)
        cn = (en ** 2).sum(1)
        better = cn < cost
        p[better] = pn[better]; e[better] = en[better]; cost[better] = cn[better]
        lam = np.where(better, lam * 0.3, lam * 10)
    # 4-param baseline: same fit with extra fixed
    nval = valid.sum(1) * 2
    return p, cost, nval




def fit_kick(d, starts, ms, N, tau=0.008, iters=10, extra_delay=0.0015, stale_ms=12, mode='kick', timing_ms=3.0):
    """z0 + one uncommanded cart speed jump dvx at sample ms (per window). Readings older than
    stale_ms are down-weighted (held values)."""
    r = d['ratios']
    W = len(starts)
    idx = starts[:, None] + np.arange(N)[None, :]
    dv_all = np.diff(np.concatenate([[0.0], d['v']]))
    dvs0 = dv_all[idx]
    m1, m2 = d['th1'][idx], d['th2'][idx]
    ag1 = np.nan_to_num(d['age1'][idx], nan=99.0); ag2 = np.nan_to_num(d['age2'][idx], nan=99.0)
    del1 = ag1 / 1000 + extra_delay; del2 = ag2 / 1000 + extra_delay
    valid1 = np.isfinite(m1) & (ag1 <= stale_ms); valid2 = np.isfinite(m2) & (ag2 <= stale_ms)
    wr1 = np.abs(np.nan_to_num(d['w1'][idx])); wr2 = np.abs(np.nan_to_num(d['w2'][idx]))
    sig1 = np.sqrt((3 * 2 * np.pi / 16384) ** 2 + (timing_ms / 1000 * wr1) ** 2)
    sig2 = np.sqrt((3 * 2 * np.pi / 16384) ** 2 + (timing_ms / 1000 * wr2) ** 2)
    onehot = np.zeros((W, N))
    if mode == 'kick' and ms is not None:
        onehot[np.arange(W), ms] = 1.0
    fixed = (mode == 'kick' and ms is None) or mode == 'none'
    p = np.zeros((W, 5))
    p[:, 0], p[:, 1], p[:, 2], p[:, 3] = d['e1'][starts], d['w1'][starts], d['e2'][starts], d['w2'][starts]

    if mode == 'k':
        p[:, 4] = 1.0

    def resid(pp):
        if mode == 'kick' or mode == 'none':
            o = simulate(np.c_[pp[:, :4], np.zeros(W)], dvs0 + onehot * pp[:, 4:5], r, 'dA', tau)
        else:
            o = simulate(pp, dvs0, r, mode, tau)
        a1 = o[:, :, 0] - o[:, :, 1] * del1
        a2 = o[:, :, 2] - o[:, :, 3] * del2
        e = np.concatenate([np.where(valid1, wrap(m1 - a1) / sig1, 0), np.where(valid2, wrap(m2 - a2) / sig2, 0)], axis=1)
        return e
    lam = np.full(W, 1e-3)
    e = resid(p); cost = (e ** 2).sum(1)
    steps = np.array([1e-4, 1e-3, 1e-4, 1e-3, 1e-3])
    for it in range(iters):
        J = np.zeros((W, e.shape[1], 5))
        for i in range(4 if fixed else 5):
            pp = p.copy(); pp[:, i] += steps[i]
            J[:, :, i] = (resid(pp) - e) / steps[i]
        JtJ = np.einsum('wni,wnj->wij', J, J); Jte = np.einsum('wni,wn->wi', J, e)
        A = JtJ + lam[:, None, None] * np.eye(5)[None] * (np.einsum('wii->wi', JtJ)[:, :, None] + 1e-9)
        if fixed:
            A[:, 4, :] = 0; A[:, :, 4] = 0; A[:, 4, 4] = 1; Jte[:, 4] = 0
        dp = -np.linalg.solve(A, Jte[:, :, None])[:, :, 0]
        pn = p + dp; en = resid(pn); cn = (en ** 2).sum(1)
        b = cn < cost
        p[b] = pn[b]; e[b] = en[b]; cost[b] = cn[b]
        lam = np.where(b, lam * 0.3, lam * 10)
    n = valid1.sum(1) + valid2.sum(1)
    return p, cost, n
