"""Fit the double-pendulum-on-cart model (same equations as PythonMaster2_swingup f(z,a,r))
to a PendulumExcite run.

Model (angles from upright, J1-normalised ratios, cart acceleration is the input):
  R1 = g P1 sin th1 - P1 cos th1 a - H sin(d) w2^2 - B1 w1 + B2 (w2-w1) - C1 tanh(w1/eps)
  R2 = g P2 sin th2 - P2 cos th2 a + H sin(d) w1^2 - B2 (w2-w1)        - C2 tanh((w2-w1)/eps)
  [1, H cos d; H cos d, J2r] [w1'; w2'] = [R1; R2 + C2-term on joint ...]
C1/C2 (Coulomb, smoothed) are extra over the controller model; set 0 to get the controller model.
The stepper changes speed in jumps, so a speed change dv is an impulse: kick() as in the controller.
"""
import numpy as np
import pandas as pd

G = 9.81
EPS_W = 0.05  # rad/s, Coulomb smoothing
PNAMES = ["P1", "P2", "J2r", "H", "B1", "B2", "C1", "C2", "D1", "D2"]

USECOLS = ["t_mono_s", "cmd_sent_after_frame_ms", "frame_ok", "motion", "segment", "governor",
           "stopping", "arm1_abs_deg", "arm2_abs_deg", "v_sent_m_s", "cart_m", "status_1", "status_3"]


def load(path):
    df = pd.read_csv(path, usecols=lambda c: c in USECOLS, low_memory=False)
    t = df["t_mono_s"].to_numpy(float)
    t = t - t[0]
    out = {
        "t": t,
        "t_cmd": t + np.nan_to_num(df["cmd_sent_after_frame_ms"].to_numpy(float)) / 1000.0,
        "ok": df["frame_ok"].to_numpy(int) == 1,
        "phi1": np.radians(df["arm1_abs_deg"].to_numpy(float)),
        "phi2": np.radians(df["arm2_abs_deg"].to_numpy(float)),
        "v": np.nan_to_num(df["v_sent_m_s"].to_numpy(float)),
        "x": df["cart_m"].to_numpy(float),
        "motion": df["motion"].astype(str).to_numpy(),
        "segment": df["segment"].to_numpy(int),
    }
    return out


def rhs(th1, w1, th2, w2, a, p):
    P1, P2, J2r, H, B1, B2, C1, C2 = p[:8]
    D1, D2 = (p[8], p[9]) if len(p) > 8 else (0.0, 0.0)
    d = th1 - th2
    c, s = np.cos(d), np.sin(d)
    rel = w2 - w1
    cj = C2 * np.tanh(rel / EPS_W)
    R1 = G * P1 * np.sin(th1) - P1 * np.cos(th1) * a - H * s * w2 * w2 - B1 * w1 + B2 * rel \
        - C1 * np.tanh(w1 / EPS_W) + cj - D1 * w1 * np.abs(w1)
    R2 = G * P2 * np.sin(th2) - P2 * np.cos(th2) * a + H * s * w1 * w1 - B2 * rel - cj - D2 * w2 * np.abs(w2)
    M12 = H * c
    det = J2r - M12 * M12
    return w1, (J2r * R1 - M12 * R2) / det, w2, (R2 - M12 * R1) / det


def kick(th1, w1, th2, w2, dv, p):
    P1, P2, J2r, H = p[:4]
    M12 = H * np.cos(th1 - th2)
    det = J2r - M12 * M12
    q1, q2 = -P1 * np.cos(th1) * dv, -P2 * np.cos(th2) * dv
    return w1 + (J2r * q1 - M12 * q2) / det, w2 + (q2 - M12 * q1) / det


def simulate(z0, dv, dt, p, sub=2):
    """z0: (N,4) start states; dv: (N,K) cart speed jump at the START of each of K intervals.
    Returns (N,K+1,4) states at the grid points. Cart acceleration between jumps is 0."""
    th1, w1, th2, w2 = (z0[:, i].copy() for i in range(4))
    N, K = dv.shape
    out = np.empty((N, K + 1, 4))
    out[:, 0] = z0
    h = dt / sub
    for k in range(K):
        w1, w2 = kick(th1, w1, th2, w2, dv[:, k], p)
        for _ in range(sub):
            k1 = rhs(th1, w1, th2, w2, 0.0, p)
            k2 = rhs(th1 + h / 2 * k1[0], w1 + h / 2 * k1[1], th2 + h / 2 * k1[2], w2 + h / 2 * k1[3], 0.0, p)
            k3 = rhs(th1 + h / 2 * k2[0], w1 + h / 2 * k2[1], th2 + h / 2 * k2[2], w2 + h / 2 * k2[3], 0.0, p)
            k4 = rhs(th1 + h * k3[0], w1 + h * k3[1], th2 + h * k3[2], w2 + h * k3[3], 0.0, p)
            th1 = th1 + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
            w1 = w1 + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
            th2 = th2 + h / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
            w2 = w2 + h / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])
        out[:, k + 1] = np.stack([th1, w1, th2, w2], -1)
    return out


def regrid(d, dt=0.01, latency=0.0):
    """Resample angles onto a uniform grid and put the commanded cart speed on it (shifted by latency)."""
    t = d["t"]
    tg = np.arange(t[0], t[-1], dt)
    ok = d["ok"] & np.isfinite(d["phi1"]) & np.isfinite(d["phi2"])
    phi1 = np.interp(tg, t[ok], d["phi1"][ok])
    phi2 = np.interp(tg, t[ok], d["phi2"][ok])
    # frame validity on the grid: nearest sample must be ok and close in time
    idx = np.clip(np.searchsorted(t, tg), 1, len(t) - 1)
    near = np.where(np.abs(t[idx] - tg) < np.abs(t[idx - 1] - tg), idx, idx - 1)
    gok = ok[near] & (np.abs(t[near] - tg) < 0.015)
    # cart speed: zero-order hold of v_sent from the command time (+ latency)
    tc = d["t_cmd"] + latency
    j = np.searchsorted(tc, tg, side="right") - 1
    vg = np.where(j >= 0, d["v"][np.clip(j, 0, None)], 0.0)
    xg = np.interp(tg, t, d["x"])
    return {"t": tg, "phi1": phi1, "phi2": phi2, "v": vg, "x": xg, "ok": gok,
            "motion": d["motion"][near], "segment": d["segment"][near]}


def savgol_rates(y, dt, win=11, order=3):
    from scipy.signal import savgol_filter
    return savgol_filter(y, win, order, deriv=1, delta=dt)


def lag(v, tau, dt=0.01):
    """First-order lag of the commanded speed (cart does not reach the new speed at once)."""
    if tau <= 0:
        return v
    from scipy.signal import lfilter
    al = dt / (tau + dt)
    return lfilter([al], [1, -(1 - al)], v)


def make_windows(g, length_s, hop_s, sign, cart_sign, dt=0.01, max_rate=40.0, motions=None, tau=0.0, offset=0):
    """Cut windows of `length_s` where every frame is valid. Returns dict of arrays."""
    n = int(round(length_s / dt))
    hop = int(round(hop_s / dt))
    th1 = np.pi + sign * g["phi1"]
    th2 = np.pi + sign * g["phi2"]
    w1 = savgol_rates(th1, dt)
    w2 = savgol_rates(th2, dt)
    v = cart_sign * lag(g["v"], tau, dt)
    bad = ~g["ok"]
    cbad = np.concatenate([[0], np.cumsum(bad)])
    starts = []
    for s0 in range(5 + offset, len(th1) - n - 5, hop):
        if cbad[s0 + n + 1] - cbad[s0] > 0:
            continue
        if motions is not None and g["motion"][s0] not in motions:
            continue
        if abs(w1[s0]) > max_rate or abs(w2[s0]) > max_rate:
            continue
        starts.append(s0)
    starts = np.array(starts, int)
    if len(starts) == 0:
        raise ValueError("no clean windows")
    ii = starts[:, None] + np.arange(n + 1)[None, :]
    vprev = v[starts - 1]
    vw = v[ii[:, :-1]]
    dv = np.diff(np.concatenate([vprev[:, None], vw], 1), axis=1)
    z0 = np.stack([th1[starts], w1[starts], th2[starts], w2[starts]], -1)
    return {"z0": z0, "dv": dv, "th1": th1[ii], "th2": th2[ii], "starts": starts,
            "motion": g["motion"][starts], "t0": g["t"][starts]}


def residuals(W, p, fit_z0=None):
    z0 = W["z0"] if fit_z0 is None else fit_z0
    sim = simulate(z0, W["dv"], 0.01, p)
    r1 = sim[:, :, 0] - W["th1"]
    r2 = sim[:, :, 2] - W["th2"]
    return r1, r2


def rms_deg(W, p):
    r1, r2 = residuals(W, p)
    return np.degrees(np.sqrt(np.mean(r1 ** 2))), np.degrees(np.sqrt(np.mean(r2 ** 2)))


def fit(W, p0, free=None, bounds=None, fit_z0=True, verbose=1, max_nfev=60):
    """Simulation-error fit. Free globals + (optionally) per-window rate offsets of the start state."""
    from scipy.optimize import least_squares
    p0 = np.asarray(p0, float)
    free = np.ones(len(p0), bool) if free is None else np.asarray(free, bool)
    nf = free.sum()
    N = len(W["z0"])
    lo = np.full(len(p0), -np.inf) if bounds is None else np.asarray(bounds[0], float)
    hi = np.full(len(p0), np.inf) if bounds is None else np.asarray(bounds[1], float)

    def unpack(q):
        p = p0.copy()
        p[free] = q[:nf]
        z0 = W["z0"].copy()
        if fit_z0:
            z0[:, 1] += q[nf:nf + N]
            z0[:, 3] += q[nf + N:nf + 2 * N]
        return p, z0

    def fun(q):
        p, z0 = unpack(q)
        r1, r2 = residuals(W, p, z0)
        return np.concatenate([r1.ravel(), r2.ravel()])

    q0 = np.concatenate([p0[free], np.zeros(2 * N if fit_z0 else 0)])
    qlo = np.concatenate([lo[free], np.full(2 * N if fit_z0 else 0, -np.inf)])
    qhi = np.concatenate([hi[free], np.full(2 * N if fit_z0 else 0, np.inf)])
    # Jacobian sparsity: each window's residuals depend on globals + its own 2 offsets
    from scipy.sparse import lil_matrix
    K1 = W["th1"].shape[1]
    S = lil_matrix((2 * N * K1, len(q0)), dtype=int)
    S[:, :nf] = 1
    if fit_z0:
        for i in range(N):
            for blk in (0, 1):
                rows = slice(blk * N * K1 + i * K1, blk * N * K1 + (i + 1) * K1)
                S[rows, nf + i] = 1
                S[rows, nf + N + i] = 1
    res = least_squares(fun, q0, bounds=(qlo, qhi), jac_sparsity=S, x_scale="jac",
                        verbose=verbose, max_nfev=max_nfev, loss="soft_l1", f_scale=np.radians(3))
    p, z0 = unpack(res.x)
    return p, z0, res
