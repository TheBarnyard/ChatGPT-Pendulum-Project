import numpy as np, sys, pickle, time, json
sys.path.insert(0,'.')
import fitlib as F
d = pickle.load(open('d.pkl','rb'))
g = F.regrid(d, latency=0.0)
lo = [1,0.2,0.05,0.05,0,0,0,0,0,0]; hi=[15,6,2,2,2,2,2,2,2,2]
p0 = np.array([5.23, 2.01, 0.448, 0.401, 0.001, 0.001, 0.05, 0.0, 0.002, 0.004])
# TRAIN: 1 s windows starting on even seconds; TEST: windows starting on odd seconds (+1 s offset)
Wtr = F.make_windows(g, 1.0, 2.0, +1, +1)
t0=time.time()
pA, _, res = F.fit(Wtr, p0, bounds=(lo,hi), verbose=0, max_nfev=80)
print("full model fit", np.round(pA,5), f"{time.time()-t0:.0f}s", flush=True)
# controller-form model (viscous only: B1, B2; no Coulomb/drag)
free = np.array([1,1,1,1,1,1,0,0,0,0],bool)
pB0 = pA.copy(); pB0[6:]=0; pB0[4]=0.01; pB0[5]=0.01
pB, _, res = F.fit(Wtr, pB0, free=free, bounds=(lo,hi), verbose=0, max_nfev=80)
print("viscous-only fit", np.round(pB,5), flush=True)
sets = {
 "fit_full_2026-11-04": pA,
 "fit_viscous_2026-11-04": pB,
 "swingfit_2026-10-27": np.array([5.164,1.802,0.391,0.392,0,0.114,0,0,0,0]),
 "sysid_2026-10-27": np.array([5.107,1.864,0.389,0.384,0.100,0.0049,0,0,0,0]),
 "STL_CAD_2026-10-29": np.array([5.233,2.329,0.503,0.466,0,0,0,0,0,0]),
}
out = {"params": {k: dict(zip(F.PNAMES, map(float,v))) for k,v in sets.items()}, "valid": {}}
for L in (0.5, 1.0, 2.0, 3.0):
    Wte = F.make_windows(g, L, 2.0, +1, +1, offset=100)
    out["valid"][L] = {}
    for k,p in sets.items():
        r1,r2 = F.residuals(Wte, p)
        e = np.degrees(np.sqrt((r1**2).mean(1))), np.degrees(np.sqrt((r2**2).mean(1)))
        row = {"N": len(e[0]), "med1": float(np.median(e[0])), "med2": float(np.median(e[1])),
               "p90_1": float(np.percentile(e[0],90)), "p90_2": float(np.percentile(e[1],90)),
               "frac_both_lt5": float(np.mean((e[0]<5)&(e[1]<5)))}
        out["valid"][L][k]=row
        print(f"L={L} {k:26s} N={row['N']} median rms {row['med1']:.2f}/{row['med2']:.2f}  p90 {row['p90_1']:.1f}/{row['p90_2']:.1f}  both<5deg {row['frac_both_lt5']*100:.0f}%", flush=True)
json.dump(out, open('final_results.json','w'), indent=1)
