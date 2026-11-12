"""V2b vs V2b_D on the digital twin [SIM], paired: the same radio, encoder noise and receiver timing for
both (the Arduino USB delay has its own random stream), so only the way the cart commands are sent differs.
usage: TWINPY=<twinpy dir> python3 twin_v2bd_check.py <dir with V2b as PythonMaster2_swingup_V2.py> <dir with V2b_D as PythonMaster2_swingup_V2.py>
twin_rig.py is in DoubleSwingUp_V2/tools (branch claude/project-thread-74rdr9); twinpy comes from the TwinV3 twin_worker.js."""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import twin_rig, twin_crn
twin_rig.TW = twin_crn.twin_rig.TW = os.environ.get("TWINPY", "twinpy")
from multiprocessing import Pool
J2 = dict(lam=[8, 3, 0], dmin=4, dmean=10, qmax=8, retries=6, fail=30, a0=0.3, a1=1.8, exp=True)   # radio fitted to the 2026-11-04 rig run
COND = {"clean": dict(), "clean, arm2 I +10%": dict(scale={(2, "I"): 1.10}), "clean, arm2 m -4%": dict(scale={(2, "m"): 0.96}), "rig radio": dict(radio_f=J2)}
SEEDS = {"clean": range(61, 65), "clean, arm2 I +10%": range(61, 65), "clean, arm2 m -4%": range(61, 65), "rig radio": range(100, 154)}
A12 = [(r"^CART_ACCEL_MAX_M_S2 = 16.0$", "CART_ACCEL_MAX_M_S2 = 12.0")]
CFG = {"V2b": (sys.argv[1], []), "V2b_D": (sys.argv[2], []), "V2b_D accel-max 12": (sys.argv[2], A12)}
jobs = [dict(pkg=pk, plan="same_4.0s", seed=s, seconds=40.0, case=f"{cn}|{kn}", patches=pt, **kw)
        for cn, (pk, pt) in CFG.items() for kn, kw in COND.items() for s in SEEDS[kn]]
if __name__ == "__main__":
    agg, faults = {}, {}
    with Pool(4) as pool:
        for r in pool.imap_unordered(twin_crn.run_job, jobs, chunksize=1):
            a = agg.setdefault(r["case"], [0, 0, 0, 0, 0, 0]); a[0] += r["attempts"]; a[1] += r["catches"]; a[2] += int(r["first_try"]); a[3] += r["lost"]; a[4] += int(bool(r["faults"])); a[5] += 1
            if r["faults"]: faults.setdefault(r["case"], r["faults"][0][:200])
    json.dump({"agg": agg, "faults": faults}, open("twin_v2bd_check.json", "w"), indent=1)
    for kn in COND:
        print(kn)
        for cn in CFG:
            a = agg[f"{cn}|{kn}"]
            print(f"   {cn:19s} runs {a[5]:2d}: attempts {a[0]:3d} catches {a[1]:3d} ({100*a[1]/max(1,a[0]):.0f}%), first try {a[2]}/{a[5]}, balance lost {a[3]}, fault {a[4]}")
