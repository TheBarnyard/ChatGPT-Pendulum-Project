"""NewArms double on the twin [SIM]. usage: python3 check.py <pkg dir with script as PythonMaster2_swingup_V2.py + plans> <plan> <out.json>"""
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import twin_rig, twin_crn
twin_rig.TW = twin_crn.twin_rig.TW = os.environ.get("TWINPY")
NEW = ({"m": 0.165400, "c": 0.169095, "I": 3.58603e-4, "L": 0.2, "b": 1.62994e-4},
       {"m": 0.174025, "c": 0.209476, "I": 6.69226e-4, "L": 0.25, "b": 1.23589e-4})
FIT = twin_rig.physical()
twin_rig._orig_physical = twin_rig.physical
def make_physical(truth):
    def physical(scale=None):
        if str(truth).startswith("blend"):
            f = float(truth[5:])
            j1n = NEW[0]["m"] * 0 + 0.012049   # J1 of the new values
            Rn = {"P1": 5.2098, "P2": 3.0256, "J2r": 0.6894, "H": 0.6051, "B1": 0.0135, "B2": 0.0103}
            R0 = dict(twin_rig.R)
            twin_rig.R.update({k: R0[k] + f * (Rn[k] - R0[k]) for k in R0})
            a1, a2 = twin_rig._orig_physical()
            twin_rig.R.update(R0)
            return a1, a2
        a1, a2 = (dict(x) for x in (NEW if truth == "new" else FIT))
        for (arm, key), f in (scale or {}).items():
            (a1 if arm == 1 else a2)[key] *= f
        return a1, a2
    return physical
def job_run(job):
    twin_rig.physical = make_physical(job.pop("truth"))
    return twin_crn.run_job(job)
J2 = dict(lam=[8, 3, 0], dmin=4, dmean=10, qmax=8, retries=6, fail=30, a0=0.3, a1=1.8, exp=True)
FITP = [(r'^ARM_MODEL = "new"$', 'ARM_MODEL = "fit"')]
PKG, PLAN, OUT = sys.argv[1:4]
jobs = []
def add(case, truth, patches, seeds, **kw):
    for s in seeds:
        jobs.append(dict(pkg=PKG, plan=PLAN, seed=s, seconds=40.0, case=case, truth=truth, patches=patches, **kw))
add("script new | truth new | clean", "new", [], range(61, 65))
add("script new | truth new | arm2 I+15%", "new", [], range(61, 63), scale={(2, "I"): 1.15})
add("script new | truth new | arm2 m-10%", "new", [], range(61, 63), scale={(2, "m"): 0.90})
add("script new | truth new | rig radio", "new", [], range(100, 112), radio_f=J2)
add("script new | truth 11-04 fit | clean", "fit", [], range(61, 65))
add("script fit | truth new | clean", "new", FITP, range(61, 65))
if os.environ.get("BLEND"):
    b = os.environ["BLEND"]; PATCH = [(r'^ARM_MODEL = "new"$', f'ARM_MODEL = "b{int(float(b)*100)}"'), (r'^SYSID_RATIOS = .*$', 'SYSID_RATIOS = None')]
    jobs = [dict(pkg=PKG, plan=PLAN, seed=s, seconds=40.0, case=f"script b{int(float(b)*100)} | truth same | clean", truth="blend"+b, patches=PATCH) for s in (61, 62)]
elif os.environ.get("QUICK"):
    jobs = [j for j in jobs if j["case"] == "script new | truth new | clean"][:2]
if __name__ == "__main__":
    from multiprocessing import Pool
    agg = {}
    with Pool(4) as pool:
        for r in pool.imap_unordered(job_run, jobs, chunksize=1):
            a = agg.setdefault(r["case"], [0, 0, 0, 0, 0, 0]); a[0] += r["attempts"]; a[1] += r["catches"]; a[2] += int(r["first_try"]); a[3] += r["lost"]; a[4] += int(bool(r["faults"])); a[5] += 1
            print(r["case"], r["seed"], "att", r["attempts"], "catch", r["catches"], "first", r["first_try"], r["aborts"][:2], r["faults"][:1], flush=True)
    json.dump(agg, open(OUT, "w"), indent=1)
    for k, a in agg.items():
        print(f"{k:42s} runs {a[5]:2d}: attempts {a[0]:3d} catches {a[1]:3d} ({100*a[1]/max(1,a[0]):.0f}%), first try {a[2]}/{a[5]}, lost {a[3]}, fault runs {a[4]}")
