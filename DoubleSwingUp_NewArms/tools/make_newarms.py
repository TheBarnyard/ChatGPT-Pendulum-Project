"""usage: python3 make_newarms.py PythonMaster2_swingup_V2b_D.py PythonMaster2_swingup_V2b_D_NewArms.py
Make the NewArms double script from V2b_D (which stays unchanged): arm values from Barney's
2026-11-20 estimates (tools/arm_values.py), --model new|fit, plan files double_swingup_NA_<name>.json
for the new values. Everything else is V2b_D."""
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()
def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (s.count(old), old[:80])
    s = s.replace(old, new)

rep('# PythonMaster2_swingup_V2b_D.py: PythonMaster2_swingup_V2b.py with the cart commands sent the way\n',
'''# PythonMaster2_swingup_V2b_D_NewArms.py: COPY of PythonMaster2_swingup_V2b_D.py with the arm values
# Barney estimated on 2026-11-20 (V2b_D is not changed). Changes only:
#   - ARM1_* / ARM2_* = the new estimates (arm 1 200 mm, 165.4 g, CoM 169.1 mm; arm 2 250 mm,
#     174.0 g, CoM 209.5 mm, joint-3 shaft + 2 collars on its tip), inertias worked out the same way,
#   - ARM_MODEL / --model: "new" (default) builds the model from those values; "fit" runs exactly
#     like V2b_D on the 2026-11-04 rig fit (SYSID_RATIOS) with its plans; b10 ... b50 are in between (every 10 %),
#   - new-value plans are double_swingup_NA_<name>.json (re-planned 2026-11-20); fit plans keep
#     their V2 names; PLAN_SPEED_M_S 0.8 for --plan (no "same" plan exists at 0.75 for the new values).
# Everything else (cart commands every 5 ms, observer, balance, recovery, limits, CSV) is V2b_D.
# PythonMaster2_swingup_V2b_D.py: PythonMaster2_swingup_V2b.py with the cart commands sent the way
''')

rep('''# Arm 1 alone (rod, weights, TX1 board, battery, screws), kg.
ARM1_MASS_KG = 0.207
# Main pivot axis to arm 1's centre of mass, m (balance the removed arm on a knife edge).
ARM1_COM_M = 0.087
# Arm 1's moment of inertia about its own centre of mass, kg m^2.
ARM1_INERTIA_KG_M2 = 0.00123324992
# Main pivot axis to arm 2's joint axis, m.
ARM1_LENGTH_M = 0.2
# Arm 2 alone, kg.
ARM2_MASS_KG = 0.210
# Arm 2's joint axis to its centre of mass, m.
ARM2_COM_M = 0.106
# Arm 2's moment of inertia about its own centre of mass, kg m^2.
ARM2_INERTIA_KG_M2 = 0.00197563667
# Viscous friction at the pivot and the joint, N m s/rad. Raise them if the arms lose their swing
# much faster than the model (the plan then pushes a little harder to make up for it).
ARM1_PIVOT_FRICTION = 0.0009
ARM2_JOINT_FRICTION = 0.0001
''', '''# NewArms (2026-11-20, Barney's estimates [HYP], see tools/arm_values.py). Each arm = PLA bar 34.5 g
# per 200 mm (uniform) + parts placed from its END shaft: 2x608 bearings 23.6 g on the end axis,
# 8x100 shaft 39.2 g + 2 collars 11.0 g on the end axis, rest of the hub (XIAO, LiPo, MT6701,
# switch) 57.1 g centred 29.1 mm inboard (radius of gyration 12 mm assumed).
# The joint-2 shaft and collars are counted in arm 1 (it makes no difference to the motion where a
# mass sitting on the joint axis is counted, so this matches the triple script's way exactly).
# Old values (V2b_D, from the 2026-10 fitted model, unused there because SYSID_RATIOS overrides
# them): arm 1 0.207 kg / 0.087 m / 0.00123324992; arm 2 0.210 kg / 0.106 m / 0.00197563667.
# Arm 1 alone (rod, hub electronics, bearings, joint-2 shaft and collars), kg.
ARM1_MASS_KG = 0.16540
# Main pivot axis to arm 1's centre of mass, m (balance the removed arm on a knife edge).
ARM1_COM_M = 0.16910
# Arm 1's moment of inertia about its own centre of mass, kg m^2.
ARM1_INERTIA_KG_M2 = 3.5860e-4
# Main pivot axis to arm 2's joint axis, m.
ARM1_LENGTH_M = 0.2
# Arm 2 alone (250 mm, with the joint-3 shaft and 2 collars on its tip), kg.
ARM2_MASS_KG = 0.17400
# Arm 2's joint axis to its centre of mass, m.
ARM2_COM_M = 0.20950
# Arm 2's moment of inertia about its own centre of mass, kg m^2.
ARM2_INERTIA_KG_M2 = 6.6923e-4
# Viscous friction at the pivot and the joint, N m s/rad (the 2026-11-04 fit's B1, B2 times its
# J1, the same numbers the triple script uses).
ARM1_PIVOT_FRICTION = 1.62994e-4
ARM2_JOINT_FRICTION = 1.23589e-4
# "new" = build the model from the values above; "fit" = the 2026-11-04 rig fit (SYSID_RATIOS
# below) and its plans, exactly as V2b_D. Also on the command line: --model new|fit
ARM_MODEL = "new"
# In-between models for --model: the fit's ratios moved this fraction of the way to the new
# values' ratios (P1, P2, J2r, H, B1, B2). Plans double_swingup_B10_<name>.json etc.
BLEND_MODELS = {f"b{p}": p / 100 for p in range(10, 60, 10)}   # b10 ... b50
''')
rep('''SYSID_RATIOS = {"P1": 5.2253, "P2": 2.0148, "J2r": 0.4506, "H": 0.4027, "B1": 0.0182, "B2": 0.0138}
''', '''FIT_RATIOS = {"P1": 5.2253, "P2": 2.0148, "J2r": 0.4506, "H": 0.4027, "B1": 0.0182, "B2": 0.0138}
SYSID_RATIOS = FIT_RATIOS if ARM_MODEL == "fit" else None
''')
rep('''SWING_PATH = "same_4.0s"
TRAJECTORY_FILE = HERE / f"double_swingup_V2_{SWING_PATH}.json"
''', '''SWING_PATH = "same_4.0s" if ARM_MODEL == "fit" else "same_3.5s"   # NewArms: no same_4.0s plan for the new values
PLAN_PREFIX = "double_swingup_V2_" if ARM_MODEL == "fit" else "double_swingup_NA_"
TRAJECTORY_FILE = HERE / f"{PLAN_PREFIX}{SWING_PATH}.json"
''')
rep('''    ap.add_argument("--list", action="store_true", help="list the plan files in this folder and stop")
    args = ap.parse_args()
''', '''    ap.add_argument("--list", action="store_true", help="list the plan files in this folder and stop")
    ap.add_argument("--model", choices=("new", "fit", *BLEND_MODELS), default=ARM_MODEL,
                    help="arm model: new = the 2026-11-20 arm values (default), fit = the 2026-11-04 rig fit (as V2b_D), "
                         "b10 ... b50 = the fit moved 10 ... 50 %% of the way to the new values")
    args = ap.parse_args()
    if args.model != ARM_MODEL:   # other model than the file's: its default plan
        SWING_PATH = "same_4.0s" if args.model == "fit" else "same_3.5s"
    ARM_MODEL = args.model
    SYSID_RATIOS = FIT_RATIOS if ARM_MODEL == "fit" else None
    PLAN_PREFIX = "double_swingup_V2_" if ARM_MODEL == "fit" else "double_swingup_NA_"
    if ARM_MODEL in BLEND_MODELS:
        # in between: the 11-04 fit moved part of the way toward the new values (every ratio)
        _new = arm_ratios()
        _f = BLEND_MODELS[ARM_MODEL]
        SYSID_RATIOS = {k: FIT_RATIOS[k] + _f * (_new[k] - FIT_RATIOS[k]) for k in FIT_RATIOS}
        PLAN_PREFIX = f"double_swingup_{ARM_MODEL.upper()}_"
    TRAJECTORY_FILE = HERE / f"{PLAN_PREFIX}{SWING_PATH}.json"
    _r = arm_ratios()
    log("START", f"arm model '{ARM_MODEL}': P1={_r['P1']:.4f} P2={_r['P2']:.4f} J2/J1={_r['J2r']:.4f} H={_r['H']:.4f} "
                 f"B1={_r['B1']:.4f} B2={_r['B2']:.4f}")
''')
rep('''        for f in sorted(HERE.glob("double_swingup_V2_*.json")):''', '''        for f in sorted(HERE.glob(f"{PLAN_PREFIX}*.json")):''')
rep('''            print(f"{f.stem[len('double_swingup_V2_'):]:12s}''', '''            print(f"{f.stem[len(PLAN_PREFIX):]:12s}''')
rep('''        TRAJECTORY_FILE = HERE / f"double_swingup_V2_{SWING_PATH}.json"
''', '''        TRAJECTORY_FILE = HERE / f"{PLAN_PREFIX}{SWING_PATH}.json"
''')
rep('''PLAN_SPEED_M_S = 0.75
''', '''# NewArms: 0.8 (the heavier arm 2 has no "same" plan at 0.75 m/s and 8 m/s^2). Used only by --plan.
PLAN_SPEED_M_S = 0.8
''')
open(dst, "w", encoding="utf-8").write(s)
print("wrote", dst)
