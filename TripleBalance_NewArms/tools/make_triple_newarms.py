"""usage: python3 make_triple_newarms.py PythonMaster3_triple.py PythonMaster3_triple_NewArms.py
Make the NewArms triple script from PythonMaster3_triple.py (unchanged): arm values from Barney's
2026-11-20 estimates (DoubleSwingUp_NewArms/tools/arm_values.py), --arms new|old."""
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()
def rep(old, new, count=1):
    global s
    assert s.count(old) == count, (s.count(old), old[:80])
    s = s.replace(old, new)

rep('''"""PythonMaster3_triple.py - balance the TRIPLE pendulum (V2 arms + arm 3) on the cart.
''', '''"""PythonMaster3_triple_NewArms.py - COPY of PythonMaster3_triple.py with the arm values Barney
estimated on 2026-11-20 (PythonMaster3_triple.py is not changed). Changes only: ARM_MASS_KG,
ARM_COM_M, ARM_INERTIA_KG_M2, ARM3_PRESETS = the new values; --arms old runs the 2026-11-18 values.

PythonMaster3_triple.py - balance the TRIPLE pendulum (V2 arms + arm 3) on the cart.
''')
rep('''# Mass of each arm alone, kg: everything that swings with it except the arms above it (rod,
# weights, the transmitter board and battery on its hub, the next joint's shaft parts).
ARM_MASS_KG = [0.098136, 0.161803, 0.169850]
# Distance from the arm's own pivot to its centre of mass, m.
ARM_COM_M = [0.147803, 0.111518, 0.135413]
# Pivot to the next joint, m (arm 3: pivot to tip; it does not enter the dynamics).
ARM_LENGTH_M = [0.200, 0.250, 0.300]
# Moment of inertia of each arm about its OWN centre of mass, kg m^2.
ARM_INERTIA_KG_M2 = [3.39766e-4, 2.02321e-3, 3.05751e-3]
''', '''# NewArms (2026-11-20, Barney's estimates [HYP]). Each arm = PLA bar 34.5 g per 200 mm (uniform) +
# parts placed from its END shaft: 2x608 bearings 23.6 g on the end axis, 8x100 shaft 39.2 g + 2
# collars 11.0 g on the end axis (arms 1 and 2 only), rest of the hub (XIAO, LiPo, MT6701, switch)
# 57.1 g centred 29.1 mm inboard (radius of gyration 12 mm assumed). Arms 1/2/3 = 200/250/300 mm.
# Here each joint's shaft and collars are counted in the arm BELOW it (where they sit, at its end),
# not in the arm above as in PythonMaster3_triple.py. A mass on the joint axis moves the arms the
# same either way, so only the bookkeeping differs.
# Mass of each arm alone, kg: everything that swings with it except the arms above it.
ARM_MASS_KG = [0.165400, 0.174025, 0.132450]
# Distance from the arm's own pivot to its centre of mass, m.
ARM_COM_M = [0.169095, 0.209476, 0.228848]
# Pivot to the next joint, m (arm 3: pivot to tip; it does not enter the dynamics).
ARM_LENGTH_M = [0.200, 0.250, 0.300]
# Moment of inertia of each arm about its OWN centre of mass, kg m^2.
ARM_INERTIA_KG_M2 = [3.58603e-4, 6.69226e-4, 9.38529e-4]
# The 2026-11-18 values (PythonMaster3_triple.py, STL model, shafts counted in the arm above): --arms old
OLD_ARMS = ([0.098136, 0.161803, 0.169850], [0.147803, 0.111518, 0.135413], [3.39766e-4, 2.02321e-3, 3.05751e-3],
            {"full": (0.169850, 0.135413, 3.05751e-3), "bare": (0.138350, 0.116183, 2.46953e-3)})
''')
rep('''ARM3_PRESETS = {
    "full": (0.169850, 0.135413, 3.05751e-3),
    "bare": (0.138350, 0.116183, 2.46953e-3),
}''', '''# NewArms: "full" = Barney's arm 3 (hub parts and both bearings, no shaft or collars);
# "bare" = only the bar and the two bearings.
ARM3_PRESETS = {
    "full": (0.132450, 0.228848, 9.38529e-4),
    "bare": (0.075350, 0.196981, 7.52813e-4),
}''')
rep('''    ARM3_PRESET = args.arm3
    m3, c3, I3 = ARM3_PRESETS[ARM3_PRESET]
''', '''    if args.arms == "old":
        ARM_MASS_KG[:], ARM_COM_M[:], ARM_INERTIA_KG_M2[:] = (list(v) for v in OLD_ARMS[:3])
        ARM3_PRESETS.clear()
        ARM3_PRESETS.update(OLD_ARMS[3])
    ARM3_PRESET = args.arm3
    m3, c3, I3 = ARM3_PRESETS[ARM3_PRESET]
''')
rep('''    ap.add_argument("--arm3-mass-g",''', '''    ap.add_argument("--arms", choices=("new", "old"), default="new",
                    help="arm values: new = 2026-11-20 estimates (default), old = PythonMaster3_triple.py's 2026-11-18 values")
    ap.add_argument("--arm3-mass-g",''')
open(dst, "w", encoding="utf-8").write(s)
print("wrote", dst)
