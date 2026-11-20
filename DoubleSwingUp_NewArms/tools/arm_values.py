"""Arm mass properties from Barney's 2026-11-20 layout (end parts fixed relative to the END shaft).
Per arm of length L (pivot to end shaft):
  bar: 34.5 g per 200 mm, uniform from pivot to L
  bearings 2x608 = 23.6 g on the end axis (at L)
  shaft 8x100 39.2 g + 2 collars 11.0 g on the end axis (arms with a joint above only)
  rest of hub (electronics, LiPo, MT6701, hub plastic) 57.1 g centred 29.1 mm inboard of L, own k = 12 mm [HYP]
"""
import numpy as np
BAR_G_PER_M = 34.5/0.2
BEAR=0.0236; SHAFT=0.0392+0.011; ELEC=0.0571; ELEC_IN=0.0291; ELEC_K=0.012
def arm(L, shaft=True):
    mb=BAR_G_PER_M*L/1000
    parts=[(mb, L/2, mb*L*L/12), (BEAR, L, 0.0), (ELEC, L-ELEC_IN, ELEC*ELEC_K**2)]
    if shaft: parts.append((SHAFT, L, 0.0))
    m=sum(p[0] for p in parts); c=sum(p[0]*p[1] for p in parts)/m
    Ip=sum(p[2]+p[0]*p[1]**2 for p in parts)
    return m, c, Ip-m*c*c, Ip
if __name__=="__main__":
    for name,L,s in [("arm1",.2,True),("arm2",.25,True),("arm2 tip (no shaft)",.25,False),("arm3",.3,False)]:
        m,c,Ic,Ip=arm(L,s); T=2*np.pi*np.sqrt(Ip/(m*9.81*c))
        print(f"{name:22s} m={m*1000:6.1f} g c={c*1000:6.1f} mm Icm={Ic:.4e} Ipiv={Ip:.4e} T_hang={T:.3f}s")
