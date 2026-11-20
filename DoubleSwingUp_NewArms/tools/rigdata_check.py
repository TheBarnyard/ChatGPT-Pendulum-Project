import numpy as np, sys, json
sys.path.insert(0,'.')
import fitlib as F, arms as A
d=F.load('ex/excite_20261104_093436/samples.csv'); g=F.regrid(d,latency=0.0)
def ratios(a1,a2,L1=0.2,B1=0.0182,B2=0.0138):
    m1,c1,I1,Ip1=a1; m2,c2,I2,Ip2=a2
    J1=Ip1+m2*L1**2
    return np.array([(m1*c1+m2*L1)/J1, m2*c2/J1, Ip2/J1, m2*L1*c2/J1, B1, B2,0,0,0,0]), J1
a1=A.arm(.2,True)
sets={"fit_2026-11-04":np.array([5.2253,2.0148,0.4506,0.4027,0.0182,0.0138,0,0,0,0])}
for nm,a2 in [("new arm2 with shaft",A.arm(.25,True)),("new arm2 tip",A.arm(.25,False))]:
    r,J1=ratios(a1,a2); sets[nm]=r; print(nm, np.round(r[:4],4), "J1",round(J1,6))
# old V2b_D physical constants
m1,c1,I1=0.207,0.087,0.00123324992; m2,c2,I2=0.210,0.106,0.00197563667
sets["old physical consts in V2b_D"],_=ratios((m1,c1,I1,I1+m1*c1*c1),(m2,c2,I2,I2+m2*c2*c2))
for L in (1.0,3.0):
    W=F.make_windows(g,L,2.0,+1,+1,offset=100)
    for k,p in sets.items():
        r1,r2=F.residuals(W,p); e1=np.degrees(np.sqrt((r1**2).mean(1))); e2=np.degrees(np.sqrt((r2**2).mean(1)))
        print(f"L={L} {k:30s} med rms {np.median(e1):.2f}/{np.median(e2):.2f} deg (N={len(e1)})")
