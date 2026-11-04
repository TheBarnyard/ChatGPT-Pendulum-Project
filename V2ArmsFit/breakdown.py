import numpy as np, sys, pickle, json
sys.path.insert(0,'.')
import fitlib as F
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
d = pickle.load(open('d.pkl','rb')); g = F.regrid(d)
R = json.load(open('final_results.json'))
P = {k: np.array([v[n] for n in F.PNAMES]) for k,v in R['params'].items()}
W = F.make_windows(g, 2.0, 2.0, +1, +1, offset=100)
amp2 = np.degrees(np.abs(np.pi - np.abs(np.angle(np.exp(1j*W['th2'])))).max(1))  # max deviation from hanging (wrapped)
amp1 = np.degrees(np.abs(W['th1']-np.pi).max(1))
spin = np.degrees(np.ptp(W['th2'],1))>360
for k in ['fit_viscous_2026-11-04','sysid_2026-10-27','STL_CAD_2026-10-29']:
    r1,r2 = F.residuals(W,P[k]); e1=np.degrees(np.sqrt((r1**2).mean(1))); e2=np.degrees(np.sqrt((r2**2).mean(1)))
    for lab,m in [('arm2<45',amp2<45),('45-120',(amp2>=45)&(amp2<120)),('>=120 no spin',(amp2>=120)&~spin),('arm2 spins',spin)]:
        print(f"{k:24s} {lab:14s} n={m.sum():4d} med {np.median(e1[m]):5.2f}/{np.median(e2[m]):5.2f}")
# figure: 4 example windows, measured vs fit vs sysid vs CAD
k='fit_viscous_2026-11-04'
r1,r2=F.residuals(W,P[k]); e2=np.degrees(np.sqrt((r2**2).mean(1)))
order=np.argsort(amp2)
picks=[order[int(len(order)*q)] for q in (0.5,0.8,0.93,0.985)]
fig,ax=plt.subplots(4,2,figsize=(11,10),sharex=True)
tt=np.arange(W['th1'].shape[1])*0.01
for i,w in enumerate(picks):
    for j,key in enumerate(['th1','th2']):
        a=ax[i,j]; a.plot(tt,np.degrees(W[key][w]-np.pi),'k',lw=2,label='measured')
        for kk,c in [('fit_viscous_2026-11-04','C0'),('sysid_2026-10-27','C1'),('STL_CAD_2026-10-29','C2')]:
            s=F.simulate(W['z0'][w:w+1],W['dv'][w:w+1],0.01,P[kk])[0]
            a.plot(tt,np.degrees(s[:,0 if j==0 else 2]-np.pi),c,lw=1,label=kk)
        a.set_ylabel(f"arm {j+1} from hanging (deg)"); a.set_title(f"t={W['t0'][w]:.0f} s, {W['motion'][w]}",fontsize=9)
ax[0,0].legend(fontsize=7); ax[-1,0].set_xlabel('s'); ax[-1,1].set_xlabel('s')
plt.tight_layout(); plt.savefig('fit_examples.png',dpi=110)
