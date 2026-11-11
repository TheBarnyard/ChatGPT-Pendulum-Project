import sys, glob, os, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from winfit import load, fit_kick
N=50; hop=10
out={}
for f in sorted(glob.glob(os.path.join(sys.argv[1], 'swingup_double_*.csv'))):
    d=load(f)
    if d['ratios'] is None or len(d['t'])<200: continue
    t=d['t']; md=d['mode']
    st=np.array([i for i in range(0,len(t)-N,hop) if np.isfinite(d['e1'][i]) and md[i]!='WAIT' and md[i]!=''])
    if len(st)==0: continue
    p1,c1,n1=fit_kick(d,st,None,N,mode='none',iters=12)
    d2=dict(d); d2['v']=np.zeros_like(d['v'])
    p0,c0,_=fit_kick(d2,st,None,N,mode='none',iters=12)
    rows=[]
    for j,s in enumerate(st):
        sl=slice(s,s+N); n=max(n1[j],1)
        w=max(np.nanmax(np.abs(d['w1'][sl])),np.nanmax(np.abs(d['w2'][sl])))
        v=d['v'][sl]; a=np.diff(np.concatenate([[d['v'][s-1] if s>0 else 0],v]))/0.01
        rows.append(dict(t=float(t[s]),mode=str(md[s]),r1=float(np.sqrt(c1[j]/n)),r0=float(np.sqrt(c0[j]/n)),
            dv=float(np.abs(np.diff(v)).sum()),vmax=float(np.abs(v).max()),amax=float(np.abs(a).max()),wmax=float(w)))
    run=os.path.basename(f)[24:30]
    out[run]=rows
    print(run,len(rows),flush=True)
json.dump(out, open(sys.argv[2] if len(sys.argv) > 2 else 'dead.json', 'w'))
