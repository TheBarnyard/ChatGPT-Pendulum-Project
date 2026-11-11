import sys, glob, os, numpy as np, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from winfit import load, fit_kick
N=20; hop=5
out=[]
for f in sorted(glob.glob(os.path.join(sys.argv[1], 'swingup_double_*.csv'))):
    d=load(f)
    if d['ratios'] is None or len(d['t'])<200: continue
    md=d['mode']; v=d['v']
    st=[]
    for i in range(0,len(md)-N,hop):
        seg=md[i:i+N]
        if not np.all(seg==seg[0]) or seg[0]=='' : continue
        if not np.isfinite(d['e1'][i]): continue
        st.append(i)
    st=np.array(st)
    if len(st)==0: continue
    p0,c0,n0=fit_kick(d,st,None,N,mode='none')
    pA,cA,_=fit_kick(d,st,None,N,mode='dA')
    pk,ck,_=fit_kick(d,st,None,N,mode='k')
    dv=np.diff(np.concatenate([[0],v]))/0.01
    for j,s in enumerate(st):
        sl=slice(s,s+N)
        w=max(np.nanmax(np.abs(d['w1'][sl])),np.nanmax(np.abs(d['w2'][sl])))
        out.append(dict(run=os.path.basename(f)[15:30],t=float(d['t'][s]),mode=str(md[s]),n=int(n0[j]),
            rms0=float(np.sqrt(c0[j]/max(n0[j],1))),rmsA=float(np.sqrt(cA[j]/max(n0[j],1))),dA=float(pA[j,4]),
            rmsk=float(np.sqrt(ck[j]/max(n0[j],1))),k=float(pk[j,4]),
            amean=float(dv[sl].mean()),aabs=float(np.abs(dv[sl]).mean()),amax=float(np.abs(dv[sl]).max()),
            vabs=float(np.abs(v[sl]).mean()),vmax=float(np.abs(v[sl]).max()),
            power=float(np.mean(dv[sl]*v[sl])),wmax=float(w),
            dvsum=float(np.abs(np.diff(v[sl])).sum())))
    print(os.path.basename(f)[15:30], len(st), flush=True)
json.dump(out, open(sys.argv[2] if len(sys.argv) > 2 else 'windows.json', 'w'))
