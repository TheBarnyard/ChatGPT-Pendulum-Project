import sys, json, time
from multiprocessing import Pool
sys.path.insert(0, '/tmp/claude-0/work/triple_rig/tools')
import fake_rig as fr
R = fr.RIG_RADIO
import signal
class WallTimeout(Exception): pass
def _alarm(*a): raise WallTimeout()
def case(kw):
    name = kw.pop('name')
    signal.signal(signal.SIGALRM, _alarm); signal.alarm(90)
    try:
        r = fr.run_case(**kw)
    except BaseException as e:
        import traceback
        r = dict(held=False, stop=[f'harness {type(e).__name__} {e} ' + traceback.format_exc()[-600:].replace(chr(10),' | ')])
    signal.alarm(0)
    return name, r
CASES = []
def add(name, seeds=(0,), **kw):
    for s in seeds:
        CASES.append(dict(name=f'{name} s{s}', seed=s, **kw))
add('nominal clean radio, 1 count', seeds=(0,1))
add('clean radio, 3 counts noise', noise=3.0, seeds=(0,1))
add('rig radio, 2 counts', radio=R, noise=2.0, T_end=64.0, seeds=tuple(range(8)))
add('arm3 true bare', arm3_true='bare', seeds=(0,))
for k,v in [('m',0.8),('m',1.2),('I',0.85),('I',0.8),('I',1.2),('I',1.4)]:
    add(f'arm3 {k} x{v}', scale={2:{k:v}}, seeds=(0,))
for dc in (-0.02,0.02):
    add(f'arm3 c {dc*1000:+.0f}mm', scale={2:{'c':dc}}, seeds=(0,))
for k,v in [('m',1.15),('c',0.01)]:
    add(f'arm2 {k} {v}', scale={1:{k:v}}, seeds=(0,))
for d in (0.002,0.004,0.005,0.006):
    add(f'extra delay {d*1000:.0f} ms', extra_delay=d, seeds=(0,))
add('encoder zeros 0.3/-0.2/0.2 deg', enc_err_deg=(0.3,-0.2,0.2), seeds=(0,))
add('encoder zeros 0.5/-0.5/0.5 deg', enc_err_deg=(0.5,-0.5,0.5), seeds=(0,))
for lean in [(0,0,1.0),(0,0,1.5),(0,0,2.0),(0,1.0,0),(0,1.5,0),(0,2.0,0),(2,0,0),(4,0,0),(1,-1,1),(1,1,1)]:
    add(f'start lean {lean}', start_lean_deg=lean, seeds=(0,))
for rel in (-0.1,-0.05,0.0,0.05,0.1):
    add(f'countdown, let go {rel:+.2f}s from GO', args=('--countdown','3'), release_s=rel, start_lean_deg=(0.3,-0.3,0.3), seeds=(0,))
add('countdown + rig radio', args=('--countdown','3'), release_s=0.0, start_lean_deg=(0.3,-0.3,0.3), radio=R, noise=2.0, seeds=(0,1))
add('worst: radio+bare+delay2+zeros', radio=R, noise=2.0, arm3_true='bare', extra_delay=0.002, enc_err_deg=(0.3,-0.2,0.2), T_end=64.0, seeds=(0,1,2))
if __name__ == '__main__':
    t0 = time.time()
    only = sys.argv[1] if len(sys.argv) > 1 else None
    cases = [c for c in CASES if only is None or only in c['name']]
    out = {}
    with Pool(4) as p:
      for name, r in p.imap_unordered(case, cases):
        out[name] = r
        import re
        m = re.search(r'balanced ([0-9.]+) s', r.get('balanced',''))
        r['bal_s'] = float(m.group(1)) if m else 0.0
        print(f"{name:42s} {'HELD' if r.get('held') else 'FELL'} {r['bal_s']:5.1f}s  max x {r.get('max_x_mm',0):6.0f} mm  v {r.get('max_v',0):.2f}  th {r.get('max_th_deg',0):5.1f}  {'' if r.get('held') else (r.get('stop') or [''])[0][-110:]}")
    json.dump(out, open('/tmp/claude-0/work/triple_rig/batch1.json','w'), indent=1, default=float)
    print(f'{time.time()-t0:.0f}s')
