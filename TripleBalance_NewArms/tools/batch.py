import sys, json, time, signal
from multiprocessing import Pool
sys.path.insert(0, '.')
import fake_rig as fr
R = fr.RIG_RADIO
def _alarm(*a): raise TimeoutError()
def case(kw):
    name = kw.pop('name'); signal.signal(signal.SIGALRM, _alarm); signal.alarm(150)
    try: r = fr.run_case(**kw)
    except BaseException as e: r = dict(held=False, stop=[f'harness {type(e).__name__} {e}'])
    signal.alarm(0); return name, r
C = []
def add(name, seeds=(0,), **kw):
    for s in seeds: C.append(dict(name=f'{name} s{s}', seed=s, **kw))
add('truth=new, clean', seeds=(0, 1))
add('truth=new, rig radio', radio=R, noise=2.0, T_end=40.0, seeds=(0, 1, 2, 3))
add('truth=old (11-18 values)', seeds=(0,))
add('truth=old arms 1-2 + new arm 3', seeds=(0,))
for i in (0, 1, 2):
    for k, v in [('m', 0.85), ('m', 1.15), ('I', 0.8), ('I', 1.4)]:
        add(f'truth=new, arm{i+1} {k} x{v}', scale={i: {k: v}})
    for dc in (-0.02, 0.02):
        add(f'truth=new, arm{i+1} c {dc*1000:+.0f}mm', scale={i: {'c': dc}})
add('truth=new, countdown', args=('--countdown', '3'), release_s=0.0, start_lean_deg=(0.3, -0.3, 0.3))
def fixtruth(c):
    if 'truth=old (' in c['name']: c['arm3_true'] = 'old'
    elif 'old arms 1-2' in c['name']: c['arm3_true'] = 'oldarms12_newarm3'
    else: c['arm3_true'] = 'new'
    return c
C = [fixtruth(c) for c in C]
if __name__ == '__main__':
    t0 = time.time(); out = {}
    with Pool(3) as p:
        for name, r in p.imap_unordered(case, C):
            out[name] = r
            print(f"{name:40s} held={r['held']} maxx={r.get('max_x_mm',0):.0f}mm stop={r.get('stop')[:1]}", flush=True)
    json.dump(out, open('batch_results.json', 'w'), indent=1); print(f'{time.time()-t0:.0f}s')
