import sys, json, signal
from multiprocessing import Pool
sys.path.insert(0, '.')
import fake_rig_old as fr
def _alarm(*a): raise TimeoutError()
def case(kw):
    name = kw.pop('name'); signal.signal(signal.SIGALRM, _alarm); signal.alarm(150)
    try: r = fr.run_case(**kw)
    except BaseException as e: r = dict(held=False, stop=[f'harness {type(e).__name__} {e}'])
    signal.alarm(0); return name, r
C = []
for i in (0, 1):
    for k, v in [('m', 0.85), ('m', 1.15)]:
        C.append(dict(name=f'OLD script, truth=old, arm{i+1} {k} x{v}', seed=0, arm3_true='old', scale={i: {k: v}}))
    C.append(dict(name=f'OLD script, truth=old, arm{i+1} c +20mm', seed=0, arm3_true='old', scale={i: {'c': 0.02}}))
C.append(dict(name='OLD script, truth=new', seed=0, arm3_true='new'))
if __name__ == '__main__':
    out = {}
    with Pool(2) as p:
        for name, r in p.imap_unordered(case, C):
            out[name] = r; print(f"{name:45s} held={r['held']} maxx={r.get('max_x_mm',0):.0f}mm stop={r.get('stop')[:1]}", flush=True)
    json.dump(out, open('batch_old_results.json', 'w'), indent=1)
