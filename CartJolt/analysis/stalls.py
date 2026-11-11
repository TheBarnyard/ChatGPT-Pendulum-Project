"""Summarise deadscan.py output: per run, a timeline (one character per 0.1 s window) and stall onsets.

  a = arms fit the commanded cart motion at least 2x better than 'no cart acceleration' (cart alive)
  D = arms fit 'no cart acceleration' at least 2x better and well (cart stalled / locked)
  ? = undecided (usually fast arms + radio timing), . = too little commanded motion to tell

Usage: python3 stalls.py dead.json [run_logs_dir]   (the log dir adds the peak commanded accel before each onset)
"""
import glob, json, os, sys
import numpy as np


def classify(r):
    if r['dv'] < 0.25:
        return '.'
    if r['r0'] < 0.5 * r['r1'] and r['r0'] < 3:
        return 'D'
    if r['r1'] < 0.5 * r['r0'] and r['r1'] < 3:
        return 'a'
    return '?'


D = json.load(open(sys.argv[1]))
logdir = sys.argv[2] if len(sys.argv) > 2 else None
if logdir:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from winfit import load
for run, rows in D.items():
    s = ''.join(classify(r) for r in rows)
    m = ''.join(r['mode'][0] for r in rows)
    on = [i for i in range(len(s)) if s[i] == 'D' and (i == 0 or s[i - 1] != 'D')]
    print(f"{run}  alive={s.count('a')} stalled={s.count('D')} undecided={s.count('?')}")
    print('   ', s)
    print('   ', m)
    if on and logdir:
        f = glob.glob(os.path.join(logdir, f'swingup_double_*_{run}_*.csv'))[0]
        d = load(f)
        t, v = d['t'], d['v']
        a = np.diff(np.concatenate([[0], v])) / 0.01
        for i in on:
            j = np.searchsorted(t, rows[i]['t'])
            pre = slice(max(j - 40, 0), j + 50)
            print(f"    stall onset {rows[i]['t']:6.2f} s ({rows[i]['mode']}): max |a_cmd| {np.abs(a[pre]).max():4.1f} m/s^2, "
                  f"{int((np.abs(a[pre]) >= 12).sum())} commands >= 12 m/s^2, max |v| {np.abs(v[pre]).max():.2f} m/s")
