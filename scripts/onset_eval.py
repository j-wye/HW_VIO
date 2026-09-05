#!/usr/bin/env python3
"""Jamming-onset evaluation: score the pose the live node would actually publish.

  1. Pick the onset t_J (seconds after the first estimate).
  2. Before t_J GNSS is valid. Estimate the VIO-world -> ENU transform on the
     window [t_J - W, t_J]: yaw-only rotation (both frames are gravity-aligned)
     fitted on the tracks, translation pinned so the two positions coincide at
     t_J (the last good fix). This is what the node does while GNSS is trusted.
  3. At t_J freeze it. For t > t_J: p_world = T * p_vio, error = |p_world - GT|.
  4. Optionally write TUM files from t_J with identity quaternions and cross-check
     with evo: because the heading is already applied, `--align_origin` reduces
     to the translation pin and must give the same numbers.

usage: onset_eval.py --bag datasets/AMtown03 --onset 150 [--window 30] est.csv ...
"""
import argparse
import json
import os
import subprocess

import numpy as np

from eval_vio import load_est, load_gt


def yaw_fit(pv, pg):
    A, B = pv[:, :2] - pv[:, :2].mean(0), pg[:, :2] - pg[:, :2].mean(0)
    U, _, Vt = np.linalg.svd(B.T @ A)
    C = np.diag([1.0, np.sign(np.linalg.det(U @ Vt))])
    R = np.eye(3); R[:2, :2] = U @ C @ Vt
    return R


def one(est_path, tg, pg, onset, window, evo_dir=None):
    te, pe, _ = load_est(est_path)
    k = (te >= max(te[0], tg[0])) & (te <= min(te[-1], tg[-1]))
    te, pe = te[k], pe[k]
    pgi = np.column_stack([np.interp(te, tg, pg[:, i]) for i in range(3)])
    T = te - te[0]
    w = (T >= onset - window) & (T <= onset)
    j = np.searchsorted(T, onset)
    path_w = np.linalg.norm(np.diff(pgi[w, :2], axis=0), axis=1).sum()
    R = yaw_fit(pe[w], pgi[w])
    t = pgi[j] - R @ pe[j]                       # pinned at the last good fix
    after = T >= onset
    pw = (R @ pe[after].T).T + t
    e = np.linalg.norm(pw - pgi[after], axis=1)
    Ta = T[after] - onset
    out = {'window_path_m': path_w, 'median': np.median(e), 'rmse': np.sqrt((e**2).mean()),
           'max': e.max(), 'final': e[-1]}
    for m in (30, 60, 120, 300):
        i = np.searchsorted(Ta, m)
        if i < len(e): out[f'at_{m}s'] = e[i]
    for thr in (10, 15):
        over = np.nonzero(e > thr)[0]
        out[f't_exceed_{thr}m'] = Ta[over[0]] if len(over) else Ta[-1]
    out = {k: round(float(v), 2) for k, v in out.items()}
    if evo_dir:
        os.makedirs(evo_dir, exist_ok=True)
        base = os.path.splitext(os.path.basename(est_path))[0]
        g, s = f'{evo_dir}/{base}_gt.tum', f'{evo_dir}/{base}_world.tum'
        with open(g, 'w') as f:
            for ti, p in zip(te[after], pgi[after]): f.write('%.9f %.6f %.6f %.6f 0 0 0 1\n' % (ti, *p))
        with open(s, 'w') as f:
            for ti, p in zip(te[after], pw): f.write('%.9f %.6f %.6f %.6f 0 0 0 1\n' % (ti, *p))
        for flag in ('--align_origin', ''):
            r = subprocess.run(['evo_ape', 'tum', g, s, '-r', 'trans_part', '--t_max_diff', '0.1'] + ([flag] if flag else []),
                               capture_output=True, text=True).stdout
            vals = {ln.split()[0]: float(ln.split()[1]) for ln in r.splitlines() if ln.strip().split()[:1] and ln.split()[0] in ('rmse', 'median')}
            out[f'evo{flag or "_noalign"}'] = vals
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True); ap.add_argument('--onset', type=float, required=True)
    ap.add_argument('--window', type=float, default=30.0); ap.add_argument('--evo-dir')
    ap.add_argument('est', nargs='+')
    a = ap.parse_args()
    tg, pg = load_gt(a.bag)
    rows = [one(p, tg, pg, a.onset, a.window, a.evo_dir) for p in a.est]
    num = [k for k in rows[0] if not k.startswith('evo')]
    print(json.dumps({'onset_s': a.onset, 'window_s': a.window, 'n_runs': len(rows),
                      'median_over_runs': {k: round(float(np.median([r[k] for r in rows])), 2) for k in num},
                      'range_over_runs': {k: [round(float(min(r[k] for r in rows)), 2), round(float(max(r[k] for r in rows)), 2)] for k in num},
                      'evo_check_run0': {k: v for k, v in rows[0].items() if k.startswith('evo')}}, indent=1))


if __name__ == '__main__':
    main()
