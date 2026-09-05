#!/usr/bin/env python3
"""Quantitative evaluation of a VIO trajectory against the RTK ground truth.

    python3 scripts/eval_vio.py --est <out.csv> --bag <bag_dir>
    python3 scripts/eval_vio.py --selftest        # validate the evaluator itself

Reports two alignments, because either one alone lies:
  * SE3  (metric, scale fixed at 1) -- the metric error, reported as the headline number.
  * Sim3 (scale free)               -- separates "wrong shape" from "wrong scale".

and these, which are the ones that actually decide things for us:

  shape_err_ratio = Sim3 RMSE / (RMS radius of GT about its centroid)
      1.0 means the estimate carries as little shape information as collapsing
      every pose onto the GT centroid. An estimate that shoots off in a straight
      line makes Sim3 shrink the scale until it is a dot, which makes the raw
      Sim3 RMSE look small -- this ratio is what stops that from fooling us.

  drift_1s = || (p_est(t+1s) - p_est(t)) - (p_gt(t+1s) - p_gt(t)) ||

Covariance is scored next to the error, never instead of it (project rule, 2026-09-03):
when the CSV carries the filter's P00..P88 diagonals, the report also has
  sigma_pos_*            sqrt(P66+P77+P88), the filter's own position 1-sigma [m]
  sigma_vel_median_mps   sqrt(P33+P44+P55)
  cons_global_ratio      median |SE3-aligned err| / sigma_pos   (~1 = honest, >1 = overconfident,
                                                                 <1 = underconfident)
  cons_head_ratio        same, aligned on the first 30 s only    (causal, what a live score sees)
  cons_1s_ratio          median drift_1s / (sigma_vel * 1 s)     (the gap-filling prediction)
  spearman_sigma_pos_drift1s  rank correlation: does sigma rise when the 1 s error rises?
First values on AMtown03 (2026-09-03): untuned baseline 9.8 / 22.5 / 12.8 / +0.10, guarded
deployment config 0.55 / 8.5 / 1.9 / +0.21 -- the tuning inflated sigma (bigger IMU Q) so the
global ratio flipped to underconfident while the causal and 1 s ratios stay overconfident.
These are what an accuracy-score model has to calibrate; an experiment that records only
the error cannot be revisited for that later.
      When the estimate has to bridge short gaps between absolute position fixes,
      the 1-second increment is what matters. A config can improve global RMSE and
      make this worse; both must always be reported together.

  local30s_rmse / time_within_kpi_s
      Align on the first 30 s only, then ask how large the error is and how long
      it stays under the 15 m threshold (KPI_M). Answers "once it is initialised properly, how
      long is it usable", which a whole-run RMSE cannot.
"""
import argparse
import glob
import os
import sqlite3
import sys

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
KPI_M = 15.0   # error threshold (m) behind time_within_kpi_s / divergence_s


# --------------------------------------------------------------------- data ---
def load_gt(bag_dir, gt_time_offset=0.0):
    """RTK NavSatFix -> (t, ENU xyz) with the first fix as the local origin.

    gt_time_offset is ADDED to the RTK timestamps, for bags where the DJI flight
    controller clock does not agree with the Livox/camera clock. Measured per
    sequence by cross-correlating RTK heading rate against the livox gyro
    (HKairport03 needs -2.75 s; every other sequence is within +-0.45 s). Leave at
    0.0 unless a measurement says otherwise -- this shifts the scoring reference,
    so a wrong value silently flatters or punishes the estimate.
    """
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import NavSatFix

    db = glob.glob(os.path.join(bag_dir, '*.db3'))
    if not db:
        sys.exit(f"no .db3 in {bag_dir}")
    con = sqlite3.connect('file:%s?mode=ro' % db[0], uri=True)
    row = con.execute("SELECT id FROM topics WHERE name='/ground_truth/fix'").fetchone()
    if row is None:
        sys.exit(f"{bag_dir} has no /ground_truth/fix")
    t, lat, lon, alt = [], [], [], []
    for (blob,) in con.execute(
            'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (row[0],)):
        m = deserialize_message(bytes(blob), NavSatFix)
        t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
        lat.append(m.latitude); lon.append(m.longitude); alt.append(m.altitude)
    t = np.asarray(t) + gt_time_offset
    lat = np.asarray(lat); lon = np.asarray(lon); alt = np.asarray(alt)
    e2 = WGS84_F * (2 - WGS84_F)
    s = np.sin(np.radians(lat[0]))
    rn = WGS84_A / np.sqrt(1 - e2 * s * s)
    dE = np.radians(lon - lon[0]) * rn * np.cos(np.radians(lat[0]))
    dN = np.radians(lat - lat[0]) * (rn * (1 - e2) / (1 - e2 * s * s))
    return t, np.column_stack([dE, dN, alt - alt[0]])


def load_est(path):
    d = np.genfromtxt(path, delimiter=',', skip_header=1)
    d = np.atleast_2d(d)
    finite = np.all(np.isfinite(d[:, :4]), axis=1)
    return d[finite, 0], d[finite, 1:4], int((~finite).sum())


def load_cov(path):
    """Filter covariance diagonals written by run_offline / run_feeder, in the same row
    order (and with the same non-finite rows dropped) as load_est.

    Column layout of those CSVs: t, p(3), q(4), v(3), bw(3), ba(3), P00..P88 -- the nine
    diagonals in MSCEqF's [attitude, velocity, position] order -- so P33..P55 are the
    velocity variances and P66..P88 the position variances. Returns (None, None) when the
    file has no covariance columns (synthetic self-test inputs), so every metric below
    degrades gracefully to 'not reported' instead of failing."""
    d = np.atleast_2d(np.genfromtxt(path, delimiter=',', skip_header=1))
    if d.shape[1] < 26:
        return None, None
    d = d[np.all(np.isfinite(d[:, :4]), axis=1)]
    sig_pos = np.sqrt(np.clip(d[:, 23] + d[:, 24] + d[:, 25], 0.0, None))
    sig_vel = np.sqrt(np.clip(d[:, 20] + d[:, 21] + d[:, 22], 0.0, None))
    return sig_pos, sig_vel


def spearman(a, b):
    """Rank correlation without scipy; nan when there is too little to rank."""
    if len(a) < 10:
        return float('nan')
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    ra -= ra.mean()
    rb -= rb.mean()
    den = np.sqrt((ra ** 2).sum() * (rb ** 2).sum())
    return float((ra * rb).sum() / den) if den > 0 else float('nan')


# ---------------------------------------------------------------- alignment ---
def umeyama(src, dst, with_scale):
    """Least-squares similarity that maps src onto dst (Umeyama 1991)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    cov = d0.T @ s0 / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    scale = (D * np.diag(S)).sum() / s0.var(0).sum() if with_scale else 1.0
    return R, scale, mu_d - scale * R @ mu_s


def apply_sim3(R, s, t, p):
    return (s * (R @ p.T)).T + t


# ------------------------------------------------------------------ metrics ---
def path_length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def drift_1s(t, est, gt, window=1.0, min_speed=1.0, sig_vel=None, sig_pos=None):
    """Error of the 1-second position increment, on the moving part of the flight.

    With the covariance given, the same windows also score the filter's own prediction
    of that error: over one second the position uncertainty it adds is sigma_vel * 1 s,
    so err / (sigma_vel * window) near 1 means the covariance is honest and well above 1
    means it is overconfident (the earlier analysis quoted the inverse, predicted/actual
    0.27-0.33 on the tuned config, i.e. ratio 3-4 here; 12.8 untuned, 1.9 guarded)."""
    j = np.searchsorted(t, t + window)
    ok = j < len(t)
    i = np.nonzero(ok)[0]
    j = j[ok]
    d_gt = gt[j] - gt[i]
    d_est = est[j] - est[i]
    moving = np.linalg.norm(d_gt, axis=1) > min_speed * window
    if moving.sum() < 10:
        return {}
    err = np.linalg.norm(d_est[moving] - d_gt[moving], axis=1)
    ref = np.linalg.norm(d_gt[moving], axis=1)
    out = {
        'drift_1s_median_m': float(np.median(err)),
        'drift_1s_p90_m': float(np.percentile(err, 90)),
        'drift_1s_rel_median_pct': float(np.median(err / ref) * 100),
        'drift_1s_n': int(moving.sum()),
    }
    if sig_vel is not None:
        pred = sig_vel[i][moving] * window
        good = pred > 0
        if good.sum() >= 10:
            out['cons_1s_ratio_median'] = float(np.median(err[good] / pred[good]))
    if sig_pos is not None:
        out['spearman_sigma_pos_drift1s'] = spearman(sig_pos[i][moving], err)
    return out


def evaluate(t, est, gt, n_dropped=0, sig_pos=None, sig_vel=None):
    m = {'n_pose': len(t), 'n_nonfinite_dropped': n_dropped,
         'duration_s': float(t[-1] - t[0]) if len(t) > 1 else 0.0}
    m['pose_rate_hz'] = float((len(t) - 1) / m['duration_s']) if m['duration_s'] > 0 else 0.0

    # --- SE3 (metric) ---
    R, s, tr = umeyama(est, gt, with_scale=False)
    e_se3 = np.linalg.norm(apply_sim3(R, s, tr, est) - gt, axis=1)
    m['se3_rmse_m'] = float(np.sqrt((e_se3 ** 2).mean()))
    m['se3_median_m'] = float(np.median(e_se3))
    m['se3_max_m'] = float(e_se3.max())
    m['se3_final_m'] = float(e_se3[-1])

    # --- the filter's own uncertainty, scored against the error it is supposed to bound ---
    if sig_pos is not None:
        m['sigma_pos_median_m'] = float(np.median(sig_pos))
        m['sigma_pos_p90_m'] = float(np.percentile(sig_pos, 90))
        m['sigma_pos_final_m'] = float(sig_pos[-1])
        pos_ok = sig_pos > 0
        if pos_ok.sum() >= 10:
            m['cons_global_ratio_median'] = float(np.median(e_se3[pos_ok] / sig_pos[pos_ok]))
    if sig_vel is not None:
        m['sigma_vel_median_mps'] = float(np.median(sig_vel))

    # --- Sim3 (scale free) ---
    Rs, ss, trs = umeyama(est, gt, with_scale=True)
    e_sim3 = np.linalg.norm(apply_sim3(Rs, ss, trs, est) - gt, axis=1)
    m['sim3_rmse_m'] = float(np.sqrt((e_sim3 ** 2).mean()))
    m['sim3_scale'] = float(ss)

    extent = float(np.sqrt(((gt - gt.mean(0)) ** 2).sum(1).mean()))
    m['gt_extent_m'] = extent
    m['shape_err_ratio'] = float(m['sim3_rmse_m'] / extent) if extent > 0 else float('nan')

    gt_len, est_len = path_length(gt), path_length(est)
    m['gt_path_m'] = gt_len
    m['path_ratio'] = float(est_len / gt_len) if gt_len > 0 else float('nan')
    m['drift_pct_of_path'] = float(m['se3_rmse_m'] / gt_len * 100) if gt_len > 0 else float('nan')

    # --- operational: align on the first 30 s, then see how long it stays usable ---
    w = t - t[0] <= 30.0
    if w.sum() >= 10:
        Rl, sl, trl = umeyama(est[w], gt[w], with_scale=False)
        e_local = np.linalg.norm(apply_sim3(Rl, sl, trl, est) - gt, axis=1)
        m['local30s_rmse_m'] = float(np.sqrt((e_local[w] ** 2).mean()))
        over = np.nonzero(e_local > KPI_M)[0]
        m['time_within_kpi_s'] = float(t[over[0]] - t[0]) if len(over) else float(t[-1] - t[0])
        big = np.nonzero(e_local > 10 * KPI_M)[0]
        m['divergence_s'] = float(t[big[0]] - t[0]) if len(big) else float('nan')
        if sig_pos is not None:
            # The causal version: the frame is fixed once at the start, as it would be in
            # flight, so this is the error a live accuracy score actually has to predict.
            pos_ok = sig_pos > 0
            if pos_ok.sum() >= 10:
                m['cons_head_ratio_median'] = float(np.median(e_local[pos_ok] / sig_pos[pos_ok]))

    m.update(drift_1s(t, apply_sim3(R, s, tr, est), gt, sig_vel=sig_vel, sig_pos=sig_pos))
    return m


def run(est_path, bag_dir, resample_hz=None, gt_time_offset=0.0):
    te, pe, dropped = load_est(est_path)
    sp, sv = load_cov(est_path)
    tg, pg = load_gt(bag_dir, gt_time_offset)
    lo, hi = max(te[0], tg[0]), min(te[-1], tg[-1])
    k = (te >= lo) & (te <= hi)
    if k.sum() < 10:
        sys.exit("estimate and ground truth do not overlap in time")
    if sp is not None:
        sp, sv = sp[k], sv[k]
    te, pe = te[k], pe[k]

    if resample_hz:
        # drift_1s steps with searchsorted(t, t+1s), so a run that only outputs at
        # 3 Hz gets measured over ~1.3 s intervals and looks worse for that reason
        # alone. Put every run on the same uniform grid before comparing rates.
        # Linear interpolation is the conservative stand-in: the deployed node fills
        # these gaps by IMU propagation, which is strictly better than a straight line.
        tu = np.arange(te[0], te[-1], 1.0 / resample_hz)
        pe = np.column_stack([np.interp(tu, te, pe[:, i]) for i in range(3)])
        if sp is not None:
            sp, sv = np.interp(tu, te, sp), np.interp(tu, te, sv)
        te = tu

    pg_i = np.column_stack([np.interp(te, tg, pg[:, i]) for i in range(3)])
    return evaluate(te, pe, pg_i, dropped, sig_pos=sp, sig_vel=sv)


# ----------------------------------------------------------------- selftest ---
def selftest():
    """The evaluator must recover a known transform exactly, or nothing below it means anything."""
    rng = np.random.default_rng(0)
    t = np.arange(0, 120, 0.1)
    gt = np.column_stack([60 * np.sin(t / 12), 40 * np.cos(t / 9), 5 * np.sin(t / 30)])
    ok = True

    th = 0.7
    R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    est = (R @ gt.T).T + np.array([12.0, -34.0, 5.0])
    m = evaluate(t, est, gt)
    print(f"  rigid transform      -> SE3 RMSE {m['se3_rmse_m']:.3e} m (want 0), "
          f"sim3 scale {m['sim3_scale']:.6f} (want 1), path ratio {m['path_ratio']:.6f} (want 1)")
    ok &= m['se3_rmse_m'] < 1e-9 and abs(m['sim3_scale'] - 1) < 1e-9 and abs(m['path_ratio'] - 1) < 1e-9

    m = evaluate(t, gt * 0.5, gt)
    print(f"  half scale           -> Sim3 RMSE {m['sim3_rmse_m']:.3e} m (want 0), "
          f"sim3 scale {m['sim3_scale']:.6f} (want 2), path ratio {m['path_ratio']:.6f} (want 0.5)")
    ok &= m['sim3_rmse_m'] < 1e-9 and abs(m['sim3_scale'] - 2) < 1e-9 and abs(m['path_ratio'] - 0.5) < 1e-9

    # a straight runaway: Sim3 RMSE alone looks fine, shape_err_ratio must expose it
    run_away = np.column_stack([t * 40, t * 0, t * 0])
    m = evaluate(t, run_away, gt)
    print(f"  straight runaway     -> Sim3 RMSE {m['sim3_rmse_m']:.1f} m, "
          f"shape_err_ratio {m['shape_err_ratio']:.3f} (want ~1: no shape information)")
    ok &= m['shape_err_ratio'] > 0.8

    # exact estimate must give zero 1 s drift; a 3% scale error must show up as ~3%
    m = evaluate(t, gt.copy(), gt)
    print(f"  exact               -> drift_1s median {m.get('drift_1s_median_m', -1):.3e} m (want 0)")
    ok &= m.get('drift_1s_median_m', 1) < 1e-9
    m = evaluate(t, gt * 1.03, gt)
    print(f"  +3% scale           -> drift_1s relative {m.get('drift_1s_rel_median_pct', -1):.2f} % (want 3.00)")
    ok &= abs(m.get('drift_1s_rel_median_pct', 0) - 3.0) < 0.01

    print(f"\n  SELFTEST {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--est'); ap.add_argument('--bag')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--resample', type=float, default=None,
                    help='interpolate the estimate onto a uniform grid [Hz] before '
                         'evaluating, so runs at different update rates compare fairly')
    ap.add_argument('--gt-time-offset', type=float, default=0.0,
                    help='seconds added to the RTK ground-truth timestamps, for bags whose '
                         'DJI clock disagrees with the Livox/camera clock (HKairport03: -2.75)')
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    if not (a.est and a.bag):
        sys.exit("need --est and --bag (or --selftest)")
    m = run(a.est, a.bag, a.resample, a.gt_time_offset)
    if a.json:
        import json
        print(json.dumps(m, indent=2))
    else:
        for k, v in m.items():
            print(f"  {k:26s} {v:.4f}" if isinstance(v, float) else f"  {k:26s} {v}")


if __name__ == '__main__':
    main()
