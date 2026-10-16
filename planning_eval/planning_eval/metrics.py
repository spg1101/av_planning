"""Pure metric functions (unit-tested, no ROS)."""

import numpy as np


def match_by_time(t_est, t_gt, max_dt=0.05):
    """For each estimate time, index of the nearest ground-truth time (or -1 if too far)."""
    t_gt = np.asarray(t_gt)
    idx = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    left, right = idx - 1, idx
    nearest = np.where(np.abs(t_gt[left] - t_est) <= np.abs(t_gt[right] - t_est), left, right)
    nearest[np.abs(t_gt[nearest] - t_est) > max_dt] = -1
    return nearest


def position_rmse(est_xy, gt_xy) -> float:
    err = np.linalg.norm(np.asarray(est_xy) - np.asarray(gt_xy), axis=1)
    return float(np.sqrt(np.mean(err**2)))
