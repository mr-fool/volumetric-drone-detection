"""
Ground-truth scoring for detector outputs.

Detected targets are matched one-to-one to true drone positions with the
Hungarian algorithm; a match counts only if it is within `gate_m` metres.
Unmatched detections are false positives (including ghost points); unmatched
drones are misses.

Recall is reported two ways:
  recall           = found / all drones in the scene
  recall_observable = found among drones detected by >= 2 cameras (the most any
                      multi-view method could hope to localise)
"""

from typing import Dict, List, Optional
import numpy as np
from scipy.optimize import linear_sum_assignment


def observable_drones(ground_truth: Dict[str, List[List[int]]], min_cameras: int = 2) -> set:
    """Drone indices detected by at least `min_cameras` cameras this frame."""
    counts = {}
    for blobs in ground_truth.values():
        seen = {i for blob in blobs for i in blob}
        for i in seen:
            counts[i] = counts.get(i, 0) + 1
    return {i for i, c in counts.items() if c >= min_cameras}


def score_frame(targets, true_positions: np.ndarray, gate_m: float = 10.0,
                observable: Optional[set] = None) -> Dict[str, float]:
    true_positions = np.asarray(true_positions, dtype=float).reshape(-1, 3)
    n_true, n_det = len(true_positions), len(targets)
    errors = np.empty(0)
    matched_true = set()
    if n_true and n_det:
        P = np.array([t.position for t in targets])
        D = np.linalg.norm(P[:, None, :] - true_positions[None, :, :], axis=2)
        r, c = linear_sum_assignment(D)
        ok = D[r, c] <= gate_m
        errors = D[r, c][ok]
        matched_true = set(c[ok].tolist())
    tp = len(errors)
    fp = n_det - tp
    fn = n_true - tp
    precision = tp / n_det if n_det else (1.0 if n_true == 0 else 0.0)
    recall = tp / n_true if n_true else 1.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) else 0.0
    out = {
        'n_true': n_true, 'n_detected': n_det, 'tp': tp, 'fp': fp, 'fn': fn,
        'precision': precision, 'recall': recall, 'f1': f1,
        'mean_error_m': float(errors.mean()) if tp else float('nan'),
        'rmse_m': float(np.sqrt((errors ** 2).mean())) if tp else float('nan'),
        'errors_m': errors.tolist(),
    }
    if observable is not None:
        n_obs = len(observable)
        out['n_observable'] = n_obs
        out['recall_observable'] = len(matched_true & observable) / n_obs if n_obs else 1.0
    return out
