"""
Draw the corrected-results figures from a statistical_validation results file.

Usage:
    python make_result_figures.py results/validation_50runs_corrected.json [output_dir]
"""

import collections
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

METHODS = ['triangulation', 'space_carving', 'occupancy_grid', 'hybrid']
LABELS = ['Triangulation', 'Space carving', 'Occupancy grid', 'Hybrid']
COLORS = ['#2a6fdb', '#e08a1e', '#8a8a8a', '#2e9e6b']
PATTERNS = ['coordinated_attack', 'random_dispersal', 'evasive_maneuvers', 'formation_flying']


def main(path, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    d = json.load(open(path))
    st, raw = d['method_statistics'], d['raw_results']
    n = len(raw)

    # 1. Accuracy by method
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    panels = [('f1', 'F1 score vs ground truth', (0, 1)),
              ('mean_error_m', 'Mean position error (m)', None),
              ('fp', 'False positives per frame', None)]
    for ax, (key, title, ylim) in zip(axes, panels):
        m = np.array([st[x][key]['mean'] for x in METHODS])
        ci = np.array([st[x][key]['confidence_interval'] for x in METHODS])
        ax.bar(LABELS, m, color=COLORS, yerr=[m - ci[:, 0], ci[:, 1] - m], capsize=4)
        ax.set_title(title)
        ax.tick_params(axis='x', rotation=20)
        if ylim:
            ax.set_ylim(*ylim)
        ax.spines[['top', 'right']].set_visible(False)
    fig.suptitle(f'Corrected results, {n} Monte Carlo scenarios (95% bootstrap CI over scenarios)')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'fig_accuracy_by_method.png'), dpi=160)
    plt.close(fig)

    # 2. F1 by flight pattern
    f1 = collections.defaultdict(list)
    for r in raw:
        for m, v in r['method_results'].items():
            f1[(r['scenario_params']['pattern'], m)].append(np.mean(v['f1']))
    pats = [p for p in PATTERNS if (p, METHODS[0]) in f1]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    w, x = 0.2, np.arange(len(pats))
    for i, m in enumerate(METHODS):
        ax.bar(x + (i - 1.5) * w, [np.mean(f1[(p, m)]) for p in pats], w, label=LABELS[i], color=COLORS[i])
    ax.set_xticks(x, [f"{p.replace('_', ' ').title()}\n(n={len(f1[(p, METHODS[0])])})" for p in pats])
    ax.set_ylim(0, 1.0)
    ax.set_ylabel('F1')
    ax.legend(frameon=False, ncol=4, loc='lower center', bbox_to_anchor=(0.5, 1.0))
    ax.set_title('F1 by flight pattern (dense formations are hardest)', pad=28)
    ax.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'fig_f1_by_pattern.png'), dpi=160)
    plt.close(fig)

    # 3. Sensor noise vs triangulation error
    noise = np.array([r['scenario_params']['noise_level'] for r in raw])
    err = np.array([np.nanmean(r['method_results']['triangulation']['mean_error_m']) for r in raw])
    ok = np.isfinite(err)
    rho = stats.spearmanr(noise[ok], err[ok])[0]
    fig, ax = plt.subplots(figsize=(6, 4.2))
    ax.scatter(noise[ok], err[ok], color=COLORS[0])
    ax.set_xlabel('Pixel noise multiplier')
    ax.set_ylabel('Triangulation error (m)')
    ax.set_title(f'Error grows with sensor noise (Spearman r = {rho:.2f})')
    ax.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'fig_noise_vs_error.png'), dpi=160)
    plt.close(fig)

    # Per-pattern table for the README / paper
    print("| Pattern | n | " + " | ".join(LABELS) + " |")
    print("|---|---|" + "---|" * len(LABELS))
    for p in pats:
        print(f"| {p.replace('_', ' ')} | {len(f1[(p, METHODS[0])])} | "
              + " | ".join(f"{np.mean(f1[(p, m)]):.2f}" for m in METHODS) + " |")


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else 'results')
