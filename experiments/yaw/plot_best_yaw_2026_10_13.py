#!/usr/bin/env python3
"""Plot the selected 6.23 rad / 0.7 N m local yaw fit and its holdout.

Uses the same lock-in extraction and first-run fit as the generalization report.
The plotted signals are fundamental components reconstructed from raw CSVs,
not the full measured waveforms.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

import analyze_frequency_yaw_retest as sweep


ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'frequency_yaw_retest_2026-10-13_matched_0p7nm'
FIGURE = ROOT / 'report_figures/best_yaw_6p23rad_0p7nm_2026-10-13.png'
TABLE = ROOT / 'report_figures/best_yaw_6p23rad_0p7nm_2026-10-13.csv'
REFERENCE = ROOT / 'yaw_generalization_2026-10-13_analysis.json'


def main() -> None:
    records = sweep.load_group((DATA,))
    train = [r for r in records if r['label'].endswith('run1')]
    holdout = [r for r in records if r['label'].endswith('run2')]
    assert len(train) == len(holdout) == 2
    fit = sweep.fit(train)
    reference = json.loads(REFERENCE.read_text())['secondary_0p7Nm_6p2rad']
    expected = reference['new_local_fit']
    assert abs(fit['J_kg_m2'] - expected['J_kg_m2']) < 1e-12
    assert abs(fit['B_Nm_s_per_rad'] - expected['B_Nm_s_per_rad']) < 1e-12

    cjk_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    font_manager.fontManager.addfont(cjk_path)
    plt.rcParams.update({
        'font.family': font_manager.FontProperties(fname=cjk_path).get_name(),
        'font.size': 10,
        'axes.spines.top': False,
        'axes.spines.right': False,
    })
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 6.4), sharex=True,
                             constrained_layout=True)
    phase_cycles = np.linspace(0, 2, 600)
    rows = []
    for col, frequency in enumerate((3.125, 6.25)):
        for row, role in enumerate(('run1', 'run2')):
            r = next(item for item in records
                     if item['frequency_hz'] == frequency and item['label'].endswith(role))
            omega = 2 * np.pi * frequency
            z = fit['B_Nm_s_per_rad'] + 1j * omega * fit['J_kg_m2']
            observed = r['speed_phasor']
            predicted = r['torque_phasor'] / z
            error = abs(observed - predicted) / abs(predicted)
            ax = axes[row, col]
            measured_curve = np.real(observed * np.exp(2j * np.pi * phase_cycles))
            modeled_curve = np.real(predicted * np.exp(2j * np.pi * phase_cycles))
            ax.plot(phase_cycles, measured_curve, color='#1f5d99', lw=2,
                    label='实测相对 IMU 角速度基频')
            ax.plot(phase_cycles, modeled_curve, color='#dd7530', lw=2,
                    ls='--', label='由实测力矩和 J、B 预测')
            ax.set_title(f'{frequency:g} Hz  ·  {"拟合轮" if row == 0 else "独立留出轮"}  ·  误差 {error:.3f}')
            ax.set_xlim(0, 2)
            ax.grid(color='#d9e1e7', alpha=.65)
            if col == 0:
                ax.set_ylabel('相对 yaw 角速度（rad/s）')
            if row == 1:
                ax.set_xlabel('基频周期数')
            rows.append({
                'CSV': f'{r["label"]}.csv',
                '用途': '拟合' if row == 0 else '独立留出',
                '频率_Hz': frequency,
                '起始_yaw_rad': r['start_angle_rad'],
                '实测力矩基频幅值_Nm': abs(r['torque_phasor']),
                '实测速度基频幅值_rad_s': abs(observed),
                '模型速度基频幅值_rad_s': abs(predicted),
                '模型相对实测相位差_deg': np.angle(predicted / observed, deg=True),
                '速度基频相对误差': error,
            })
    axes[0, 0].legend(loc='lower left', fontsize=8.3, frameon=True)
    fig.suptitle('6.23 rad、0.7 N·m 局部二阶模型：训练与独立重复轮的基频对照',
                 fontsize=13, fontweight='bold')
    FIGURE.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE, dpi=220, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    with TABLE.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    holdout_mean = float(np.mean([r['速度基频相对误差'] for r in rows
                                  if r['用途'] == '独立留出']))
    assert abs(holdout_mean - reference['holdout_new_local'][
        'mean_relative_torque_error_from_imu_speed']) < 1e-12
    print(FIGURE)
    print(TABLE)
    print('J', fit['J_kg_m2'], 'B', fit['B_Nm_s_per_rad'],
          'holdout mean error', holdout_mean)


if __name__ == '__main__':
    main()
