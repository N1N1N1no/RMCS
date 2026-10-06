#!/usr/bin/env python3
"""Plot current yaw PID A/B in a common reference frame using logged control error.

The reconstructed world-relative curve is target offset minus control angle error;
it is derived from the controller signal, not an independent world-angle sensor.
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from compare_feedforward import load


ROOT = Path(__file__).resolve().parent
RUN_DIR = ROOT / 'controller_ab_current_2026-10-02'
OUT = ROOT / 'current_ki005_pid_diagnostic.png'


def frame_terms(data):
    center = np.median(data['angle'][:100])
    encoder_offset = data['angle'] - center
    apparent_gap = data['reference'] - encoder_offset
    frame_difference = apparent_gap - data['error']
    reconstructed_world = data['reference'] - data['error']
    return encoder_offset, apparent_gap, frame_difference, reconstructed_world


def main():
    original = load(RUN_DIR / '01_original.csv')
    final = load(RUN_DIR / '02_final.csv')
    _, _, _, original_world = frame_terms(original)
    _, final_gap, final_frame, final_world = frame_terms(final)
    deg = np.rad2deg

    fig, axes = plt.subplots(3, 1, figsize=(11.5, 8.5), sharex=True,
                             constrained_layout=True)
    axes[0].plot(final['t'], deg(final['reference']), 'k--', lw=1.3,
                 label='commanded yaw in odom frame')
    axes[0].plot(original['t'], deg(original_world), color='#3465a4', lw=1.0,
                 label='original reconstructed yaw')
    axes[0].plot(final['t'], deg(final_world), color='#e66100', lw=1.0,
                 label='current Ki=0.05 reconstructed yaw')
    axes[0].set_ylabel('odom-frame yaw offset (deg)')
    axes[0].legend(loc='upper right', fontsize=8)

    axes[1].plot(original['t'], deg(original['error']), color='#3465a4', lw=0.9,
                 label='original')
    axes[1].plot(final['t'], deg(final['error']), color='#e66100', lw=0.9,
                 label='current Ki=0.05')
    axes[1].set_ylabel('control angle error (deg)')
    axes[1].legend(loc='upper right', fontsize=8)

    axes[2].plot(final['t'], deg(final_gap), color='#777777', lw=0.9,
                 label='apparent encoder-target gap')
    axes[2].plot(final['t'], deg(final_frame), color='#6a9e3f', lw=1.0,
                 label='inferred frame mismatch')
    axes[2].set_ylabel('gap on relative-encoder plot (deg)')
    axes[2].set_xlabel('time since sweep start (s)')
    axes[2].legend(loc='upper left', fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.28)
    fig.savefig(OUT, dpi=180)
    plt.close(fig)

    metrics = []
    for name in ('02_final.csv', '04_final.csv', '06_final.csv'):
        data = load(RUN_DIR / name)
        _, apparent_gap, frame_difference, _ = frame_terms(data)
        mask = (data['t'] >= 14.65) & (data['t'] < 14.90)
        metrics.append({
            'csv': str(RUN_DIR / name),
            'last_hold_apparent_gap_deg': float(deg(np.median(apparent_gap[mask]))),
            'last_hold_control_error_deg': float(deg(np.median(data['error'][mask]))),
            'last_hold_frame_difference_deg': float(deg(np.median(frame_difference[mask]))),
        })
    (ROOT / 'current_ki005_pid_diagnostic.json').write_text(
        json.dumps({'method': 'apparent_gap=reference-(encoder-encoder_initial); '
                              'frame_difference=apparent_gap-control_angle_error; '
                              'world_yaw_reconstruction=reference-control_angle_error',
                    'note': 'world-yaw curve and frame mismatch are reconstructed '
                            'from controller error, not independently measured',
                    'runs': metrics}, indent=2) + '\n')
    print(OUT)
    for row in metrics:
        print(row)


if __name__ == '__main__':
    main()
