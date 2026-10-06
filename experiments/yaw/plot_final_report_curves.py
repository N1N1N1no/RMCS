#!/usr/bin/env python3
"""Plot recorded C-car yaw runs used by the readable final report."""

from pathlib import Path
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "report_figures"
DT = 0.005


def load(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    index = np.array([int(row["index"]) for row in rows])
    if not len(index) or np.any(np.diff(index) != 1):
        raise ValueError(f"Incomplete or discontinuous data: {path}")

    def col(name):
        return np.array([float(row[name]) for row in rows])

    return {
        "time": (index - index[0]) * DT,
        "target": np.rad2deg(col("/gimbal/yaw_sweep/target_offset")),
        "error": np.rad2deg(col("/gimbal/yaw/control_angle_error")),
        "torque": col("/gimbal/yaw/torque"),
    }


def setup():
    font_manager.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Noto Sans CJK JP", "DejaVu Sans"],
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "font.size": 10,
        "axes.titlesize": 11,
        "figure.dpi": 160,
    })


def draw_pair(original, selected, labels, output):
    original = load(original)
    selected = load(selected)
    fig, axes = plt.subplots(3, 1, figsize=(10.0, 7.3), sharex=True,
                             gridspec_kw={"height_ratios": [1.0, 1.3, 1.1]})
    blue, orange = "#2364aa", "#e76f00"
    axes[0].plot(original["time"], original["target"], color="#343a40", lw=1.5)
    axes[0].set_ylabel("目标偏移 (°)")
    axes[0].set_title("相同的目标轨迹")
    for data, label, color in ((original, labels[0], blue),
                               (selected, labels[1], orange)):
        axes[1].plot(data["time"], data["error"], label=label, color=color, lw=1.4)
        axes[2].plot(data["time"], data["torque"], label=label, color=color, lw=1.0)
    axes[1].axhline(0, color="#343a40", lw=0.7)
    axes[1].set_ylabel("控制角误差 (°)")
    axes[1].legend(loc="upper right", ncol=2, frameon=False)
    axes[2].set_ylabel("反馈力矩 (N·m)")
    axes[2].set_xlabel("每轮开始后的时间 (s)")
    axes[2].legend(loc="upper right", ncol=2, frameon=False)
    axes[2].set_xlim(0, 15)
    fig.tight_layout(h_pad=0.9)
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)


def main():
    setup()
    OUT.mkdir(exist_ok=True)
    ab = ROOT / "controller_ab_current_2026-10-02"
    draw_pair(ab / "01_original.csv", ab / "02_final.csv",
              ("原控制器", "最终控制器：Ki=0.05 + 前馈"),
              OUT / "final_ab_measured_curves.png")
    ki = ROOT / "low_speed_ki_ab_2026-10-02"
    draw_pair(ki / "01_current.csv", ki / "02_ki_005.csv",
              ("Ki=0.02", "Ki=0.05"),
              OUT / "ki_measured_curves.png")


if __name__ == "__main__":
    main()
