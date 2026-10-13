#!/usr/bin/env python3
"""Plot measured and model-predicted yaw speed fundamentals at current position."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


ROOT = Path(__file__).resolve().parent
ANALYSIS = ROOT / "current_position_yaw_tuning_chassis_disabled_retry_2026-10-13/analysis.json"
FIGURE = ROOT / "report_figures/current_position_yaw_5p3rad_0p7nm_2026-10-13.png"
TABLE = ROOT / "report_figures/current_position_yaw_5p3rad_0p7nm_2026-10-13.csv"


def complex_from_json(value):
    return complex(value["real"], value["imag"])


def main():
    data = json.loads(ANALYSIS.read_text())
    fit = data["models"]["current_position_fit_run1"]
    font = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font_manager.fontManager.addfont(font)
    plt.rcParams.update({"font.family": font_manager.FontProperties(fname=font).get_name(),
                         "font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.4), sharex=True,
                             constrained_layout=True)
    phase = np.linspace(0, 2, 600)
    rows = []
    for column, frequency in enumerate(data["frequency_hz"]):
        for row, role in enumerate(("run1", "run2")):
            record = next(r for r in data["records"]
                          if r["frequency_hz"] == frequency and r["label"].endswith(role))
            torque = complex_from_json(record["torque_phasor"])
            observed = complex_from_json(record["speed_phasor"])
            predicted = torque / (fit["B_Nm_s_per_rad"] +
                                  2j * np.pi * frequency * fit["J_kg_m2"])
            error = abs(observed - predicted) / abs(predicted)
            axis = axes[row, column]
            axis.plot(phase, np.real(observed * np.exp(2j * np.pi * phase)),
                      label="实测相对 IMU 角速度基频", lw=2, color="#1f5d99")
            axis.plot(phase, np.real(predicted * np.exp(2j * np.pi * phase)),
                      label="由实测力矩和 J、B 预测", lw=2, ls="--", color="#dd7530")
            axis.set_title(f"{frequency:g} Hz · {'拟合轮' if row == 0 else '独立留出轮'} · 误差 {error:.3f}")
            axis.set_xlim(0, 2)
            axis.grid(color="#d9e1e7", alpha=.65)
            if column == 0:
                axis.set_ylabel("相对 yaw 角速度（rad/s）")
            if row == 1:
                axis.set_xlabel("基频周期数")
            rows.append({"CSV": record["label"] + ".csv",
                         "用途": "拟合" if row == 0 else "独立留出",
                         "频率_Hz": frequency,
                         "起始_yaw_rad": record["start_angle_rad"],
                         "实测力矩基频幅值_Nm": abs(torque),
                         "实测速度基频幅值_rad_s": abs(observed),
                         "模型速度基频幅值_rad_s": abs(predicted),
                         "模型相对实测相位差_deg": np.angle(predicted / observed, deg=True),
                         "速度基频相对误差": error})
    axes[0, 0].legend(loc="lower left", fontsize=8.2)
    fig.suptitle("底盘控制取消使能后，5.3 rad、0.7 N·m 局部模型的拟合与留出对照",
                 fontsize=13, fontweight="bold")
    FIGURE.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    with TABLE.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    actual = np.mean([r["速度基频相对误差"] for r in rows if r["用途"] == "独立留出"])
    expected = data["scores_on_holdout"]["current_position_fit_run1"][
        "mean_relative_torque_phasor_error_from_imu_speed"]
    assert abs(actual - expected) < 1e-12
    print(FIGURE, TABLE, "holdout mean", actual)


if __name__ == "__main__":
    main()
