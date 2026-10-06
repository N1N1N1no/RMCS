#!/usr/bin/env python3
"""Analyze the original-versus-current yaw A/B with the existing metrics."""

import analyze_controller_ab as analysis


if __name__ == "__main__":
    analysis.RUN_DIR = analysis.ROOT / "controller_ab_current_2026-10-02"
    analysis.main()
