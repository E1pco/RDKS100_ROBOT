#!/usr/bin/env python3
"""
Pure checkerboard MVS intrinsic calibration wrapper.

Default target:
  inner corners: 7 x 10 (cols x rows; row 10, col 7)
  checker size:  23 mm

This is a thin wrapper around calibrate_mvs_charuco.py with --pattern checkerboard.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def main():
    script = Path(__file__).with_name("calibrate_mvs_charuco.py")
    defaults = [
        "--pattern", "checkerboard",
        "--squares-x", "7",
        "--squares-y", "10",
        "--checkerboard-spec", "inner",
        "--square-size-mm", "23",
        "--output", "tools/mvs_checkerboard_intrinsics.yaml",
        "--report", "tools/mvs_checkerboard_intrinsics_report.json",
    ]

    args = sys.argv[1:]
    # Keep user-specified values authoritative.
    for option in ("--pattern", "--squares-x", "--squares-y", "--checkerboard-spec", "--square-size-mm", "--output", "--report"):
        if option in args:
            i = defaults.index(option)
            defaults = defaults[:i] + defaults[i + 2:]

    sys.argv = [str(script)] + defaults + args
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
