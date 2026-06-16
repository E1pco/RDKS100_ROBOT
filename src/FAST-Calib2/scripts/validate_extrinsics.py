#!/usr/bin/env python3
"""Compare single-scene and multi-scene extrinsics on recorded center pairs."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Iterable

import numpy as np


DEFAULT_OUTPUT_DIR = Path("/home/sunrise/fast_ws/src/FAST-Calib2/output")


def parse_center_line(line: str) -> np.ndarray:
    points = []
    for match in re.findall(r"\{([^}]*)\}", line):
        values = [float(value.strip()) for value in match.split(",")]
        if len(values) != 3:
            raise ValueError(f"Expected 3 values in point {{{match}}}")
        points.append(values)
    if not points:
        raise ValueError(f"No points found in line: {line}")
    return np.asarray(points, dtype=float)


def load_center_blocks(path: Path) -> list[dict[str, object]]:
    lines = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    blocks: list[dict[str, object]] = []

    i = 0
    while i + 2 < len(lines):
        if (
            lines[i].startswith("time:")
            and "lidar_centers:" in lines[i + 1]
            and "qr_centers:" in lines[i + 2]
        ):
            lidar = parse_center_line(lines[i + 1])
            camera = parse_center_line(lines[i + 2])
            if lidar.shape != camera.shape:
                raise ValueError(f"Mismatched point count near {lines[i]}")
            blocks.append({"time": lines[i][len("time:") :].strip(), "lidar": lidar, "camera": camera})
            i += 3
        else:
            i += 1

    if not blocks:
        raise ValueError(f"No center blocks parsed from {path}")
    return blocks


def parse_matrix_numbers(text: str, key: str, expected_count: int) -> np.ndarray:
    match = re.search(rf"{re.escape(key)}\s*:\s*\[([^\]]+)\]", text, re.MULTILINE | re.DOTALL)
    if not match:
        raise ValueError(f"Could not find {key}: [...]")
    numbers = [float(value) for value in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", match.group(1))]
    if len(numbers) != expected_count:
        raise ValueError(f"{key} has {len(numbers)} values, expected {expected_count}")
    return np.asarray(numbers, dtype=float)


def load_extrinsic(path: Path) -> tuple[np.ndarray, np.ndarray]:
    text = path.read_text()
    rotation = parse_matrix_numbers(text, "Rcl", 9).reshape(3, 3)
    translation = parse_matrix_numbers(text, "Pcl", 3)
    return rotation, translation


def find_extrinsic_files(output_dir: Path, include_multi: bool) -> list[Path]:
    files = sorted(output_dir.glob("*/single_calib_result.txt"))
    top_level_single = output_dir / "single_calib_result.txt"
    if top_level_single.exists():
        files.insert(0, top_level_single)

    multi = output_dir / "multi_calib_result.txt"
    if include_multi and multi.exists():
        files.append(multi)

    if not files:
        raise FileNotFoundError(f"No calibration result files found under {output_dir}")
    return files


def rmse(errors: Iterable[float]) -> float:
    errors_array = np.asarray(list(errors), dtype=float)
    if errors_array.size == 0:
        return float("nan")
    return math.sqrt(float(np.mean(errors_array * errors_array)))


def evaluate_extrinsic(rotation: np.ndarray, translation: np.ndarray, blocks: list[dict[str, object]]) -> tuple[list[dict[str, float]], np.ndarray]:
    rows = []
    all_errors = []
    for index, block in enumerate(blocks, start=1):
        lidar = block["lidar"]
        camera = block["camera"]
        transformed = (rotation @ lidar.T).T + translation
        errors = np.linalg.norm(transformed - camera, axis=1)
        all_errors.extend(errors.tolist())
        rows.append(
            {
                "scene": index,
                "rmse": rmse(errors),
                "mean": float(np.mean(errors)),
                "max": float(np.max(errors)),
            }
        )
    return rows, np.asarray(all_errors, dtype=float)


def print_table(results: list[dict[str, object]]) -> None:
    name_width = max(12, max(len(str(result["name"])) for result in results))
    print(f"{'extrinsic':<{name_width}}  {'overall_rmse(m)':>15}  {'mean(m)':>9}  {'max(m)':>9}  per-scene rmse(m)")
    print("-" * (name_width + 62))
    for result in results:
        per_scene = "  ".join(f"S{row['scene']}={row['rmse']:.4f}" for row in result["rows"])
        print(
            f"{result['name']:<{name_width}}  "
            f"{result['overall_rmse']:>15.4f}  "
            f"{result['overall_mean']:>9.4f}  "
            f"{result['overall_max']:>9.4f}  "
            f"{per_scene}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate FAST-Calib2 extrinsics against all center pairs in circle_center_record.txt."
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="Directory containing calibration result files.")
    parser.add_argument(
        "--centers",
        type=Path,
        default=None,
        help="Path to merged circle_center_record.txt. Defaults to <output-dir>/circle_center_record.txt.",
    )
    parser.add_argument(
        "--extrinsic",
        action="append",
        type=Path,
        default=None,
        help="Calibration result file to evaluate. Can be passed multiple times. Defaults to all scene_*/single_calib_result.txt plus multi_calib_result.txt.",
    )
    parser.add_argument("--no-multi", action="store_true", help="Do not auto-include multi_calib_result.txt.")
    args = parser.parse_args()

    output_dir = args.output_dir.expanduser().resolve()
    centers_path = (args.centers or output_dir / "circle_center_record.txt").expanduser().resolve()
    blocks = load_center_blocks(centers_path)

    extrinsic_files = args.extrinsic or find_extrinsic_files(output_dir, include_multi=not args.no_multi)
    results = []
    for path in extrinsic_files:
        path = path.expanduser().resolve()
        rotation, translation = load_extrinsic(path)
        rows, all_errors = evaluate_extrinsic(rotation, translation, blocks)
        try:
            name = str(path.relative_to(output_dir))
        except ValueError:
            name = str(path)
        results.append(
            {
                "name": name,
                "rows": rows,
                "overall_rmse": rmse(all_errors),
                "overall_mean": float(np.mean(all_errors)),
                "overall_max": float(np.max(all_errors)),
            }
        )

    results.sort(key=lambda result: result["overall_rmse"])
    print(f"Centers: {centers_path}")
    print(f"Scenes: {len(blocks)}")
    print_table(results)
    print(f"\nBest by overall RMSE: {results[0]['name']} ({results[0]['overall_rmse']:.4f} m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
