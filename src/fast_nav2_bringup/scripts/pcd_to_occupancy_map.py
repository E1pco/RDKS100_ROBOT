#!/usr/bin/env python3
"""Convert a FAST-LIVO2 PCD point cloud into a Nav2 2D occupancy map."""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image
import yaml


UNKNOWN = 205
FREE = 254
OCCUPIED = 0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Project a PCD point cloud to a Nav2 .yaml + .pgm occupancy map."
    )
    parser.add_argument("--pcd", required=True, help="Input PCD file.")
    parser.add_argument("--output", required=True, help="Output map yaml path.")
    parser.add_argument("--resolution", type=float, default=0.05, help="Meters per pixel.")
    parser.add_argument(
        "--ground-percentile",
        type=float,
        default=5.0,
        help="Percentile of z used as the global ground estimate.",
    )
    parser.add_argument(
        "--free-min",
        type=float,
        default=-0.20,
        help="Minimum height relative to estimated ground to mark free cells.",
    )
    parser.add_argument(
        "--free-max",
        type=float,
        default=0.20,
        help="Maximum height relative to estimated ground to mark free cells.",
    )
    parser.add_argument(
        "--occupied-min",
        type=float,
        default=0.25,
        help="Minimum height relative to estimated ground to mark occupied cells.",
    )
    parser.add_argument(
        "--occupied-max",
        type=float,
        default=2.00,
        help="Maximum height relative to estimated ground to mark occupied cells.",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=1.0,
        help="Extra meters around the cloud bounding box.",
    )
    parser.add_argument(
        "--crop-percentile",
        type=float,
        default=0.0,
        help="Crop XY map bounds by this percentile on each side to remove outliers.",
    )
    parser.add_argument(
        "--occupied-dilation",
        type=int,
        default=1,
        help="Dilate occupied cells by this many pixels.",
    )
    parser.add_argument(
        "--free-min-points",
        type=int,
        default=1,
        help="Minimum near-ground points needed to mark a cell free.",
    )
    parser.add_argument(
        "--occupied-min-points",
        type=int,
        default=1,
        help="Minimum obstacle points needed to mark a cell occupied.",
    )
    parser.add_argument(
        "--preview",
        default=None,
        help="Optional PNG preview path. Defaults to output path with .preview.png.",
    )
    return parser.parse_args()


def dilate(mask, radius):
    if radius <= 0 or not mask.any():
        return mask

    result = mask.copy()
    padded = np.pad(mask, radius, mode="constant", constant_values=False)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            y0 = radius + dy
            x0 = radius + dx
            result |= padded[y0 : y0 + mask.shape[0], x0 : x0 + mask.shape[1]]
    return result


def main():
    args = parse_args()
    pcd_path = Path(args.pcd)
    output_yaml = Path(args.output)
    output_yaml.parent.mkdir(parents=True, exist_ok=True)

    cloud = o3d.io.read_point_cloud(str(pcd_path))
    points = np.asarray(cloud.points)
    if points.size == 0:
        raise RuntimeError(f"No points loaded from {pcd_path}")

    points = points[np.isfinite(points).all(axis=1)]
    ground_z = float(np.percentile(points[:, 2], args.ground_percentile))

    if args.crop_percentile > 0.0:
        low = args.crop_percentile
        high = 100.0 - args.crop_percentile
        min_x, max_x = np.percentile(points[:, 0], [low, high])
        min_y, max_y = np.percentile(points[:, 1], [low, high])
        in_bounds = (
            (points[:, 0] >= min_x)
            & (points[:, 0] <= max_x)
            & (points[:, 1] >= min_y)
            & (points[:, 1] <= max_y)
        )
        points = points[in_bounds]
    else:
        min_x = float(points[:, 0].min())
        max_x = float(points[:, 0].max())
        min_y = float(points[:, 1].min())
        max_y = float(points[:, 1].max())

    min_x = float(min_x - args.padding)
    max_x = float(max_x + args.padding)
    min_y = float(min_y - args.padding)
    max_y = float(max_y + args.padding)
    width = int(np.ceil((max_x - min_x) / args.resolution)) + 1
    height = int(np.ceil((max_y - min_y) / args.resolution)) + 1

    cols = np.floor((points[:, 0] - min_x) / args.resolution).astype(np.int32)
    rows_bottom = np.floor((points[:, 1] - min_y) / args.resolution).astype(np.int32)
    valid = (cols >= 0) & (cols < width) & (rows_bottom >= 0) & (rows_bottom < height)
    cols = cols[valid]
    rows_bottom = rows_bottom[valid]
    rel_z = points[valid, 2] - ground_z

    free_points = (rel_z >= args.free_min) & (rel_z <= args.free_max)
    occupied_points = (rel_z >= args.occupied_min) & (rel_z <= args.occupied_max)

    free_counts = np.zeros((height, width), dtype=np.uint16)
    occupied_counts = np.zeros((height, width), dtype=np.uint16)
    np.add.at(free_counts, (rows_bottom[free_points], cols[free_points]), 1)
    np.add.at(occupied_counts, (rows_bottom[occupied_points], cols[occupied_points]), 1)

    free = free_counts >= args.free_min_points
    occupied = occupied_counts >= args.occupied_min_points
    occupied = dilate(occupied, args.occupied_dilation)

    grid_bottom = np.full((height, width), UNKNOWN, dtype=np.uint8)
    grid_bottom[free] = FREE
    grid_bottom[occupied] = OCCUPIED

    image_grid = np.flipud(grid_bottom)
    pgm_path = output_yaml.with_suffix(".pgm")
    Image.fromarray(image_grid, mode="L").save(pgm_path)

    preview = np.zeros((height, width, 3), dtype=np.uint8)
    preview[grid_bottom == UNKNOWN] = (150, 150, 150)
    preview[grid_bottom == FREE] = (255, 255, 255)
    preview[grid_bottom == OCCUPIED] = (0, 0, 0)
    preview_path = Path(args.preview) if args.preview else output_yaml.with_suffix(".preview.png")
    Image.fromarray(np.flipud(preview), mode="RGB").save(preview_path)

    metadata = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(args.resolution),
        "origin": [min_x, min_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with output_yaml.open("w", encoding="utf-8") as f:
        yaml.safe_dump(metadata, f, sort_keys=False)

    print(f"Loaded: {pcd_path}")
    print(f"Points: {points.shape[0]}")
    print(f"Ground z percentile {args.ground_percentile:g}: {ground_z:.3f} m")
    if args.crop_percentile > 0.0:
        print(f"XY crop percentile: {args.crop_percentile:g}%")
    print(f"Map size: {width} x {height} cells @ {args.resolution:g} m/cell")
    print(f"Origin: [{min_x:.3f}, {min_y:.3f}, 0.0]")
    print(f"Free cells: {int(free.sum())}")
    print(f"Occupied cells: {int(occupied.sum())}")
    print(f"Wrote: {output_yaml}")
    print(f"Wrote: {pgm_path}")
    print(f"Wrote: {preview_path}")


if __name__ == "__main__":
    main()
