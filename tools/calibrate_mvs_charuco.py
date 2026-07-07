#!/usr/bin/env python3
"""
Calibrate MVS camera intrinsics with an 8x11 ChArUco or plain checkerboard target.

Default board parameters:
  squares:      8 x 11
  checker size: 23 mm
  marker size:  17 mm
  dictionary:   DICT_4X4_50

Examples:
  # Offline calibration from saved images
  python3 tools/calibrate_mvs_charuco.py --images /tmp/mvs_calib/*.png

  # Capture from ROS2 MVS topic and calibrate with ChArUco
  zsh -lic 'cd ~/fast_ws && python3 tools/calibrate_mvs_charuco.py --ros --topic /left_camera/image --samples 40'

  # Plain checkerboard mode, 8x11 inner corners, 23 mm square
  zsh -lic 'cd ~/fast_ws && python3 tools/calibrate_mvs_charuco.py --pattern checkerboard --ros --topic /left_camera/image'
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

os.environ.setdefault("OPENCV_OPENCL_RUNTIME", "disabled")
import cv2
cv2.ocl.setUseOpenCL(False)
import numpy as np


DICT_NAMES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}


@dataclass
class Detection:
    image_name: str
    image_size: tuple[int, int]
    object_points: np.ndarray
    image_points: np.ndarray
    corner_count: int
    annotated: np.ndarray | None = None
    pattern_size: tuple[int, int] | None = None


def make_board(args):
    if args.dictionary not in DICT_NAMES:
        names = ", ".join(sorted(DICT_NAMES))
        raise ValueError(f"Unsupported dictionary {args.dictionary!r}. Use one of: {names}")
    dictionary = cv2.aruco.getPredefinedDictionary(DICT_NAMES[args.dictionary])
    square_length = args.square_size_mm / 1000.0
    marker_length = args.marker_size_mm / 1000.0
    return cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        square_length,
        marker_length,
        dictionary,
    )


def detect_charuco(image: np.ndarray, image_name: str, board, min_corners: int, annotate: bool) -> Detection | None:
    if image is None:
        return None
    if image.ndim == 2:
        gray = image
        display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if annotate else None
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        display = image.copy() if annotate else None

    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    if charuco_corners is None or charuco_ids is None:
        return None
    corner_count = int(len(charuco_ids))
    if corner_count < min_corners:
        return None

    object_points, image_points = board.matchImagePoints(charuco_corners, charuco_ids)
    if object_points is None or image_points is None or len(object_points) < min_corners:
        return None

    if annotate and display is not None:
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
        cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids)
        cv2.putText(
            display,
            f"{corner_count} charuco corners",
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    h, w = gray.shape[:2]
    return Detection(
        image_name=image_name,
        image_size=(w, h),
        object_points=object_points.astype(np.float32),
        image_points=image_points.astype(np.float32),
        corner_count=corner_count,
        annotated=display,
    )



def make_checkerboard_object_points(pattern_size: tuple[int, int], square_size_mm: float) -> np.ndarray:
    cols, rows = pattern_size
    square = square_size_mm / 1000.0
    obj = np.zeros((cols * rows, 1, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj[:, 0, :2] = grid * square
    return obj


def checkerboard_candidates(args) -> list[tuple[int, int]]:
    candidates = []
    raw = [(args.squares_x, args.squares_y), (args.squares_y, args.squares_x)]
    if args.checkerboard_spec in ("inner", "auto"):
        candidates.extend(raw)
    if args.checkerboard_spec in ("squares", "auto"):
        candidates.extend((max(2, x - 1), max(2, y - 1)) for x, y in raw)
    unique = []
    for size in candidates:
        if size not in unique:
            unique.append(size)
    return unique


def detect_checkerboard(image: np.ndarray, image_name: str, args, annotate: bool) -> Detection | None:
    if image is None:
        return None
    if image.ndim == 2:
        gray = image
        display_base = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if annotate else None
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        display_base = image.copy() if annotate else None

    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    best = None
    for pattern_size in checkerboard_candidates(args):
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
        if not found or corners is None:
            continue
        corner_count = int(len(corners))
        if corner_count < args.min_corners:
            continue
        best = (pattern_size, corners.astype(np.float32), corner_count)
        break

    if best is None:
        return None

    pattern_size, image_points, corner_count = best
    object_points = make_checkerboard_object_points(pattern_size, args.square_size_mm)
    display = display_base
    if annotate and display is not None:
        cv2.drawChessboardCorners(display, pattern_size, image_points, True)
        cv2.putText(
            display,
            f"{corner_count} checker corners {pattern_size[0]}x{pattern_size[1]}",
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    h, w = gray.shape[:2]
    return Detection(
        image_name=image_name,
        image_size=(w, h),
        object_points=object_points.astype(np.float32),
        image_points=image_points.astype(np.float32),
        corner_count=corner_count,
        annotated=display,
        pattern_size=pattern_size,
    )

def detect_pattern(image: np.ndarray, image_name: str, board, args, annotate: bool) -> Detection | None:
    if args.pattern == "checkerboard":
        return detect_checkerboard(image, image_name, args, annotate)
    return detect_charuco(image, image_name, board, args.min_corners, annotate)

def expand_images(patterns: Iterable[str]) -> list[Path]:
    paths: list[Path] = []
    for item in patterns:
        p = Path(item).expanduser()
        if p.is_dir():
            for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff"):
                paths.extend(sorted(p.glob(ext)))
        else:
            matches = sorted(Path(x) for x in glob.glob(str(p)))
            paths.extend(matches if matches else [p])
    return [p for p in paths if p.exists()]


def collect_from_images(args, board) -> list[Detection]:
    detections: list[Detection] = []
    image_paths = expand_images(args.images or [])
    if not image_paths:
        raise RuntimeError("No input images found. Use --images '/path/*.png' or --ros.")

    annotated_dir = Path(args.annotated_dir).expanduser() if args.annotated_dir else None
    if annotated_dir:
        annotated_dir.mkdir(parents=True, exist_ok=True)

    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        det = detect_pattern(image, str(path), board, args, bool(annotated_dir or args.preview))
        if det is None:
            print(f"[skip] {path}: pattern not detected or not enough corners")
            continue
        print(f"[use ] {path}: {det.corner_count} corners")
        detections.append(det)
        if annotated_dir and det.annotated is not None:
            cv2.imwrite(str(annotated_dir / f"{path.stem}_charuco.jpg"), det.annotated)
        if args.preview and det.annotated is not None:
            cv2.imshow("charuco detection", det.annotated)
            cv2.waitKey(50)
    if args.preview:
        cv2.destroyAllWindows()
    return detections


def draw_ros_overlay(frame, det, sample_count, target_samples, auto_mode):
    display = det.annotated.copy() if det and det.annotated is not None else frame.copy()
    corners = det.corner_count if det else 0
    status = "READY" if det else "NO TARGET"
    color = (0, 220, 0) if det else (0, 0, 255)
    lines = [
        f"{status}  corners={corners}",
        f"samples={sample_count}/{target_samples}  mode={'AUTO' if auto_mode else 'MANUAL'}",
        "Space/s: capture   a: auto/manual   q/Esc: finish",
    ]
    y = 34
    for i, text in enumerate(lines):
        c = color if i == 0 else (255, 255, 255)
        cv2.putText(display, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(display, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, c, 2, cv2.LINE_AA)
        y += 32
    return display


def collect_from_ros(args, board) -> list[Detection]:
    try:
        import rclpy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
    except Exception as exc:
        raise RuntimeError(
            "ROS mode requires sourced ROS2 environment with rclpy, sensor_msgs, and cv_bridge. "
            "Run with: zsh -lic 'cd ~/fast_ws && python3 tools/calibrate_mvs_charuco.py --ros'"
        ) from exc

    save_dir = Path(args.save_dir).expanduser() if args.save_dir else None
    if save_dir:
        save_dir.mkdir(parents=True, exist_ok=True)
    annotated_dir = Path(args.annotated_dir).expanduser() if args.annotated_dir else None
    if annotated_dir:
        annotated_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=None)
    node = rclpy.create_node("mvs_charuco_calibrator")
    bridge = CvBridge()
    detections: list[Detection] = []
    last_accept = 0.0
    frame_count = 0
    latest_frame = None
    latest_det = None
    should_finish = False
    auto_mode = args.auto
    window_name = "MVS ChArUco calibration"

    print(f"Subscribing {args.topic}; OpenCV window controls: Space/s capture, a auto/manual, q/Esc finish.")
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    def save_detection(det: Detection, image: np.ndarray):
        index = len(detections)
        if save_dir:
            cv2.imwrite(str(save_dir / f"sample_{index:03d}.png"), image)
        if annotated_dir and det.annotated is not None:
            cv2.imwrite(str(annotated_dir / f"sample_{index:03d}_charuco.jpg"), det.annotated)

    def accept_current(reason: str):
        nonlocal last_accept
        if len(detections) >= args.samples:
            return
        if latest_frame is None or latest_det is None:
            print("[skip] no valid ChArUco board in current frame")
            return
        detections.append(latest_det)
        save_detection(latest_det, latest_frame)
        last_accept = time.time()
        print(f"[sample {len(detections):02d}/{args.samples}] {latest_det.corner_count} corners ({reason})")

    def on_image(msg: Image):
        nonlocal frame_count, latest_frame, latest_det, last_accept
        if len(detections) >= args.samples:
            return
        frame_count += 1
        try:
            image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        latest_frame = image
        latest_det = detect_pattern(image, f"ros_frame_{frame_count:04d}", board, args, True)
        if auto_mode and latest_det is not None and time.time() - last_accept >= args.interval:
            accept_current("auto")

    sub = node.create_subscription(Image, args.topic, on_image, 10)
    try:
        while rclpy.ok() and len(detections) < args.samples and not should_finish:
            rclpy.spin_once(node, timeout_sec=0.03)
            if latest_frame is not None:
                display = draw_ros_overlay(latest_frame, latest_det, len(detections), args.samples, auto_mode)
                cv2.imshow(window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                should_finish = True
            elif key in (ord('s'), ord(' ')):
                accept_current("manual")
            elif key == ord('a'):
                auto_mode = not auto_mode
                print(f"[mode] {'AUTO' if auto_mode else 'MANUAL'}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyWindow(window_name)
    return detections

def calibrate(detections: list[Detection], args):
    if len(detections) < args.min_samples:
        raise RuntimeError(f"Need at least {args.min_samples} valid samples, got {len(detections)}")
    image_sizes = {d.image_size for d in detections}
    if len(image_sizes) != 1:
        raise RuntimeError(f"All images must have same size, got: {sorted(image_sizes)}")
    image_size = detections[0].image_size
    object_points = [d.object_points for d in detections]
    image_points = [d.image_points for d in detections]

    flags = 0
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=flags,
    )

    per_view_errors = []
    for det, obj, img, rvec, tvec in zip(detections, object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        err = cv2.norm(img, projected, cv2.NORM_L2) / math.sqrt(len(projected))
        per_view_errors.append(float(err))

    return {
        "camera_name": args.camera_name,
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "board": {
            "type": args.pattern,
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "detected_inner_corners": list(detections[0].pattern_size) if args.pattern == "checkerboard" and detections and detections[0].pattern_size else None,
            "square_size_mm": args.square_size_mm,
            "marker_size_mm": args.marker_size_mm if args.pattern == "charuco" else None,
            "dictionary": args.dictionary if args.pattern == "charuco" else None,
        },
        "sample_count": len(detections),
        "rms_reprojection_error": float(rms),
        "mean_view_error_px": float(np.mean(per_view_errors)),
        "max_view_error_px": float(np.max(per_view_errors)),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_model": "plumb_bob",
        "distortion_coefficients": dist_coeffs.reshape(-1).tolist(),
        "samples": [
            {
                "image": d.image_name,
                "corners": d.corner_count,
                "view_error_px": per_view_errors[i],
            }
            for i, d in enumerate(detections)
        ],
    }


def write_yaml(path: Path, result: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    k = result["camera_matrix"]
    d = result["distortion_coefficients"]
    data = {
        "camera_name": result["camera_name"],
        "image_width": result["image_width"],
        "image_height": result["image_height"],
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(x) for row in k for x in row],
        },
        "distortion_model": result["distortion_model"],
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(d),
            "data": [float(x) for x in d],
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [
                float(k[0][0]), float(k[0][1]), float(k[0][2]), 0.0,
                float(k[1][0]), float(k[1][1]), float(k[1][2]), 0.0,
                float(k[2][0]), float(k[2][1]), float(k[2][2]), 0.0,
            ],
        },
    }
    try:
        import yaml
        with path.open("w") as f:
            yaml.safe_dump(data, f, sort_keys=False)
    except Exception:
        with path.open("w") as f:
            for key, value in data.items():
                f.write(f"{key}: {json.dumps(value)}\n")


def parse_args():
    parser = argparse.ArgumentParser(description="MVS camera intrinsic calibration with a ChArUco board")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--images", nargs="+", help="Image files, globs, or directories for offline calibration")
    mode.add_argument("--ros", action="store_true", help="Collect images from ROS2 sensor_msgs/Image")
    parser.add_argument("--topic", default="/left_camera/image", help="ROS image topic for --ros mode")
    parser.add_argument("--samples", type=int, default=40, help="Target valid ROS samples")
    parser.add_argument("--min-samples", type=int, default=12, help="Minimum valid samples required for calibration")
    parser.add_argument("--pattern", default="charuco", choices=("charuco", "checkerboard"), help="Calibration target type. checkerboard uses plain chessboard corners.")
    parser.add_argument("--checkerboard-spec", default="auto", choices=("auto", "inner", "squares"), help="For --pattern checkerboard: whether --squares-x/y are inner corners, printed squares, or auto-try both plus transpose.")
    parser.add_argument("--min-corners", type=int, default=18, help="Minimum detected corners per accepted image")
    parser.add_argument("--interval", type=float, default=0.5, help="Minimum seconds between accepted ROS samples")
    parser.add_argument("--save-dir", default="", help="Optional directory to save accepted ROS frames")
    parser.add_argument("--annotated-dir", default="", help="Optional directory to save detection overlays")
    parser.add_argument("--preview", action="store_true", help="Show detection preview windows for offline images")
    parser.add_argument("--auto", action="store_true", help="In --ros mode, accept valid frames automatically. Without this flag, use Space/s to capture.")
    parser.add_argument("--output", default="tools/mvs_camera_intrinsics.yaml", help="Output ROS camera calibration YAML")
    parser.add_argument("--report", default="tools/mvs_camera_intrinsics_report.json", help="Output detailed JSON report")
    parser.add_argument("--camera-name", default="left_camera")
    parser.add_argument("--squares-x", type=int, default=8)
    parser.add_argument("--squares-y", type=int, default=11)
    parser.add_argument("--square-size-mm", type=float, default=23.0)
    parser.add_argument("--marker-size-mm", type=float, default=17.0)
    parser.add_argument("--dictionary", default="DICT_4X4_50", choices=sorted(DICT_NAMES))
    parser.add_argument("--fix-k3", action="store_true", help="Fix k3 distortion coefficient")
    return parser.parse_args()


def main():
    args = parse_args()
    board = make_board(args) if args.pattern == "charuco" else None
    detections = collect_from_ros(args, board) if args.ros else collect_from_images(args, board)
    print(f"Valid samples: {len(detections)}")
    result = calibrate(detections, args)

    output = Path(args.output).expanduser()
    report = Path(args.report).expanduser()
    write_yaml(output, result)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, indent=2, ensure_ascii=False))

    print("Calibration complete")
    print(f"  RMS reprojection error: {result['rms_reprojection_error']:.4f} px")
    print(f"  Mean view error:        {result['mean_view_error_px']:.4f} px")
    print(f"  Max view error:         {result['max_view_error_px']:.4f} px")
    print(f"  YAML:                   {output}")
    print(f"  Report:                 {report}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
