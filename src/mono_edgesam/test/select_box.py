#!/usr/bin/env python3

import argparse
import os
import sys

import cv2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Select an approximate SAM box and print scaled box parameters.")
    parser.add_argument("image", help="Path to the input image.")
    parser.add_argument(
        "--model-size",
        type=int,
        default=1024,
        choices=(512, 1024),
        help="Target EdgeSAM model input size. Default: 1024.")
    parser.add_argument(
        "--max-window-size",
        type=int,
        default=1280,
        help="Maximum display window edge length. Default: 1280.")
    return parser.parse_args()


def scale_for_display(image, max_window_size):
    height, width = image.shape[:2]
    scale = min(1.0, float(max_window_size) / float(max(width, height)))
    if scale == 1.0:
      return image, scale
    resized = cv2.resize(
        image,
        (int(width * scale), int(height * scale)),
        interpolation=cv2.INTER_AREA)
    return resized, scale


def print_box(label, box):
    x1, y1, x2, y2 = box
    print(f"{label}: [{x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f}]")


def calc_letterbox_mapping(img_w, img_h, model_size):
    ratio_w = float(img_w) / float(model_size)
    ratio_h = float(img_h) / float(model_size)
    ratio = max(ratio_w, ratio_h)

    if ratio == ratio_w:
        resized_w = model_size
        resized_h = int(float(img_h) / ratio)
    else:
        resized_w = int(float(img_w) / ratio)
        resized_h = model_size

    remain = resized_w % 16
    if remain != 0:
        resized_w -= remain
        ratio = float(img_w) / float(resized_w)
        resized_h = int(float(img_h) / ratio)

    if resized_h % 2 != 0:
        resized_h -= 1

    return ratio, resized_w, resized_h


def main():
    args = parse_args()
    image_path = os.path.abspath(args.image)
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image is None:
        print(f"Failed to read image: {image_path}", file=sys.stderr)
        return 1

    shown, display_scale = scale_for_display(image, args.max_window_size)
    window_name = "select subject box: drag, press ENTER/SPACE to confirm, ESC to cancel"
    roi = cv2.selectROI(window_name, shown, fromCenter=False, showCrosshair=True)
    cv2.destroyWindow(window_name)

    x, y, w, h = roi
    if w <= 0 or h <= 0:
        print("No box selected.", file=sys.stderr)
        return 2

    inv = 1.0 / display_scale
    orig_x1 = x * inv
    orig_y1 = y * inv
    orig_x2 = (x + w) * inv
    orig_y2 = (y + h) * inv

    img_h, img_w = image.shape[:2]
    ratio, resized_w, resized_h = calc_letterbox_mapping(img_w, img_h, args.model_size)
    model_box = (
        orig_x1 / ratio,
        orig_y1 / ratio,
        orig_x2 / ratio,
        orig_y2 / ratio,
    )

    print(f"image: {image_path}")
    print(f"image_size: {img_w}x{img_h}")
    print(f"model_size: {args.model_size}x{args.model_size}")
    print(f"resized_valid_area: {resized_w}x{resized_h}, scale_ratio: {ratio:.6f}")
    print_box("original_box", (orig_x1, orig_y1, orig_x2, orig_y2))
    print_box(f"box_{args.model_size}", model_box)
    print("")
    print(f"ros2 params for {args.model_size} model:")
    print(
        f"-p box_x1:={model_box[0]:.2f} "
        f"-p box_y1:={model_box[1]:.2f} "
        f"-p box_x2:={model_box[2]:.2f} "
        f"-p box_y2:={model_box[3]:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
