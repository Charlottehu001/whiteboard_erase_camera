"""Run the camera pipeline on one image file.

Prints the result as JSON and saves a debug picture to output/.

Usage:
  python scripts/inspect_image.py data/synthetic/drawing.png
  python scripts/inspect_image.py photo.png --reference data/reference.png --show
  python scripts/inspect_image.py photo.png --min-darkness 0.08     # try a threshold

Lens data saved next to the image by ros/capture.py (photo.json) is used
automatically.
"""
import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import cv2

from whiteboard_camera import (
    RectificationError,
    WhiteboardInspector,
    camera_info_for_image,
    draw_camera_view,
    draw_debug,
    undistort,
)

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image")
    parser.add_argument("--config", default=ROOT / "config" / "board.yaml")
    parser.add_argument("--reference", help="rectified photo of the clean board (scripts/save_reference.py)")
    parser.add_argument("--out", default=ROOT / "output")
    parser.add_argument("--min-darkness", type=float, help="override segmentation.min_darkness from the config")
    parser.add_argument("--show", action="store_true", help="open the debug picture in a window")
    args = parser.parse_args()

    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        sys.exit(f"cannot read image: {args.image}")
    inspector = WhiteboardInspector.from_files(args.config, args.reference)
    if args.min_darkness is not None:
        inspector.cfg = replace(inspector.cfg, min_darkness=args.min_darkness)
    camera_matrix, dist_coeffs = camera_info_for_image(args.image)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stem = Path(args.image).stem
    try:
        result = inspector.inspect(image, camera_matrix, dist_coeffs)
    except RectificationError as error:
        # Show which markers were found, so the cause is visible.
        view_path = out / f"{stem}_camera_view.png"
        cv2.imwrite(str(view_path), draw_camera_view(undistort(image, camera_matrix, dist_coeffs), inspector))
        sys.exit(f"whiteboard not found: {error}\ncamera view: {view_path}")
    cv2.imwrite(
        str(out / f"{stem}_camera_view.png"),
        draw_camera_view(undistort(image, camera_matrix, dist_coeffs), inspector),
    )

    summary = result.to_dict()
    regions = summary.pop("regions")
    print(json.dumps(summary, indent=2))
    print(f"regions (x_mm, y_mm, r_mm): {len(regions)}")
    for x, y, r in regions:
        print(f"  {x:8.2f} {y:8.2f} {r:7.2f}")
    debug_path = out / f"{stem}_debug.png"
    debug = draw_debug(result, inspector.cfg)
    cv2.imwrite(str(debug_path), debug)
    print("debug picture:", debug_path, file=sys.stderr)
    if args.show:
        import matplotlib.pyplot as plt

        plt.figure(figsize=(11, 8))
        plt.imshow(cv2.cvtColor(debug, cv2.COLOR_BGR2RGB))
        plt.axis("off")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
