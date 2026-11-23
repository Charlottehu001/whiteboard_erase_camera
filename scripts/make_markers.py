"""Create a printable A4 sheet with the ArUco markers from the config.

Print output/markers_a4.pdf at 100 % scale ("actual size", no "fit to page"),
then measure the 100 mm check line and one marker with a ruler. If the marker
is not exactly markers.size_mm, write the measured value into the config.

Usage:  python scripts/make_markers.py
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from whiteboard_camera import load_config
from whiteboard_camera.rectify import _aruco_dictionary

ROOT = Path(__file__).resolve().parents[1]
DPI = 300
A4_MM = (210.0, 297.0)


def mm(value: float) -> int:
    return int(round(value / 25.4 * DPI))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=ROOT / "config" / "board.yaml")
    parser.add_argument("--out", default=ROOT / "output" / "markers_a4.pdf")
    args = parser.parse_args()

    cfg = load_config(args.config)
    dictionary = _aruco_dictionary(cfg.marker_dictionary)
    page = np.full((mm(A4_MM[1]), mm(A4_MM[0])), 255, np.uint8)

    side = mm(cfg.marker_size_mm)
    cell = cfg.marker_size_mm + 45.0  # marker plus white border and caption
    columns = max(1, int((A4_MM[0] - 20.0) // cell))
    for index, marker_id in enumerate(sorted(cfg.marker_centers_mm)):
        row, col = divmod(index, columns)
        x0 = mm(20.0 + col * cell + 10.0)
        y0 = mm(25.0 + row * cell + 10.0)
        if y0 + side > page.shape[0] - mm(30.0):
            raise SystemExit("too many or too large markers for one A4 page")
        page[y0 : y0 + side, x0 : x0 + side] = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
        cx, cy = cfg.marker_centers_mm[marker_id]
        caption = f"id {marker_id}  centre ({cx:g}, {cy:g}) mm  this side up"
        cv2.putText(page, caption, (x0, y0 - mm(3.0)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2, cv2.LINE_AA)

    # 100 mm line to verify the print scale.
    y = page.shape[0] - mm(15.0)
    cv2.line(page, (mm(20.0), y), (mm(120.0), y), 0, 3)
    for x in (mm(20.0), mm(120.0)):
        cv2.line(page, (x, y - mm(2.0)), (x, y + mm(2.0)), 0, 3)
    cv2.putText(page, "100 mm - check with a ruler", (mm(125.0), y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(page).save(out, "PDF", resolution=DPI)
    print("wrote", out)


if __name__ == "__main__":
    main()
