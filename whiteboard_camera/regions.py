"""Turn an ink mask into a short list of circles in whiteboard millimetres.

The board is divided into grid cells of ``cell_mm``. For every cell that
contains ink, one circle is returned: the smallest circle enclosing the ink
pixels of that cell. Together the circles cover every ink pixel, long strokes
become a chain of small circles instead of one huge one, and no circle is
larger than a cell.
"""
from __future__ import annotations

import cv2
import numpy as np

from .config import BoardConfig
from .rectify import apply_homography

Circle = tuple[float, float, float]  # (x_mm, y_mm, r_mm), r = radius of the ink


def mask_to_circles(mask: np.ndarray, cfg: BoardConfig) -> list[Circle]:
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return []
    cell_px = max(1, int(round(cfg.cell_mm * cfg.px_per_mm)))
    cell_ids = (ys // cell_px).astype(np.int64) * (mask.shape[1] // cell_px + 1) + xs // cell_px

    order = np.argsort(cell_ids, kind="stable")
    xs, ys, cell_ids = xs[order], ys[order], cell_ids[order]
    starts = np.flatnonzero(np.r_[True, cell_ids[1:] != cell_ids[:-1]])
    ends = np.r_[starts[1:], len(cell_ids)]

    half_pixel_diagonal = 0.5 * np.sqrt(2.0)  # a pixel is a square, not a point
    circles: list[Circle] = []
    for a, b in zip(starts, ends):
        pts = np.column_stack([xs[a:b], ys[a:b]]).astype(np.float32)
        (u, v), r_px = cv2.minEnclosingCircle(pts)
        x_mm, y_mm = apply_homography(cfg.canvas_to_board, [[u, v]])[0]
        r_mm = (r_px + half_pixel_diagonal) / cfg.px_per_mm
        circles.append((round(float(x_mm), 2), round(float(y_mm), 2), round(float(r_mm) + 0.005, 2)))
    return _drop_contained(circles)


def _drop_contained(circles: list[Circle]) -> list[Circle]:
    """Remove circles that lie completely inside another one (stroke ends clipping a cell corner)."""
    kept: list[Circle] = []
    for i, (x, y, r) in enumerate(circles):
        inside_other = False
        for j, (x2, y2, r2) in enumerate(circles):
            if i == j:
                continue
            contained = np.hypot(x - x2, y - y2) + r <= r2
            # Two identical circles: keep the first one only.
            if contained and not (r == r2 and x == x2 and y == y2 and i < j):
                inside_other = True
                break
        if not inside_other:
            kept.append((x, y, r))
    return kept


def circles_cover(mask: np.ndarray, circles: list[Circle], cfg: BoardConfig) -> np.ndarray:
    """Boolean image of everything inside the circles (for checks and debugging)."""
    covered = np.zeros(mask.shape, np.uint8)
    for x, y, r in circles:
        u, v = apply_homography(cfg.board_to_canvas, [[x, y]])[0]
        yy, xx = np.ogrid[: mask.shape[0], : mask.shape[1]]
        covered |= ((xx - u) ** 2 + (yy - v) ** 2 <= (r * cfg.px_per_mm) ** 2).astype(np.uint8)
    return covered.astype(bool)
