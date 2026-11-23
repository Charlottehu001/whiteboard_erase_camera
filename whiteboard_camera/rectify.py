"""Pixel <-> whiteboard mapping with a homography.

The whiteboard is a plane, so board points (X, Y) and image pixels (u, v) are
related by a single 3x3 matrix H (8 degrees of freedom). Each known point gives
two equations, so 4 points are the minimum. Here every visible ArUco marker
contributes its 4 corners, and H is fitted to all of them by least squares.
"""
from __future__ import annotations

import itertools

import cv2
import numpy as np

from .config import BoardConfig


def _aruco_dictionary(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def detect_markers(image_bgr: np.ndarray, cfg: BoardConfig) -> dict[int, np.ndarray]:
    """Find the configured markers. Returns {id: 4x2 array of corner pixels}."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY) if image_bgr.ndim == 3 else image_bgr
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(_aruco_dictionary(cfg.marker_dictionary), params)
    corners, ids, _rejected = detector.detectMarkers(gray)
    if ids is None:
        return {}
    candidates: dict[int, list[np.ndarray]] = {}
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        # Ignore ids that are not part of this setup.
        if marker_id in cfg.marker_centers_mm:
            candidates.setdefault(marker_id, []).append(
                np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
            )
    return _resolve_duplicates(candidates, cfg)


def _resolve_duplicates(candidates: dict[int, list[np.ndarray]], cfg: BoardConfig) -> dict[int, np.ndarray]:
    """Pick one detection per id when an id is seen more than once.

    A second print of a marker lying in view (e.g. a spare sheet on the table)
    has the same id as the real one. The real one is the one that fits the
    known layout together with the other markers, so every combination is tried
    and the one with the smallest homography fit error wins.
    """
    if all(len(c) == 1 for c in candidates.values()):
        return {i: c[0] for i, c in candidates.items()}
    ids = sorted(candidates)
    best, best_rms = None, float("inf")
    for choice in itertools.product(*(range(len(candidates[i])) for i in ids)):
        selection = {i: candidates[i][k] for i, k in zip(ids, choice)}
        _, rms = estimate_homography(selection, cfg)
        if rms < best_rms:
            best, best_rms = selection, rms
    if best is None:
        # Too few different markers to judge by the layout: keep the largest of each.
        best = {i: max(c, key=lambda q: cv2.contourArea(q.astype(np.float32))) for i, c in candidates.items()}
    return best


def apply_homography(H: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Map an Nx2 array of points through a 3x3 homography."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    homog = np.hstack([pts, np.ones((len(pts), 1))]) @ np.asarray(H, dtype=np.float64).T
    return homog[:, :2] / homog[:, 2:3]


def estimate_homography(
    detections: dict[int, np.ndarray], cfg: BoardConfig
) -> tuple[np.ndarray | None, float]:
    """Fit the image -> board homography from detected markers.

    Returns (H, rms) where rms is the fit error in millimetres on the marker
    corners, or (None, inf) if fewer than ``cfg.min_markers`` are visible.
    """
    if len(detections) < cfg.min_markers:
        return None, float("inf")
    image_pts = np.vstack([detections[i] for i in sorted(detections)])
    board_pts = np.vstack([cfg.marker_corners_board(i) for i in sorted(detections)])
    H, _ = cv2.findHomography(image_pts, board_pts, 0)
    if H is None:
        return None, float("inf")
    residual = apply_homography(H, image_pts) - board_pts
    rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    return H, rms


def rectify(image: np.ndarray, H_image_to_board: np.ndarray, cfg: BoardConfig, nearest=False):
    """Warp a camera image to the top-down view of the drawing area."""
    M = cfg.board_to_canvas @ H_image_to_board
    flags = cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR
    return cv2.warpPerspective(image, M, cfg.canvas_size, flags=flags, borderValue=0)


def observed_mask(image_shape, H_image_to_board: np.ndarray, cfg: BoardConfig) -> np.ndarray:
    """Rectified boolean mask of where the board is actually visible.

    False where the drawing area falls outside the camera image, inside one of
    the ``image_ignore_polygons`` (gripper), or under a marker.
    """
    h, w = image_shape[:2]
    visible = np.full((h, w), 255, np.uint8)
    for polygon in cfg.image_ignore_polygons:
        cv2.fillPoly(visible, [np.round(np.asarray(polygon)).astype(np.int32)], 0)
    mask = rectify(visible, H_image_to_board, cfg, nearest=True)

    # Markers are black squares and would be detected as ink.
    margin = 3.0
    half = cfg.marker_size_mm / 2.0 + margin
    for cx, cy in cfg.marker_centers_mm.values():
        box = np.array([[cx - half, cy - half], [cx + half, cy + half]])
        (u0, v0), (u1, v1) = apply_homography(cfg.board_to_canvas, box)
        p0 = (int(np.floor(min(u0, u1))), int(np.floor(min(v0, v1))))
        p1 = (int(np.ceil(max(u0, u1))), int(np.ceil(max(v0, v1))))
        cv2.rectangle(mask, p0, p1, 0, thickness=-1)
    return mask > 0
