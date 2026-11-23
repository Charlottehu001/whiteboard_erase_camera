"""Ink segmentation on the rectified whiteboard image.

Idea: ink absorbs light. Compare every pixel with what the clean board looks
like at that position ("background"). The fraction of light that is missing in
the most affected colour channel is the pixel's *darkness*:

    darkness = 1 - min over B,G,R of (pixel / background)

- clean board, shadow, uneven lighting: pixel == background  -> darkness ~ 0
- coloured ink: at least one channel is absorbed             -> darkness high
- black ink: all channels are absorbed                        -> darkness high
- glare: brighter than or equal to the background             -> darkness <= 0

This is the same reasoning as "ink has high saturation or low value" in HSV,
but relative to the local background, so one threshold works across the board.

The background comes from a reference photo of the clean board when available
(best: detects broad faint residue too). Without one it is estimated from the
image itself with a morphological closing, which removes dark structures
narrower than the kernel. Ink wider than the kernel is then NOT detected.
"""
from __future__ import annotations

import cv2
import numpy as np

from .config import BoardConfig


def _odd(n: float) -> int:
    n = max(3, int(round(n)))
    return n if n % 2 == 1 else n + 1


def estimate_background(rect_bgr: np.ndarray, cfg: BoardConfig) -> np.ndarray:
    """Estimate the clean board from the image itself (no reference photo)."""
    k = _odd(cfg.background_kernel_mm * cfg.px_per_mm)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    closed = cv2.morphologyEx(rect_bgr, cv2.MORPH_CLOSE, kernel, borderType=cv2.BORDER_REPLICATE)
    return cv2.GaussianBlur(closed, (5, 5), 0)


def _match_exposure(reference: np.ndarray, image: np.ndarray, usable: np.ndarray) -> np.ndarray:
    """Scale the reference per channel so its overall brightness matches the image."""
    ref = reference.astype(np.float32)
    if not usable.any():
        return ref
    for c in range(3):
        ref_med = float(np.median(ref[..., c][usable]))
        img_med = float(np.median(image[..., c][usable]))
        if ref_med > 1.0:
            ref[..., c] *= img_med / ref_med
    return ref


def segment_ink(
    rect_bgr: np.ndarray,
    cfg: BoardConfig,
    reference_bgr: np.ndarray | None = None,
    observed: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (ink_mask, unobserved_mask), both boolean, rectified image size.

    ``observed`` marks pixels that really show the board (see rectify.observed_mask).
    ``unobserved_mask`` adds glare-saturated pixels to the not-observed ones:
    ink there cannot be seen, so "no ink" is not a safe conclusion for them.
    """
    if rect_bgr.ndim != 3 or rect_bgr.shape[2] != 3:
        raise ValueError("rect_bgr must be a BGR colour image")
    h, w = rect_bgr.shape[:2]
    if observed is None:
        observed = np.ones((h, w), bool)

    saturated = rect_bgr.min(axis=2) >= cfg.saturation_level
    unobserved = saturated | ~observed

    smooth = cv2.GaussianBlur(rect_bgr, (3, 3), 0)
    image = smooth.astype(np.float32)
    if reference_bgr is not None:
        if reference_bgr.shape != rect_bgr.shape:
            raise ValueError(
                f"reference image has shape {reference_bgr.shape}, expected {rect_bgr.shape}; "
                "re-create it with scripts/save_reference.py after changing the config"
            )
        ref_smooth = cv2.GaussianBlur(reference_bgr, (3, 3), 0)
        ref_saturated = reference_bgr.min(axis=2) >= cfg.saturation_level
        # Where the reference itself is blown out it says nothing about the board.
        unobserved |= ref_saturated
        background = _match_exposure(ref_smooth, image, ~unobserved)
    else:
        background = estimate_background(smooth, cfg).astype(np.float32)

    ratio = image / np.maximum(background, 1.0)
    darkness = 1.0 - ratio.min(axis=2)
    mask = (darkness > cfg.min_darkness) & ~unobserved

    # Remove isolated pixels, then blobs too small to be ink.
    mask_u8 = cv2.morphologyEx(
        mask.astype(np.uint8), cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    )
    min_area_px = cfg.min_component_area_mm2 * cfg.px_per_mm**2
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    keep = np.zeros(n, bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_area_px
    return keep[labels], unobserved
