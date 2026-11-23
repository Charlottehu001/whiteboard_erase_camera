"""Tests of the camera pipeline on synthetic images with known ground truth."""
from dataclasses import replace

import numpy as np
import pytest

from whiteboard_camera import (
    BoardConfig,
    RectificationError,
    WhiteboardInspector,
    camera_info_for_image,
    draw_camera_view,
    save_camera_info,
)
from whiteboard_camera.rectify import apply_homography, detect_markers, estimate_homography
from whiteboard_camera.regions import circles_cover, mask_to_circles
from whiteboard_camera.synthetic import (
    Smear,
    Stroke,
    demo_strokes,
    ground_truth_mask,
    look_at_camera,
    render_scene,
)

CFG = BoardConfig()
CAMERA = look_at_camera(CFG, tilt_x_deg=12.0, tilt_y_deg=-18.0)
GLARE = {"center": (700, 330), "sigma": (500.0, 35.0), "angle_deg": 70.0, "amplitude": 170.0}


def recall(mask, truth):
    return (mask & truth).sum() / truth.sum()


def dilate(mask, px):
    import cv2

    kernel = np.ones((2 * px + 1, 2 * px + 1), np.uint8)
    return cv2.dilate(mask.astype(np.uint8), kernel).astype(bool)


@pytest.mark.parametrize("y_axis_up", [False, True])
@pytest.mark.parametrize("tilt", [(0.0, 0.0), (12.0, -18.0), (-15.0, 25.0)])
def test_homography_matches_true_camera(y_axis_up, tilt):
    cfg = replace(CFG, y_axis_up=y_axis_up)
    camera = look_at_camera(cfg, tilt_x_deg=tilt[0], tilt_y_deg=tilt[1])
    image = render_scene(cfg, camera)
    detections = detect_markers(image, cfg)
    assert sorted(detections) == [0, 1, 2, 3]
    H, rms = estimate_homography(detections, cfg)
    assert rms < 0.5

    xs, ys = np.meshgrid(np.linspace(cfg.x_min, cfg.x_max, 7), np.linspace(cfg.y_min, cfg.y_max, 5))
    board_pts = np.column_stack([xs.ravel(), ys.ravel()])
    pixels = apply_homography(camera.board_to_image(cfg), board_pts)
    error_mm = np.linalg.norm(apply_homography(H, pixels) - board_pts, axis=1)
    assert error_mm.max() < 0.5, f"max position error {error_mm.max():.2f} mm"


def test_clean_board_reports_no_ink():
    for seed in range(5):
        result = WhiteboardInspector(CFG).inspect(render_scene(CFG, CAMERA, seed=seed))
        assert result.ink_present is False
        assert result.regions == []
        assert result.unobserved_fraction < 0.01


def test_drawing_is_found_and_covered():
    strokes = demo_strokes(CFG)
    truth = ground_truth_mask(CFG, strokes)
    result = WhiteboardInspector(CFG).inspect(render_scene(CFG, CAMERA, strokes))
    assert result.ink_present is True
    assert recall(result.ink_mask, truth) > 0.9
    # No ink reported far away from real ink (2 mm tolerance for blur).
    assert not (result.ink_mask & ~dilate(truth, int(2 * CFG.px_per_mm))).any()
    # The circles cover all the real ink.
    assert recall(circles_cover(truth, result.regions, CFG), truth) > 0.98


@pytest.mark.parametrize("color", ["red", "green", "blue", "black"])
def test_every_pen_colour_is_detected(color):
    stroke = Stroke([(60.0, 100.0), (240.0, 100.0)], color)
    truth = ground_truth_mask(CFG, [stroke])
    result = WhiteboardInspector(CFG).inspect(render_scene(CFG, CAMERA, [stroke]))
    assert recall(result.ink_mask, truth) > 0.9


def test_region_coordinates_are_in_board_millimetres():
    for y_axis_up in (False, True):
        cfg = replace(CFG, y_axis_up=y_axis_up)
        camera = look_at_camera(cfg, tilt_x_deg=12.0, tilt_y_deg=-18.0)
        dot = Stroke([(70.0, 50.0), (72.0, 50.0)], "black", width_mm=6.0)
        result = WhiteboardInspector(cfg).inspect(render_scene(cfg, camera, [dot]))
        assert len(result.regions) == 1
        x, y, r = result.regions[0]
        assert abs(x - 71.0) < 1.0 and abs(y - 50.0) < 1.0
        assert 3.0 < r < 6.5


def test_glare_is_not_ink_and_is_reported_unobserved():
    result = WhiteboardInspector(CFG).inspect(render_scene(CFG, CAMERA, glare=GLARE))
    assert result.ink_present is False
    assert result.unobserved_fraction > 0.01


def test_drawing_outside_glare_is_still_found():
    strokes = demo_strokes(CFG)
    truth = ground_truth_mask(CFG, strokes)
    result = WhiteboardInspector(CFG).inspect(render_scene(CFG, CAMERA, strokes, glare=GLARE))
    visible_truth = truth & ~dilate(result.unobserved_mask, 6)
    assert recall(result.ink_mask, visible_truth) > 0.85
    assert not (result.ink_mask & ~dilate(truth, int(2 * CFG.px_per_mm))).any()


def test_reference_image_finds_broad_faint_residue():
    smear = Smear((150.0, 100.0), 45.0, "red", alpha=0.3)
    truth = ground_truth_mask(CFG, smears=[smear])
    image = render_scene(CFG, CAMERA, smears=[smear], seed=1)

    blank = WhiteboardInspector(CFG)
    reference, *_ = blank.rectify(render_scene(CFG, CAMERA, seed=2))
    with_reference = WhiteboardInspector(CFG, reference).inspect(image)
    assert with_reference.ink_present is True
    assert recall(with_reference.ink_mask, truth) > 0.9

    # Documented limitation: without a reference, residue wider than the
    # background kernel is missed.
    without_reference = WhiteboardInspector(CFG).inspect(image)
    assert recall(without_reference.ink_mask, truth) < 0.5


def test_reference_image_cancels_static_glare():
    inspector = WhiteboardInspector(CFG)
    reference, *_ = inspector.rectify(render_scene(CFG, CAMERA, glare=GLARE, seed=2))
    strokes = demo_strokes(CFG)
    inspector = WhiteboardInspector(CFG, reference)
    assert inspector.inspect(render_scene(CFG, CAMERA, glare=GLARE, seed=3)).ink_present is False
    assert inspector.inspect(render_scene(CFG, CAMERA, strokes, glare=GLARE, seed=4)).ink_present is True


def test_hidden_markers():
    inspector = WhiteboardInspector(CFG)
    hidden = render_scene(CFG, CAMERA, demo_strokes(CFG), hide_markers=(1, 2))
    with pytest.raises(RectificationError):
        inspector.inspect(hidden)
    # After one good image at the same pose the homography is reused.
    assert inspector.inspect(render_scene(CFG, CAMERA)).homography_source == "markers"
    result = inspector.inspect(hidden)
    assert result.homography_source == "cached"
    assert result.ink_present is True
    # One hidden marker is tolerated outright.
    one_hidden = render_scene(CFG, CAMERA, hide_markers=(3,))
    assert WhiteboardInspector(CFG).inspect(one_hidden).homography_source == "markers"


def test_ignore_polygon_hides_the_gripper():
    stroke = Stroke([(60.0, 100.0), (240.0, 100.0)], "black")
    image = render_scene(CFG, CAMERA, [stroke])
    cfg = replace(CFG, image_ignore_polygons=(((0, 0), (1280, 0), (1280, 720), (0, 720)),))
    result = WhiteboardInspector(cfg).inspect(image)
    assert result.ink_present is False
    assert result.unobserved_fraction == 1.0


def test_circles_cover_every_ink_pixel():
    rng = np.random.default_rng(0)
    w, h = CFG.canvas_size
    for _ in range(20):
        mask = rng.random((h, w)) > 0.995
        circles = mask_to_circles(mask, CFG)
        assert (circles_cover(mask, circles, CFG) | ~mask).all()
        assert max(r for _, _, r in circles) <= CFG.cell_mm * np.sqrt(2) / 2 + 1.0
    assert mask_to_circles(np.zeros((h, w), bool), CFG) == []


def test_camera_view_is_drawn_with_and_without_a_located_board():
    inspector = WhiteboardInspector(CFG)
    for hidden in ((), (0, 1, 2, 3)):
        image = render_scene(CFG, CAMERA, hide_markers=hidden)
        view = draw_camera_view(image, inspector)
        assert view.shape[1] == image.shape[1] and view.shape[0] > image.shape[0]
    # The outline is drawn where the drawing area really is: its corner pixels change.
    image = render_scene(CFG, CAMERA)
    corner = apply_homography(CAMERA.board_to_image(CFG), [[CFG.x_min, CFG.y_min]])[0]
    u, v = int(round(corner[0])), int(round(corner[1])) + 28  # 28 px text bar on top
    view = draw_camera_view(image, inspector)
    assert tuple(view[v, u]) == (255, 200, 0)


def test_lens_data_is_stored_next_to_the_image(tmp_path):
    image_path = tmp_path / "photo.png"
    assert camera_info_for_image(image_path) == (None, None)
    save_camera_info(image_path.with_suffix(".json"), CAMERA.K, [0.1, -0.2, 0.0, 0.0, 0.05], 1280, 720)
    camera_matrix, dist_coeffs = camera_info_for_image(image_path)
    assert np.allclose(camera_matrix, CAMERA.K)
    assert np.allclose(dist_coeffs, [0.1, -0.2, 0.0, 0.0, 0.05])


def test_lens_distortion_is_corrected():
    import cv2

    # Distort a synthetic image the way a lens would, then check that the
    # pipeline with the lens data finds the board as well as without distortion.
    cfg = CFG
    image = render_scene(cfg, CAMERA, [Stroke([(70.0, 50.0), (72.0, 50.0)], "black", width_mm=6.0)])
    dist = np.array([-0.25, 0.08, 0.0, 0.0, 0.0])
    h, w = image.shape[:2]
    # For every pixel of the distorted image: where it comes from in the ideal one.
    grid = np.stack(np.meshgrid(np.arange(w), np.arange(h)), -1).reshape(-1, 1, 2).astype(np.float32)
    ideal = cv2.undistortPoints(grid, CAMERA.K, dist, P=CAMERA.K).reshape(h, w, 2)
    distorted = cv2.remap(image, ideal[..., 0], ideal[..., 1], cv2.INTER_LINEAR, borderValue=(95, 100, 105))

    corrected = WhiteboardInspector(cfg).inspect(distorted, CAMERA.K, dist)
    assert corrected.reprojection_rms_mm < 0.5
    x, y, _ = corrected.regions[0]
    assert abs(x - 71.0) < 1.0 and abs(y - 50.0) < 1.0
    uncorrected = WhiteboardInspector(cfg).inspect(distorted)
    assert uncorrected.reprojection_rms_mm > corrected.reprojection_rms_mm
