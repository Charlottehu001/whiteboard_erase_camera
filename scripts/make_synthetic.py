"""Generate synthetic test images of the whiteboard into data/synthetic/.

Usage:  python scripts/make_synthetic.py [--config config/board.yaml]
"""
import argparse
from pathlib import Path

import cv2

from whiteboard_camera import load_config
from whiteboard_camera.synthetic import Smear, demo_strokes, look_at_camera, render_scene

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=ROOT / "config" / "board.yaml")
    parser.add_argument("--out", default=ROOT / "data" / "synthetic")
    args = parser.parse_args()

    cfg = load_config(args.config)
    camera = look_at_camera(cfg, tilt_x_deg=12.0, tilt_y_deg=-18.0)
    strokes = demo_strokes(cfg)
    centre = ((cfg.x_min + cfg.x_max) / 2, (cfg.y_min + cfg.y_max) / 2)
    glare = {"center": (700, 330), "sigma": (500.0, 35.0), "angle_deg": 70.0, "amplitude": 170.0}
    scenes = {
        "blank": dict(),
        "drawing": dict(strokes=strokes),
        "drawing_glare": dict(strokes=strokes, glare=glare),
        "blank_glare": dict(glare=glare),
        "residue": dict(smears=[Smear(centre, 45.0, "red", 0.3)]),
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for seed, (name, kwargs) in enumerate(scenes.items()):
        path = out / f"{name}.png"
        cv2.imwrite(str(path), render_scene(cfg, camera, seed=seed, **kwargs))
        print("wrote", path.relative_to(ROOT))


if __name__ == "__main__":
    main()
