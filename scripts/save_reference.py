"""Save the clean-board reference image from a photo of the CLEAN whiteboard.

The photo must be taken at the same camera pose and lighting that will be used
for inspection. The reference is stored rectified, so it has to be re-created
whenever the board geometry in the config changes.

Usage:  python scripts/save_reference.py clean_board.png [--out data/reference.png]
"""
import argparse
import sys
from pathlib import Path

import cv2

from whiteboard_camera import RectificationError, WhiteboardInspector, camera_info_for_image

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image")
    parser.add_argument("--config", default=ROOT / "config" / "board.yaml")
    parser.add_argument("--out", default=ROOT / "data" / "reference.png")
    args = parser.parse_args()

    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        sys.exit(f"cannot read image: {args.image}")
    inspector = WhiteboardInspector.from_files(args.config)
    try:
        # Check the photo really is clean before trusting it as the reference.
        result = inspector.inspect(image, *camera_info_for_image(args.image))
    except RectificationError as error:
        sys.exit(f"whiteboard not found: {error}")
    if result.ink_present:
        print(
            f"warning: {result.ink_area_mm2:.0f} mm2 of ink detected in this photo. "
            "A reference with ink on it hides that ink later.",
            file=sys.stderr,
        )
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out), result.rectified)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
