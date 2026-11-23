"""Read the current pose of the Franka arm. Read-only: sends no motion command.

Prints the 7 joint angles and the end-effector pose, and optionally saves them
under a name so the pose can be used later (e.g. the camera's photo pose).

Requires: network connection to the robot, and FCI activated in Desk
(top-right arrow menu -> Activate FCI).

Usage:
  python read_pose.py                      # print the current pose
  python read_pose.py photo_pose           # also save it to poses/photo_pose.json
  python read_pose.py photo_pose --ip 172.16.0.10
"""
import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pylibfranka

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("name", nargs="?", help="save the pose under this name")
    parser.add_argument("--ip", default="172.16.0.10")
    args = parser.parse_args()

    try:
        # kIgnore: reading the state does not need a real-time kernel.
        robot = pylibfranka.Robot(args.ip, pylibfranka.RealtimeConfig.kIgnore)
        state = robot.read_once()
    except Exception as error:  # network, version or FCI problems
        sys.exit(
            f"could not read the robot at {args.ip}: {error}\n"
            "Check: cable and IP (ping), FCI activated in Desk, no other program connected to FCI."
        )

    q = np.array(state.q)                                  # joint angles, rad
    T = np.array(state.O_T_EE).reshape(4, 4, order="F")    # base -> end effector, column-major
    position_mm = T[:3, 3] * 1000.0

    np.set_printoptions(precision=4, suppress=True)
    print("robot mode:", state.robot_mode)
    print("joint angles q (rad):", q)
    print("joint angles q (deg):", np.degrees(q).round(2))
    print("end effector position in the base frame (mm): x %.1f  y %.1f  z %.1f" % tuple(position_mm))
    print("end effector pose O_T_EE (4x4, metres):")
    print(T)

    if args.name:
        folder = HERE / "poses"
        folder.mkdir(exist_ok=True)
        path = folder / f"{args.name}.json"
        path.write_text(
            json.dumps(
                {
                    "name": args.name,
                    "saved": datetime.now().isoformat(timespec="seconds"),
                    "q_rad": q.tolist(),
                    "O_T_EE": T.tolist(),
                },
                indent=2,
            )
        )
        print("saved", path)


if __name__ == "__main__":
    main()
