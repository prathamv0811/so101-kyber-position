"""Invoke the official LeRobot interactive calibration CLI for either arm."""
import argparse
import os
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("leader", "follower"))
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", required=True, help="Stable arm ID, also used as calibration filename")
    parser.add_argument("--directory", type=Path, default=Path("calibration"))
    parser.add_argument("--attended", action="store_true", help="Arm supported; local power cutoff available")
    args = parser.parse_args()
    if not args.attended:
        parser.error("Interactive calibration requires --attended; torque may be disabled")
    if Path(args.id).name != args.id or args.id in (".", ".."):
        parser.error("id must be a filename-safe arm name")
    args.directory.mkdir(parents=True, exist_ok=True)
    prefix = "teleop" if args.role == "leader" else "robot"
    print("Starting official LeRobot calibration. Follow its pose/range prompts.", file=sys.stderr)
    os.execv(sys.executable, [sys.executable, "-m", "lerobot.scripts.lerobot_calibrate",
        f"--{prefix}.type=so101_{args.role}", f"--{prefix}.port={args.port}",
        f"--{prefix}.id={args.id}", f"--{prefix}.calibration_dir={args.directory.resolve()}"])


if __name__ == "__main__":
    main()
