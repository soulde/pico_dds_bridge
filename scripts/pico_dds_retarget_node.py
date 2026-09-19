#!/usr/bin/env python3
"""Retarget pico DDS body tracking to Unitree G1 using the GMR submodule.

The runtime path is:

    pico_dds_bridge --coordinates xrobot
        -> pico/tracking DDS
        -> this node (GMR)
        -> robot/state DDS

GMR itself is loaded from ``third_party/GMR``.  The node only owns the DDS
adapter and the command-line wiring; retargeting, models, viewer, and IK
configuration come from that submodule.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
GMR_ROOT = REPO_ROOT / "third_party" / "GMR"
sys.path.insert(0, str(PYTHON_ROOT))
sys.path.insert(0, str(GMR_ROOT))

import mujoco as mj
from rich import print

from general_motion_retargeting import (
    GeneralMotionRetargeting,
    RobotMotionViewer,
)
from pico_tracking import RobotStatePublisher
from pico_tracking.gmr_source import PicoDdsGmrSource


_running = True


def _stop(signum, _frame) -> None:
    global _running
    print(f"\nReceived signal {signum}, shutting down...")
    _running = False


def main() -> None:
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    gmr_config = GMR_ROOT / "general_motion_retargeting" / "ik_configs" / "xrobot_soulde_to_g1.json"

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--tracking-topic", default="pico/tracking")
    parser.add_argument("--robot-topic", default="robot/state")
    parser.add_argument(
        "--input-coordinates",
        choices=["xrobot", "pico"],
        default="xrobot",
        help="coordinate convention published by the bridge",
    )
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands"],
        default="unitree_g1",
        help="GMR robot model",
    )
    parser.add_argument(
        "--ik-config",
        type=Path,
        default=gmr_config,
        help="GMR IK configuration",
    )
    parser.add_argument(
        "--mjcf",
        type=Path,
        default=None,
        help="optional override; otherwise use the model from GMR",
    )
    parser.add_argument("--human-height", type=float, default=None)
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--transparent-robot", type=float, default=0.5)
    args = parser.parse_args()

    if not args.ik_config.is_file():
        raise SystemExit(f"GMR IK config does not exist: {args.ik_config}")
    if args.mjcf is not None and not args.mjcf.is_file():
        raise SystemExit(f"MJCF does not exist: {args.mjcf}")

    print("[1/2] Connecting to pico_dds_bridge...")
    source = PicoDdsGmrSource(
        domain_id=args.domain,
        topic_name=args.tracking_topic,
        coordinate_system=args.input_coordinates,
    )
    if source.get_human_frame(timeout_s=5.0) is None:
        source.close()
        raise SystemExit("No frames on tracking topic; is the bridge running with body data?")

    print("[2/2] Initializing GMR...")
    retargeter = GeneralMotionRetargeting(
        src_human="xrobot",
        tgt_robot=args.robot,
        ik_config_path=args.ik_config,
        robot_xml_path=args.mjcf,
        actual_human_height=args.human_height,
        solver="daqp",
        damping=1.0,
        use_velocity_limit=True,
    )
    publisher = RobotStatePublisher(args.domain, args.robot_topic)
    viewer = None
    if args.visualize:
        viewer = RobotMotionViewer(
            robot_type=args.robot,
            camera_follow=True,
            motion_fps=60,
            transparent_robot=args.transparent_robot,
        )

    print(f"retargeting {args.tracking_topic} -> {args.robot_topic} for {args.robot}; Ctrl+C to stop")
    published = 0
    dropped = 0
    fps_counter = 0
    fps_start = time.monotonic()

    try:
        while _running:
            human_frame = source.get_human_frame(timeout_s=0.2)
            if human_frame is None or "Pelvis" not in human_frame:
                dropped += 1
                continue

            try:
                qpos = retargeter.retarget(human_frame, offset_to_ground=False)
            except Exception as error:
                print(f"Retargeting failed: {error}")
                dropped += 1
                continue

            now_ns = time.time_ns()
            publisher.publish_qpos(
                frame_seq=published,
                source_timestamp_ns=now_ns,
                receive_timestamp_ns=now_ns,
                qpos=qpos,
            )
            published += 1

            if viewer is not None:
                viewer.step(
                    qpos[:3],
                    qpos[3:7],
                    qpos[7:],
                    human_motion_data=retargeter.scaled_human_data,
                    rate_limit=False,
                    follow_camera=True,
                )

            fps_counter += 1
            elapsed = time.monotonic() - fps_start
            if elapsed >= 2.0:
                print(
                    f"FPS: {fps_counter / elapsed:.1f} | "
                    f"Published: {published} | Dropped: {dropped}"
                )
                fps_counter = 0
                fps_start = time.monotonic()
    finally:
        source.close()
        if viewer is not None:
            viewer.close()
        print(f"published {published} robot states in total")


if __name__ == "__main__":
    main()
