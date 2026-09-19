"""MuJoCo playback node for retargeted robot state over DDS.

Subscribes to the RobotState topic published by the retargeting node and
replays qpos on a MuJoCo model. This node only depends on the DDS contract
and MuJoCo -- it has no dependency on the retargeting stack.

    python robot_motion_player.py --xml <model.xml> [--domain 0] [--topic robot/state]
"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from pico_tracking import DEFAULT_ROBOT_TOPIC, RobotStateSubscriber

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML = REPO_ROOT / "third_party" / "GMR" / "assets" / "unitree_g1" / "g1_mocap_29dof.xml"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="MuJoCo DDS robot motion player",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--xml", default=DEFAULT_XML, help="MuJoCo model to animate")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--topic", default=DEFAULT_ROBOT_TOPIC)
    parser.add_argument("--rate", type=float, default=60.0, help="viewer update rate")
    parser.add_argument(
        "--follow",
        action="store_true",
        help="keep camera centered on the robot base",
    )
    parser.add_argument(
        "--record",
        default=None,
        help="append received qpos to this .npy file on exit",
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    if model.nq < 7:
        raise SystemExit("model has no free joint root")

    subscriber = RobotStateSubscriber(args.domain, args.topic)
    base_body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, 0) or "base"

    period = 1.0 / max(args.rate, 1.0)
    qpos = np.zeros(model.nq)
    have_frame = False
    received = 0
    recorded: list[np.ndarray] = []
    last_stats = time.monotonic()
    stats_frames = 0

    print(f"listening robot state on domain={args.domain} topic={args.topic}; model={args.xml}")
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                sample = subscriber.take_latest()
                if sample is not None:
                    qpos_arr = np.asarray(
                        list(sample.root_position)
                        + list(sample.root_orientation_quat_wxyz)
                        + list(sample.joint_position),
                        dtype=float,
                    )
                    n = min(len(qpos_arr), model.nq)
                    qpos[:n] = qpos_arr[:n]
                    have_frame = True
                    received += 1
                    stats_frames += 1
                    if args.record:
                        recorded.append(qpos.copy())

                if have_frame:
                    data.qpos[:] = qpos
                    mujoco.mj_forward(model, data)

                if args.follow:
                    viewer.cam.lookat = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base_body)]
                    viewer.cam.distance = 2.5
                    viewer.cam.elevation = -10

                viewer.sync()

                now = time.monotonic()
                if now - last_stats >= 5.0:
                    hz = stats_frames / (now - last_stats)
                    print(f"[player] state_hz={hz:.1f} total={received}")
                    stats_frames = 0
                    last_stats = now
                time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.close()
        if args.record and recorded:
            path = args.record
            existing = []
            try:
                existing = list(np.load(path))
            except Exception:
                pass
            np.save(path, np.stack(existing + recorded))
            print(f"saved {len(recorded)} samples to {path}")


if __name__ == "__main__":
    main()
