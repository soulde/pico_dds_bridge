"""DDS robot/state -> GEAR-SONIC deploy ZMQ 'pose' stream.

Bridges the retargeted G1 qpos published on the DDS RobotState topic into the
packed ZMQ motion stream consumed by gear_sonic_deploy's
g1_deploy_onnx_ref node (--input-type zmq):

    pico/tracking -> retarget node -> robot/state (DDS) -> this node
                                                        -> ZMQ 'pose' topic
                                                        -> sonic tracker -> robot control

Joint values are reordered by name between the GMR G1 MJCF and the SONIC
deploy model (g1_29dof.xml).

The wire-format packers are imported from the gear_sonic checkout.
"""

import argparse
import importlib.util
import time
from collections import deque
from pathlib import Path

import mujoco
import numpy as np
import zmq

from pico_tracking import DEFAULT_ROBOT_TOPIC, RobotStateSubscriber

GEAR_SONIC_ROOT = Path("/home/jvwei/GR00T-WholeBodyControl")
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE_XML = REPO_ROOT / "third_party" / "GMR" / "assets" / "unitree_g1" / "g1_mocap_29dof.xml"


def load_gear_sonic_packers():
    module_path = GEAR_SONIC_ROOT / "gear_sonic" / "utils" / "teleop" / "zmq" / "zmq_planner_sender.py"
    spec = importlib.util.spec_from_file_location("zmq_planner_sender", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.pack_pose_message, module.build_command_message


def hinge_joint_names(xml_path):
    model = mujoco.MjModel.from_xml_path(xml_path)
    return [
        model.joint(i).name
        for i in range(model.njnt)
        if model.joint(i).type == mujoco.mjtJoint.mjJNT_HINGE
    ]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--robot-topic", default=DEFAULT_ROBOT_TOPIC)
    parser.add_argument("--zmq-host", default="*")
    parser.add_argument("--zmq-port", type=int, default=5556)
    parser.add_argument("--window", type=int, default=5, help="sliding window frames per message")
    parser.add_argument(
        "--source-xml",
        default=str(DEFAULT_SOURCE_XML),
        help="model the retargeted qpos is ordered by",
    )
    parser.add_argument(
        "--target-xml",
        default="/home/jvwei/GR00T-WholeBodyControl/gear_sonic_deploy/g1/g1_29dof.xml",
        help="SONIC deploy model whose joint order to emit",
    )
    parser.add_argument(
        "--send-start",
        action="store_true",
        help="also publish a start command (streamed-motion mode) on connect",
    )
    args = parser.parse_args()

    pack_pose_message, build_command_message = load_gear_sonic_packers()

    source_names = hinge_joint_names(args.source_xml)
    target_names = hinge_joint_names(args.target_xml)
    reorder = [source_names.index(name) for name in target_names]
    left_hand_idx = [i for i, n in enumerate(source_names) if n.startswith("left_hand")]
    right_hand_idx = [i for i, n in enumerate(source_names) if n.startswith("right_hand")]
    print(f"source {len(source_names)} joints -> sonic {len(target_names)} joints")

    subscriber = RobotStateSubscriber(args.domain, args.robot_topic)

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://{args.zmq_host}:{args.zmq_port}")

    if args.send_start:
        socket.send(build_command_message(start=True, planner=False))
        print("sent start command (streamed-motion mode)")

    window = deque(maxlen=args.window)
    frame_index = 0
    last_stats = time.monotonic()
    stats_msgs = 0

    print(f"bridging DDS {args.robot_topic} -> ZMQ tcp://{args.zmq_host}:{args.zmq_port} 'pose'")
    try:
        while True:
            sample = subscriber.wait_next(timeout_s=1.0)
            if sample is None:
                continue
            qpos = np.asarray(
                list(sample.root_position)
                + list(sample.root_orientation_quat_wxyz)
                + list(sample.joint_position),
                dtype=np.float64,
            )
            joints = qpos[7 : 7 + len(source_names)]
            window.append(joints)
            frame_index += 1

            if len(window) < args.window:
                continue

            joint_pos = np.stack(list(window))[:, reorder].astype(np.float32)
            body_quat = np.tile(
                np.asarray(sample.root_orientation_quat_wxyz, dtype=np.float32),
                (args.window, 1),
            )
            latest = list(window)[-1]
            data = {
                "joint_pos": joint_pos,
                "joint_vel": np.zeros_like(joint_pos),
                "body_quat_w": body_quat,
                "frame_index": np.arange(
                    frame_index - args.window, frame_index, dtype=np.int64
                ),
            }
            if left_hand_idx:
                data["left_hand_joints"] = np.asarray(
                    [latest[i] for i in left_hand_idx], dtype=np.float32
                )
            if right_hand_idx:
                data["right_hand_joints"] = np.asarray(
                    [latest[i] for i in right_hand_idx], dtype=np.float32
                )

            socket.send(pack_pose_message(data, topic="pose"))
            stats_msgs += 1

            now = time.monotonic()
            if now - last_stats >= 5.0:
                print(f"[bridge] msgs={stats_msgs} hz={stats_msgs / (now - last_stats):.1f}")
                stats_msgs = 0
                last_stats = now
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.close()
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
