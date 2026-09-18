import argparse
import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable

import mujoco
import mujoco.viewer
import numpy as np
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import array, float32, int32, int64, uint32, uint64


@dataclass
class Vec3(IdlStruct, typename="pico_dds::Vec3"):
    x: float
    y: float
    z: float


@dataclass
class Quaternion(IdlStruct, typename="pico_dds::Quaternion"):
    x: float
    y: float
    z: float
    w: float


@dataclass
class Pose(IdlStruct, typename="pico_dds::Pose"):
    position: Vec3
    orientation: Quaternion


@dataclass
class TrackingState(IdlStruct, typename="pico_dds::TrackingState"):
    valid: bool
    status: uint32
    timestamp_ns: int64
    pose: Pose
    linear_velocity: Vec3
    angular_velocity: Vec3
    linear_acceleration: Vec3
    angular_acceleration: Vec3


@dataclass
class ControllerState(IdlStruct, typename="pico_dds::ControllerState"):
    tracking: TrackingState
    axis_x: float32
    axis_y: float32
    grip: float32
    trigger: float32
    axis_click: bool
    primary_button: bool
    secondary_button: bool
    menu_button: bool


@dataclass
class HandJoint(IdlStruct, typename="pico_dds::HandJoint"):
    tracking: TrackingState
    radius: float32


@dataclass
class HandState(IdlStruct, typename="pico_dds::HandState"):
    active: bool
    count: uint32
    scale: float32
    timestamp_ns: int64
    joints: array[HandJoint, 26]


@dataclass
class BodyJoint(IdlStruct, typename="pico_dds::BodyJoint"):
    role: uint32
    tracking: TrackingState
    imu_timestamp_ns: int64


@dataclass
class MotionTracker(IdlStruct, typename="pico_dds::MotionTracker"):
    index: uint32
    tracking: TrackingState


@dataclass
class TrackingFrame(IdlStruct, typename="pico_dds::TrackingFrame"):
    frame_seq: uint64
    source_timestamp_ns: int64
    receive_timestamp_ns: int64
    input_mode: int32
    head: TrackingState
    left_controller: ControllerState
    right_controller: ControllerState
    left_hand: HandState
    right_hand: HandState
    body_count: uint32
    body: array[BodyJoint, 24]
    tracker_count: uint32
    trackers: array[MotionTracker, 5]


class TrackingScene:
    def __init__(self, max_points: int = 90, max_segments: int = 90) -> None:
        self.model = mujoco.MjModel.from_xml_string(self._xml(max_points, max_segments))
        self.data = mujoco.MjData(self.model)
        self.point_geoms = [
            (self.model.body(f"point_{index}").id, self.model.geom(f"point_geom_{index}").id)
            for index in range(max_points)
        ]
        self.segment_geoms = [
            self.model.geom(f"segment_geom_{index}").id
            for index in range(max_segments)
        ]
        self.max_segments = max_segments
        self._point_index = 0
        self._segment_index = 0

    @staticmethod
    def _xml(max_points: int, max_segments: int) -> str:
        point_bodies = "".join(
            f'<body name="point_{index}" pos="0 0 0">'
            f'<geom name="point_geom_{index}" type="sphere" size="0.025" rgba="0.95 0.35 0.12 1" contype="0" conaffinity="0"/>'
            "</body>"
            for index in range(max_points)
        )
        segment_geoms = "".join(
            f'<geom name="segment_geom_{index}" type="capsule" fromto="0 0 0 0.001 0 0" size="0.009" rgba="0.95 0.72 0.18 0" contype="0" conaffinity="0"/>'
            for index in range(max_segments)
        )
        return f"""
<mujoco model="pico_tracking">
  <option gravity="0 0 0" />
    <visual>
        <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0.9 0.9 0.9" />
        <rgba haze="0.15 0.25 0.35 1" />
        <global azimuth="140" elevation="-20" />
        <map znear="0.01" zfar="30" shadowclip="2" />
    </visual>
    <asset>
        <texture name="skybox" type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072" />
        <texture name="groundplane" type="2d" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300" />
        <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2" />
    </asset>
  <worldbody>
        <light name="key_light" pos="2 -3 5" dir="-0.25 0.35 -1" directional="true" diffuse="0.9 0.88 0.82" specular="0.35 0.35 0.35" castshadow="true" />
        <light name="fill_light" pos="-3 2 2.5" dir="0.35 -0.2 -1" directional="true" diffuse="0.35 0.45 0.65" specular="0.1 0.1 0.1" castshadow="false" />
        <geom name="floor" type="plane" size="0 0 0.05" material="groundplane" />
    <site name="axis_origin" pos="0 0 0" size="0.035" rgba="1 1 1 1" />
    <geom name="axis_x" type="capsule" fromto="0 0 0 0.7 0 0" size="0.012" rgba="0.9 0.12 0.12 1" contype="0" conaffinity="0" />
    <geom name="axis_y" type="capsule" fromto="0 0 0 0 0.7 0" size="0.012" rgba="0.12 0.85 0.25 1" contype="0" conaffinity="0" />
    <geom name="axis_z" type="capsule" fromto="0 0 0 0 0 0.7" size="0.012" rgba="0.18 0.4 1 1" contype="0" conaffinity="0" />
    {point_bodies}
    {segment_geoms}
  </worldbody>
</mujoco>
"""

    def reset_frame(self) -> None:
        self._point_index = 0
        self._segment_index = 0
        for _, geom_id in self.point_geoms:
            self.model.geom_rgba[geom_id, 3] = 0.0
        for geom_id in self.segment_geoms:
            self.model.geom_rgba[geom_id, 3] = 0.0

    def add_point(self, position: Vec3, color: tuple[float, float, float]) -> None:
        if self._point_index >= len(self.point_geoms):
            return
        body_id, geom_id = self.point_geoms[self._point_index]
        self.model.body_pos[body_id] = (position.x, position.y, position.z)
        self.model.geom_rgba[geom_id] = (*color, 1.0)
        self._point_index += 1

    def add_chain(
        self,
        positions: Iterable[Vec3],
        color: tuple[float, float, float],
    ) -> None:
        previous: Vec3 | None = None
        for position in positions:
            self.add_point(position, color)
            if previous is not None:
                self.add_segment(previous, position, color)
            previous = position

    def add_segment(
        self,
        start: Vec3,
        end: Vec3,
        color: tuple[float, float, float],
    ) -> None:
        if self._segment_index >= self.max_segments:
            return
        geom_id = self.segment_geoms[self._segment_index]
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-6:
            return

        self.model.geom_pos[geom_id] = (
            (start.x + end.x) * 0.5,
            (start.y + end.y) * 0.5,
            (start.z + end.z) * 0.5,
        )
        self.model.geom_size[geom_id, 1] = length * 0.5
        quaternion = np.zeros(4)
        mujoco.mju_quatZ2Vec(
            quaternion,
            np.array([dx / length, dy / length, dz / length]),
        )
        self.model.geom_quat[geom_id] = quaternion
        self.model.geom_rgba[geom_id] = (*color, 1.0)
        self._segment_index += 1

    def show_default_skeleton(self) -> None:
        self.reset_frame()
        color = (0.58, 0.62, 0.7)
        self.add_chain(
            [Vec3(0.0, 0.0, 0.95), Vec3(0.0, 0.0, 1.3), Vec3(0.0, 0.0, 1.62)],
            color,
        )
        self.add_chain(
            [Vec3(0.0, 0.0, 1.48), Vec3(-0.22, 0.0, 1.38), Vec3(-0.42, 0.0, 1.12)],
            color,
        )
        self.add_chain(
            [Vec3(0.0, 0.0, 1.48), Vec3(0.22, 0.0, 1.38), Vec3(0.42, 0.0, 1.12)],
            color,
        )
        self.add_chain(
            [Vec3(0.0, 0.0, 0.95), Vec3(-0.16, 0.0, 0.52), Vec3(-0.18, 0.0, 0.08)],
            color,
        )
        self.add_chain(
            [Vec3(0.0, 0.0, 0.95), Vec3(0.16, 0.0, 0.52), Vec3(0.18, 0.0, 0.08)],
            color,
        )

    def update(self, frame: TrackingFrame) -> None:
        self.reset_frame()

        # PICO's tracking origin sits at its anchor, not on the floor: rebase
        # the frame so the lowest foot joint defines ground level.
        ground_z = 0.0
        body_count = min(int(frame.body_count), len(frame.body))
        if body_count > min(FOOT_JOINTS):
            ground_z = min(
                frame.body[i].tracking.pose.position.z
                for i in FOOT_JOINTS
                if frame.body[i].tracking.valid
            ) if any(frame.body[i].tracking.valid for i in FOOT_JOINTS) else 0.0

        def grounded(position: Vec3) -> Vec3:
            return Vec3(position.x, position.y, position.z - ground_z)

        if frame.head.valid:
            self.add_point(grounded(frame.head.pose.position), (1.0, 0.85, 0.15))
        if frame.left_controller.tracking.valid:
            self.add_point(grounded(frame.left_controller.tracking.pose.position), (0.2, 0.55, 1.0))
        if frame.right_controller.tracking.valid:
            self.add_point(grounded(frame.right_controller.tracking.pose.position), (1.0, 0.3, 0.3))

        for hand, color in (
            (frame.left_hand, (0.25, 0.75, 1.0)),
            (frame.right_hand, (1.0, 0.4, 0.45)),
        ):
            count = min(int(hand.count), len(hand.joints))
            positions = [
                grounded(joint.tracking.pose.position)
                for joint in hand.joints[:count]
                if joint.tracking.valid
            ]
            self.add_chain(positions, color)

        positions = [
            grounded(joint.tracking.pose.position) if joint.tracking.valid else None
            for joint in frame.body[:body_count]
        ]
        for a, b in BODY_BONES:
            if a < len(positions) and b < len(positions) and positions[a] is not None and positions[b] is not None:
                self.add_segment(positions[a], positions[b], (0.95, 0.55, 0.18))
        for index, position in enumerate(positions):
            if position is not None and not any(index in bone for bone in BODY_BONES):
                self.add_point(position, (0.95, 0.55, 0.18))

        tracker_count = min(int(frame.tracker_count), len(frame.trackers))
        tracker_positions = [
            tracker.tracking.pose.position
            for tracker in frame.trackers[:tracker_count]
            if tracker.tracking.valid
        ]
        self.add_chain(tracker_positions, (0.65, 0.3, 1.0))


# PICO body-tracking joint indices (identified empirically; within each
# left/right pair the lower index is the LEFT side):
#   spine: 0 hips, 3 waist, 6 chest, 9 upper chest, 12 neck, 15 head
#   clavicles: 13 L / 14 R; shoulders: 16 L / 17 R; elbows: 18 L / 19 R;
#   wrists: 20 L / 21 R; hands: 22 L / 23 R
#   hips L/R: 1 L / 2 R; knees: 4 L / 5 R; ankles: 7 L / 8 R; feet: 10 L / 11 R
BODY_BONES: list[tuple[int, int]] = [
    (0, 3), (3, 6), (6, 9), (9, 12), (12, 15),          # spine + head
    (12, 13), (13, 16), (16, 18), (18, 20), (20, 22),   # left arm
    (12, 14), (14, 17), (17, 19), (19, 21), (21, 23),   # right arm
    (0, 1), (1, 4), (4, 7), (7, 10),                    # left leg
    (0, 2), (2, 5), (5, 8), (8, 11),                    # right leg
]
FOOT_JOINTS = (10, 11)


class DdsSubscriber:
    def __init__(self, domain_id: int, topic_name: str) -> None:
        self._participant = DomainParticipant(domain_id)
        self._topic = Topic(self._participant, topic_name, TrackingFrame)
        self._reader = DataReader(self._participant, self._topic)
        self._lock = threading.Lock()
        self._latest: TrackingFrame | None = None

    def take_latest(self) -> TrackingFrame | None:
        samples = self._reader.take(N=64)
        # instance disposes (e.g. a restarted bridge) surface as InvalidSample
        samples = [s for s in samples if type(s) is TrackingFrame]
        return samples[-1] if samples else None

    def close(self) -> None:
        self._reader = None  # type: ignore[assignment]
        self._topic = None  # type: ignore[assignment]
        self._participant = None  # type: ignore[assignment]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View pico_dds_bridge tracking in MuJoCo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--topic", default="pico/tracking")
    parser.add_argument("--rate", type=float, default=120.0, help="viewer update rate")
    parser.add_argument(
        "--show-default-skeleton",
        action="store_true",
        help="show a neutral skeleton until the first DDS frame arrives",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    subscriber = DdsSubscriber(args.domain, args.topic)
    scene = TrackingScene()
    if args.show_default_skeleton:
        scene.show_default_skeleton()
    last_frame: TrackingFrame | None = None
    period = 1.0 / max(args.rate, 1.0)

    try:
        with mujoco.viewer.launch_passive(scene.model, scene.data) as viewer:
            while viewer.is_running():
                frame = subscriber.take_latest()
                if frame is not None:
                    last_frame = frame
                if last_frame is not None:
                    scene.update(last_frame)
                    mujoco.mj_forward(scene.model, scene.data)
                viewer.sync()
                time.sleep(period)
    finally:
        subscriber.close()


if __name__ == "__main__":
    main()
