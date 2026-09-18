from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import (
    array,
    float32,
    int32,
    int64,
    uint32,
    uint64,
)


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
