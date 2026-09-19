"""DDS contract for retargeted robot state.

A retargeting node consumes ``pico/tracking`` human frames and publishes
``RobotState`` samples; playback/recording nodes subscribe and replay them.
The payload is robot-agnostic: qpos follows the MuJoCo convention
(root position xyz, root quaternion wxyz, then joint positions in model
order).
"""

from dataclasses import dataclass

from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import float64, int64, sequence, uint64
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

DEFAULT_ROBOT_TOPIC = "robot/state"


@dataclass
class RobotState(IdlStruct, typename="pico_dds::RobotState"):
    frame_seq: uint64
    source_timestamp_ns: int64
    receive_timestamp_ns: int64
    root_position: sequence[float64]
    root_orientation_quat_wxyz: sequence[float64]
    joint_position: sequence[float64]


class RobotStatePublisher:
    def __init__(self, domain_id: int = 0, topic_name: str = DEFAULT_ROBOT_TOPIC) -> None:
        self.participant = DomainParticipant(domain_id)
        self.topic = Topic(self.participant, topic_name, RobotState)
        self.writer = DataWriter(self.participant, self.topic)

    @staticmethod
    def sample_from_qpos(
        frame_seq: int,
        source_timestamp_ns: int,
        receive_timestamp_ns: int,
        qpos,
    ) -> RobotState:
        """Build a RobotState from a MuJoCo-style qpos vector."""
        qpos = list(float(v) for v in qpos)
        return RobotState(
            frame_seq=frame_seq,
            source_timestamp_ns=source_timestamp_ns,
            receive_timestamp_ns=receive_timestamp_ns,
            root_position=qpos[0:3],
            root_orientation_quat_wxyz=qpos[3:7],
            joint_position=qpos[7:],
        )

    def publish_qpos(
        self,
        frame_seq: int,
        source_timestamp_ns: int,
        receive_timestamp_ns: int,
        qpos,
    ) -> None:
        self.writer.write(
            self.sample_from_qpos(frame_seq, source_timestamp_ns, receive_timestamp_ns, qpos)
        )


class RobotStateSubscriber:
    def __init__(self, domain_id: int = 0, topic_name: str = DEFAULT_ROBOT_TOPIC) -> None:
        self.participant = DomainParticipant(domain_id)
        self.topic = Topic(self.participant, topic_name, RobotState)
        self.reader = DataReader(self.participant, self.topic)

    def take_latest(self, max_samples: int = 64) -> RobotState | None:
        samples = [
            s for s in self.reader.take(N=max_samples) if type(s) is RobotState
        ]
        return samples[-1] if samples else None

    def wait_next(self, timeout_s: float | None = None) -> RobotState | None:
        import time as _time

        deadline = None if timeout_s is None else _time.monotonic() + timeout_s
        while True:
            sample = self.take_latest()
            if sample is not None:
                return sample
            if deadline is not None and _time.monotonic() >= deadline:
                return None
            _time.sleep(0.001)

    def samples(self) -> "list[RobotState]":
        return [s for s in self.reader.take(N=64) if type(s) is RobotState]

    def close(self) -> None:
        self.reader = None  # type: ignore[assignment]
        self.topic = None  # type: ignore[assignment]
        self.participant = None  # type: ignore[assignment]
