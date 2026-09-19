"""Convert pico DDS body frames into the human-pose format consumed by GMR.

The bridge should normally be started with ``--coordinates xrobot``.  In that
mode this adapter passes positions and orientations through unchanged, which
matches GMR's ``XRobotStreamer`` convention.  ``pico`` is retained as a
fallback for recordings or a bridge configured without coordinate conversion.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from .client import PicoTrackingClient


BODY_JOINT_NAMES = [
    "Pelvis", "Left_Hip", "Right_Hip", "Spine1", "Left_Knee", "Right_Knee",
    "Spine2", "Left_Ankle", "Right_Ankle", "Spine3", "Left_Foot", "Right_Foot",
    "Neck", "Left_Collar", "Right_Collar", "Head", "Left_Shoulder", "Right_Shoulder",
    "Left_Elbow", "Right_Elbow", "Left_Wrist", "Right_Wrist", "Left_Hand", "Right_Hand",
]

_PICO_TO_XROBOT = R.from_quat(
    [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]
)


def _convert_position(position: np.ndarray, coordinate_system: str) -> np.ndarray:
    if coordinate_system == "xrobot":
        return position
    if coordinate_system == "pico":
        # Same basis as pico_dds_bridge --coordinates xrobot: (x, y, z) ->
        # (x, -z, y).
        return np.asarray([position[0], -position[2], position[1]])
    raise ValueError(
        "coordinate_system must be xrobot or pico; robot coordinates are not "
        "a GMR/XRobot input convention"
    )


def _convert_orientation(quat_xyzw: np.ndarray, coordinate_system: str) -> np.ndarray:
    quat_wxyz = np.asarray(
        [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
        dtype=float,
    )
    if coordinate_system == "xrobot":
        # The bridge has already applied the XRobot basis change. Preserve
        # the SDK quaternion exactly; GMR's quat_mul_np also does not
        # normalize its input.
        return quat_wxyz
    elif coordinate_system == "pico":
        # XRobotStreamer uses q_out = q_basis * q_input, not a passive
        # basis conjugation. Keep this identical to the C++ bridge.
        basis = _PICO_TO_XROBOT.as_quat(scalar_first=True)
        w0, x0, y0, z0 = basis
        w1, x1, y1, z1 = quat_wxyz
        return np.array([
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        ])
    else:
        raise ValueError(
            "coordinate_system must be xrobot or pico; robot coordinates are not "
            "a GMR/XRobot input convention"
        )


class PicoDdsGmrSource:
    """DDS subscriber yielding ``{joint: [position, quat_wxyz]}`` frames."""

    def __init__(
        self,
        domain_id: int = 0,
        topic_name: str = "pico/tracking",
        coordinate_system: str = "xrobot",
        timeout_s: float = 1.0,
    ) -> None:
        if coordinate_system not in {"xrobot", "pico"}:
            raise ValueError(
                "coordinate_system must be xrobot or pico; "
                "robot coordinates are not a GMR/XRobot input convention"
            )
        self._client = PicoTrackingClient(domain_id, topic_name)
        self._coordinate_system = coordinate_system
        self._timeout_s = timeout_s

    def close(self) -> None:
        self._client = None  # type: ignore[assignment]

    def get_human_frame(self, timeout_s: Optional[float] = None) -> Optional[dict]:
        frame = self._client.wait_next(timeout_s=timeout_s or self._timeout_s)
        return None if frame is None else self.frame_to_human_dict(frame)

    def frames(self):
        while True:
            frame = self._client.wait_next(timeout_s=self._timeout_s)
            if frame is not None:
                yield self.frame_to_human_dict(frame)

    def frame_to_human_dict(self, frame) -> dict:
        human: dict[str, list] = {}
        count = min(int(frame.body_count), len(frame.body))

        foot_up = []
        for i in (10, 11):
            if i >= count or not frame.body[i].tracking.valid:
                continue
            foot = np.array([
                frame.body[i].tracking.pose.position.x,
                frame.body[i].tracking.pose.position.y,
                frame.body[i].tracking.pose.position.z,
            ], dtype=float)
            foot_up.append(_convert_position(foot, self._coordinate_system)[2])
        ground_up = min(foot_up) + 0.05 if foot_up else 0.0

        for index in range(count):
            state = frame.body[index].tracking
            if not state.valid:
                continue

            position = np.array([
                state.pose.position.x,
                state.pose.position.y,
                state.pose.position.z,
            ], dtype=float)
            position = _convert_position(position, self._coordinate_system)
            position[2] -= ground_up

            quat_xyzw = np.array([
                state.pose.orientation.x,
                state.pose.orientation.y,
                state.pose.orientation.z,
                state.pose.orientation.w,
            ], dtype=float)
            orientation = _convert_orientation(quat_xyzw, self._coordinate_system)
            human[BODY_JOINT_NAMES[index]] = [position.tolist(), orientation.tolist()]

        return human


if __name__ == "__main__":
    source = PicoDdsGmrSource()
    print("waiting for pico/tracking frames...")
    deadline = time.monotonic() + 5.0
    got = 0
    while time.monotonic() < deadline and got < 3:
        frame = source.get_human_frame(timeout_s=1.0)
        if frame is None:
            continue
        got += 1
        print(f"joints={len(frame)} pelvis={np.round(frame['Pelvis'][0], 3)}")
    source.close()
