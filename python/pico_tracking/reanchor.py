"""Tracking-anchor (recenter) detection.

PICO's tracking origin sits at the anchor established when the headset is
(re)centered; it is not on the floor and no explicit recenter event exists in
the SDK feedback or the tracking JSON. A recenter therefore has to be inferred
from a sudden discontinuity in the joint coordinates.

This module distinguishes a recenter from real motion such as jumping: a jump
is continuous (per-frame foot displacement stays within a few cm at tracking
rate), while a recenter moves every joint by a large offset in a single frame.
"""

from __future__ import annotations

from dataclasses import dataclass

from .types import TrackingFrame

# Feet joint indices in the body joint array (10 left, 11 right).
FOOT_JOINTS = (10, 11)


def feet_min_z(frame: TrackingFrame) -> float | None:
    """Lowest valid foot height in the frame, or None if feet are untracked."""
    values = [
        frame.body[i].tracking.pose.position.z
        for i in FOOT_JOINTS
        if i < len(frame.body) and frame.body[i].tracking.valid
    ]
    return min(values) if values else None


@dataclass
class ReanchorEvent:
    """Emitted when the tracking anchor jumps between two consecutive frames."""

    frame_seq: int
    previous_ground_z: float
    new_ground_z: float

    @property
    def offset(self) -> float:
        return self.new_ground_z - self.previous_ground_z


class ReanchorDetector:
    """Detects tracking-anchor resets from single-frame discontinuities.

    A per-frame foot displacement beyond ``jump_threshold_m`` is physically
    impossible at tracking rate (0.15 m/frame at 90 Hz equals 13.5 m/s), so it
    indicates the anchor moved rather than the user. Real jumps stay far below
    the threshold because they develop over many frames.
    """

    def __init__(self, jump_threshold_m: float = 0.15) -> None:
        self.jump_threshold_m = jump_threshold_m
        self._last_ground_z: float | None = None

    def ground_z(self) -> float | None:
        """Best estimate of the floor height in anchor coordinates."""
        return self._last_ground_z

    def update(self, frame: TrackingFrame) -> ReanchorEvent | None:
        """Feed a frame; returns a ReanchorEvent on the frame the anchor moved."""
        current = feet_min_z(frame)
        if current is None:
            return None

        previous = self._last_ground_z
        self._last_ground_z = current
        if previous is None or abs(current - previous) <= self.jump_threshold_m:
            return None
        return ReanchorEvent(
            frame_seq=frame.frame_seq,
            previous_ground_z=previous,
            new_ground_z=current,
        )
