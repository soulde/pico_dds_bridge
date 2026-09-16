from __future__ import annotations

import time
from typing import Iterator

from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from .types import TrackingFrame


class PicoTrackingClient:
    """
    User-facing API.

    This package has no dependency on XRoboToolkit/PICO SDK. It only consumes
    the stable DDS contract published by pico_dds_bridge.
    """

    def __init__(
        self,
        domain_id: int = 0,
        topic_name: str = "pico/tracking",
    ) -> None:
        self.participant = DomainParticipant(domain_id)
        self.topic = Topic(
            self.participant,
            topic_name,
            TrackingFrame,
        )
        self.reader = DataReader(self.participant, self.topic)

    def take_latest(self) -> TrackingFrame | None:
        samples = self.reader.take(N=64)
        return samples[-1] if samples else None

    def wait_next(
        self,
        timeout_s: float | None = None,
        poll_s: float = 0.001,
    ) -> TrackingFrame | None:
        deadline = None if timeout_s is None else time.monotonic() + timeout_s

        while True:
            sample = self.reader.take_next()
            if sample is not None:
                return sample

            if deadline is not None and time.monotonic() >= deadline:
                return None

            time.sleep(poll_s)

    def frames(self) -> Iterator[TrackingFrame]:
        while True:
            sample = self.wait_next()
            if sample is not None:
                yield sample
