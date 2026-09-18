"""Print tracking-anchor (recenter) events and live ground height.

Usage:
    python detect_recenter.py [--domain 0] [--topic pico/tracking]

Recenter the headset while this runs to see an event fire; jumping in place
must NOT fire one (per-frame foot motion stays far below the threshold).
"""

import argparse
import time

from pico_tracking import PicoTrackingClient, ReanchorDetector


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--topic", default="pico/tracking")
    args = parser.parse_args()

    client = PicoTrackingClient(args.domain, args.topic)
    detector = ReanchorDetector()

    print(f"listening on domain={args.domain} topic={args.topic}; recenter the headset to trigger")
    last_report = 0.0
    for frame in client.frames():
        event = detector.update(frame)
        if event is not None:
            print(
                f"[reanchor] seq={event.frame_seq} "
                f"offset={event.offset:+.3f} m "
                f"(ground {event.previous_ground_z:+.3f} -> {event.new_ground_z:+.3f})"
            )
        now = time.monotonic()
        if now - last_report >= 5.0:
            ground = detector.ground_z()
            if ground is not None:
                print(f"[ground] z={ground:+.3f} m")
            last_report = now


if __name__ == "__main__":
    main()
