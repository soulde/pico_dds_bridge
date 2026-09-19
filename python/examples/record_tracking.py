"""Record raw pico/tracking DDS frames to an .npz file for offline analysis.

Usage:
    python record_tracking.py --seconds 20 --out /tmp/tracking.npz

Do a T-pose at the start, then walk/move naturally. Saved arrays: body
positions (N,24,3), body quats xyzw (N,24,4), valid mask, timestamps.
"""

import argparse
import time

import numpy as np

from pico_tracking import PicoTrackingClient


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--topic", default="pico/tracking")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--out", default="/tmp/tracking.npz")
    args = parser.parse_args()

    client = PicoTrackingClient(args.domain, args.topic)
    positions, quats, valids, stamps = [], [], [], []
    deadline = time.monotonic() + args.seconds
    print(f"recording {args.seconds:.0f}s from {args.topic}; T-pose first, then move naturally")
    try:
        while time.monotonic() < deadline:
            frame = client.wait_next(timeout_s=2.0)
            if frame is None:
                continue
            count = min(int(frame.body_count), len(frame.body))
            p = np.zeros((24, 3))
            q = np.zeros((24, 4))
            v = np.zeros(24, dtype=bool)
            for i in range(count):
                state = frame.body[i].tracking
                p[i] = [
                    state.pose.position.x,
                    state.pose.position.y,
                    state.pose.position.z,
                ]
                q[i] = [
                    state.pose.orientation.x,
                    state.pose.orientation.y,
                    state.pose.orientation.z,
                    state.pose.orientation.w,
                ]
                v[i] = bool(state.valid)
            positions.append(p)
            quats.append(q)
            valids.append(v)
            stamps.append(frame.receive_timestamp_ns)
    except KeyboardInterrupt:
        pass

    np.savez_compressed(
        args.out,
        positions=np.asarray(positions),
        quats=np.asarray(quats),
        valid=np.asarray(valids),
        timestamps_ns=np.asarray(stamps),
    )
    print(f"saved {len(positions)} frames to {args.out}")


if __name__ == "__main__":
    main()
