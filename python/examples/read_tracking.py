from pico_tracking import PicoTrackingClient


def main() -> None:
    pico = PicoTrackingClient()

    for frame in pico.frames():
        p = frame.head.pose.position
        print(
            f"seq={frame.sequence} "
            f"head_valid={frame.head.valid} "
            f"head=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
            f"body={frame.body_count} "
            f"trackers={frame.tracker_count}"
        )


if __name__ == "__main__":
    main()
