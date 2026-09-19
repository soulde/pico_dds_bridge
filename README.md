# pico_dds_bridge

XRoboToolkit/PICO tracking -> Cyclone DDS bridge.

The vendor SDK ends at the C++ bridge. Retargeting, recording, playback,
visualization and dataset code consume only the DDS contract.

## Runtime architecture

```text
PICO 4 Ultra
    |
    | Wi-Fi/LAN
    v
XRoboToolkit-PC-Service
    |
    | PXREARobotSDK / gRPC
    v
pico_dds_bridge   (C++11)
    |
    | DDS topic: pico/tracking
    v
pico_tracking     (Python)
    |
    +--> real-time retargeting
    +--> recorder
    +--> playback
    +--> visualization
    +--> dataset pipeline
```

The bridge intentionally keeps the PICO tracking coordinate convention
unchanged. Coordinate conversion belongs in a named downstream transform, not
as a hidden side effect in the transport adapter.

## Dependencies

Ubuntu 22.04/24.04:

- GCC/Clang with C++11
- CMake >= 3.22
- Ninja
- Eclipse Cyclone DDS
- nlohmann/json
- XRoboToolkit `PXREARobotSDK`
- Python >= 3.10 for the optional user package

The scripts install CycloneDDS and PXREARobotSDK into this repository's
`.deps/` directory. They do not install vendor SDK files globally.

## 1. XRoboToolkit PC Service

The PC Service application is a separate runtime process and still needs to be
installed and started. This repository installs/builds only the C++ Robot SDK
used to talk to that service.

Install the official `.deb` as documented upstream and start the PC Service
before running the bridge.

## 2. Bootstrap

```bash
chmod +x scripts/*.sh
./scripts/bootstrap.sh
```

Equivalent manual sequence:

```bash
./scripts/install_deps_ubuntu.sh
./scripts/install_cyclonedds.sh
./scripts/install_xrobotoolkit_sdk.sh
./scripts/install_python.sh
```

Version overrides are supported:

```bash
CYCLONEDDS_REF=11.0.1 ./scripts/install_cyclonedds.sh
XROBO_REF=main ./scripts/install_xrobotoolkit_sdk.sh
```

## 3. Build

```bash
./scripts/build.sh
```

The build uses:

```text
.deps/
├── cyclonedds/
└── xrobotoolkit/
```

No vendor source is copied into the project source tree.

## 4. Run

Start:

1. XRoboToolkit on the PICO.
2. XRoboToolkit PC Service on the PC.
3. RouDi: `./scripts/run_roudi.sh`.
4. This bridge.

Then:

```bash
./scripts/run.sh
```

The runtime scripts configure CycloneDDS PSMX through
`config/cyclonedds.xml`. Use `./scripts/run.sh --require-shm` to fail fast if
shared-memory/PSMX is unavailable.

Optional DDS settings:

```bash
./scripts/run.sh --domain 0 --topic pico/tracking
```

Select the tracking output coordinate convention with one parameter:

```bash
./scripts/run.sh --coordinates pico
./scripts/run.sh --coordinates robot
./scripts/run.sh --coordinates xrobot
```

`pico` is the default and keeps the PICO coordinate system (`X` right,
`Y` up, `Z` inward). `robot` outputs (`X` forward, `Y` left, `Z` up).
`xrobot` applies the same `(x, y, z) -> (x, -z, y)` conversion used by
GMR's XRobotStreamer.

## 5. Python consumer

```bash
source .venv/bin/activate
python python/examples/read_tracking.py
```

Application code:

```python
from pico_tracking import PicoTrackingClient

pico = PicoTrackingClient()

frame = pico.wait_next()
print(frame.head.pose.position)
print(frame.body[:frame.body_count])
print(frame.trackers[:frame.tracker_count])
```

The Python package has zero XRoboToolkit dependency.

## 6. GMR retargeting

GMR is pinned as a git submodule at `third_party/GMR`; the retarget node and
the tuned XRobot-to-G1 config use that checkout directly. Clone recursively,
or initialize it after cloning:

```bash
git clone --recurse-submodules https://github.com/soulde/pico_dds_bridge.git
git submodule update --init --recursive
source .venv/bin/activate
pip install -e third_party/GMR
```

Run the bridge with the GMR/XRobot coordinate convention, then start the
retarget node:

```bash
./scripts/run.sh --coordinates xrobot
python scripts/pico_dds_retarget_node.py --visualize
```

The default robot is GMR's `unitree_g1` 29-DoF model and the default IK file
is `third_party/GMR/general_motion_retargeting/ik_configs/xrobot_soulde_to_g1.json`.
The DDS adapter accepts `xrobot` input directly; `pico` is available for a
bridge that intentionally leaves the raw PICO coordinates unchanged.

## Data contract

High-rate DDS topic:

```text
pico/tracking
```

Contains:

- frame sequence
- PICO source timestamp
- bridge receive timestamp
- input mode
- HMD pose
- left/right controller pose + inputs
- left/right 26-joint hand tracking
- 24-joint body tracking
- up to 5 independent motion trackers

The IDL is fixed-size on purpose, avoiding unbounded DDS allocations in the
high-rate tracking path.

The bridge reserves **5 Motion Tracker slots**. `tracker_count` is authoritative,
so consumers must only use `trackers[:tracker_count]`. Five-tracker operation
requires a PICO runtime/firmware and tracking mode that exposes five trackers;
older stacks may expose fewer.

## Threading/backpressure

The XRoboToolkit callback does not parse JSON and does not publish DDS.

It only:

1. copies the SDK-owned JSON string;
2. timestamps receipt;
3. pushes it into a bounded queue.

The main bridge loop performs JSON parsing and DDS publication. If the bridge
falls behind, the bounded queue drops the oldest callback frame. This prevents
teleoperation latency from growing without bound.

## Timestamp policy

- `source_timestamp_ns`: timestamp originating from the PICO/XR data.
- `receive_timestamp_ns`: PC `CLOCK_REALTIME` equivalent timestamp when the
  callback is copied.

Downstream synchronization should prefer `source_timestamp_ns`.

## Next extension points

Recommended additions without changing the core boundary:

- `pico/device_status` low-rate topic for device SN, battery, online state.
- named coordinate-transform node:
  `pico/tracking -> human/body_tracking`.
- DDS recorder/replayer or MCAP writer.
- calibration metadata topic.
- QoS profiles in XML.
