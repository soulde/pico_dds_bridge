# PICO Tracking MuJoCo Viewer

Standalone `uv` project that subscribes to the `pico/tracking` Cyclone DDS
topic and displays tracking points in MuJoCo.

The scene includes:

- G1-style blue gradient skybox, haze, lighting, and checkerboard floor;
- world axes: X red, Y green, Z blue;
- head and controller points;
- hand, body, and motion-tracker points;
- lines between adjacent valid points in each collection.

The viewer expects the bridge and this process to use the same DDS domain and
topic. It does not import the repository's Python package.

## Run

From this directory:

```bash
uv sync
uv run pico-mujoco-viewer --domain 0 --topic pico/tracking
```

To preview the scene with a neutral skeleton before tracking data arrives:

```bash
uv run pico-mujoco-viewer --show-default-skeleton
```

The preview skeleton is replaced automatically when the first DDS frame is
received.

The project is configured to use the Aliyun PyPI mirror by default. Override
it for one command with `uv sync --index-url https://pypi.org/simple` if
needed.

Start the C++ bridge first. When using the bridge's robot-coordinate option,
start it with the same output convention:

```bash
../../../scripts/run.sh --robot-coordinates
```

The viewer has no coordinate conversion of its own; it displays the coordinates
published by DDS directly.