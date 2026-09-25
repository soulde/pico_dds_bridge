"""Resolve the robot model and IK configuration for the PICO retarget node."""

from pathlib import Path

from general_motion_retargeting.robot_plugins import (
    available_robots as gmr_available_robots,
    register_robot,
)


BUILTIN_ROBOTS = ("unitree_g1", "unitree_g1_with_hands")


def available_robots() -> tuple[str, ...]:
    """Return robot names supplied by GMR and installed packages."""
    return gmr_available_robots()


def resolve_target(
    robot: str,
    gmr_root: Path,
    ik_config: Path | None = None,
    mjcf: Path | None = None,
) -> tuple[Path, Path | None]:
    from general_motion_retargeting import IK_CONFIG_DICT, ROBOT_XML_DICT

    if robot not in available_robots():
        raise ValueError(f"unknown robot {robot!r}; install its integration package")
    register_robot(robot)
    if robot not in BUILTIN_ROBOTS:
        try:
            registered_config = IK_CONFIG_DICT["xrobot"][robot]
            registered_model = ROBOT_XML_DICT[robot]
        except KeyError as error:
            raise RuntimeError(
                f"robot plugin {robot!r} did not register an XRobot IK config and MJCF"
            ) from error
        if ik_config is None:
            ik_config = Path(registered_config)
        if mjcf is None:
            mjcf = Path(registered_model)
    elif ik_config is None:
        ik_config = (
            gmr_root
            / "general_motion_retargeting"
            / "ik_configs"
            / "xrobot_soulde_to_g1.json"
        )

    if not ik_config.is_file():
        raise FileNotFoundError(f"GMR IK config does not exist: {ik_config}")
    if mjcf is not None:
        if not mjcf.is_file():
            raise FileNotFoundError(f"MJCF does not exist: {mjcf}")
        ROBOT_XML_DICT[robot] = mjcf
    return ik_config, mjcf
