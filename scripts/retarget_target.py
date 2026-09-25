"""Resolve the robot model and IK configuration for the PICO retarget node."""

from pathlib import Path


def resolve_target(
    robot: str,
    gmr_root: Path,
    ik_config: Path | None = None,
    mjcf: Path | None = None,
) -> tuple[Path, Path | None]:
    from general_motion_retargeting import IK_CONFIG_DICT, ROBOT_XML_DICT

    if robot == "chocolate":
        try:
            from robots.chocolate.registration import register_gmr
        except ModuleNotFoundError as error:
            if error.name not in {
                "robots",
                "robots.chocolate",
                "robots.chocolate.registration",
            }:
                raise
            raise RuntimeError(
                "Chocolate is not installed in this Python environment; "
                "run: pip install -e /path/to/gmr-chocolate"
            ) from error
        register_gmr()
        if ik_config is None:
            ik_config = Path(IK_CONFIG_DICT["xrobot"]["chocolate"])
        if mjcf is None:
            mjcf = Path(ROBOT_XML_DICT["chocolate"])
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
