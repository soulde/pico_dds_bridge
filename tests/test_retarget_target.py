from pathlib import Path
import sys
import types

from scripts.retarget_target import resolve_target


def test_g1_keeps_tuned_default_config():
    config, mjcf = resolve_target("unitree_g1", Path("third_party/GMR"))

    assert config.name == "xrobot_soulde_to_g1.json"
    assert mjcf is None


def test_chocolate_loads_installed_registration(monkeypatch, tmp_path):
    from general_motion_retargeting import IK_CONFIG_DICT, ROBOT_XML_DICT

    config = tmp_path / "pico_to_chocolate.json"
    model = tmp_path / "chocolate.xml"
    config.write_text("{}")
    model.write_text("<mujoco/>")
    plugin = types.ModuleType("robots.chocolate.registration")

    def register_gmr():
        monkeypatch.setitem(IK_CONFIG_DICT.setdefault("xrobot", {}), "chocolate", config)
        monkeypatch.setitem(ROBOT_XML_DICT, "chocolate", model)

    plugin.register_gmr = register_gmr
    monkeypatch.setitem(sys.modules, "robots.chocolate.registration", plugin)

    assert resolve_target("chocolate", tmp_path) == (config, model)
