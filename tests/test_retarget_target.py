from pathlib import Path

from scripts.retarget_target import available_robots, resolve_target


def test_g1_keeps_tuned_default_config():
    config, mjcf = resolve_target("unitree_g1", Path("third_party/GMR"))

    assert config.name == "xrobot_soulde_to_g1.json"
    assert mjcf is None


def test_installed_robot_entry_point_registers_model_and_config(monkeypatch, tmp_path):
    from general_motion_retargeting import IK_CONFIG_DICT, ROBOT_XML_DICT

    config = tmp_path / "pico_to_sample_bot.json"
    model = tmp_path / "sample_bot.xml"
    config.write_text("{}")
    model.write_text("<mujoco/>")

    def register_gmr():
        monkeypatch.setitem(IK_CONFIG_DICT.setdefault("xrobot", {}), "sample_bot", config)
        monkeypatch.setitem(ROBOT_XML_DICT, "sample_bot", model)

    monkeypatch.setattr(
        "scripts.retarget_target.gmr_available_robots",
        lambda: ("unitree_g1", "unitree_g1_with_hands", "sample_bot"),
    )
    monkeypatch.setattr("scripts.retarget_target.register_robot", lambda name: register_gmr())

    assert "sample_bot" in available_robots()
    assert resolve_target("sample_bot", tmp_path) == (config, model)
