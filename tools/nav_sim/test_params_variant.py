"""Run: python3 -B test_params_variant.py"""
import os
import tempfile

import yaml

import params_variant as pv

BASE = {"amcl": {"ros__parameters": {"alpha1": 0.2, "max_particles": 2000, "do_beamskip": False, "robot_model_type": "nav2_amcl::DifferentialMotionModel", "initial_pose": {"x": -13.0}}},
        "controller_server": {"ros__parameters": {"FollowPath": {"desired_linear_vel": 0.3, "plugin": "x"}, "controller_plugins": ["FollowPath"]}}}


def apply(*assignments):
    return pv.apply(BASE, assignments)


def test_sets_nested_values_and_leaves_the_rest():
    out = apply("amcl.ros__parameters.alpha1=0.01", "controller_server.ros__parameters.FollowPath.desired_linear_vel=0.5")
    assert out["amcl"]["ros__parameters"]["alpha1"] == 0.01
    assert out["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"] == 0.5
    assert out["amcl"]["ros__parameters"]["max_particles"] == 2000
    assert BASE["amcl"]["ros__parameters"]["alpha1"] == 0.2          # the input is not modified


def test_an_integer_given_for_a_double_stays_a_double():
    out = apply("amcl.ros__parameters.alpha1=1", "controller_server.ros__parameters.FollowPath.desired_linear_vel=1")
    assert out["amcl"]["ros__parameters"]["alpha1"] == 1.0 and isinstance(out["amcl"]["ros__parameters"]["alpha1"], float)
    assert isinstance(out["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"], float)


def test_a_fractional_value_for_an_integer_parameter_is_refused():
    try:
        apply("amcl.ros__parameters.max_particles=1500.5")
    except ValueError as e:
        assert "max_particles" in str(e)
        return
    raise AssertionError("must refuse")


def test_the_type_class_must_match_for_strings_and_booleans():
    out = apply("amcl.ros__parameters.do_beamskip=true", "amcl.ros__parameters.robot_model_type=nav2_amcl::OmniMotionModel")
    assert out["amcl"]["ros__parameters"]["do_beamskip"] is True and out["amcl"]["ros__parameters"]["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    for bad in ("amcl.ros__parameters.do_beamskip=0.5", "amcl.ros__parameters.robot_model_type=3", "amcl.ros__parameters.alpha1=abc", "amcl.ros__parameters.alpha1=true", "amcl.ros__parameters.max_particles=true"):
        try:
            apply(bad)
        except ValueError:
            continue
        raise AssertionError("must refuse " + bad)


def test_unknown_keys_are_refused_not_created():
    for bad in ("amcl.ros__parameters.alpha9=0.1", "amcl.ros__parameters.initial_pose.q=1.0", "nope.ros__parameters.x=1", "amcl.ros__parameters.alpha1.deeper=1", "no_equals_sign"):
        try:
            apply(bad)
        except ValueError:
            continue
        raise AssertionError("must refuse " + bad)


def test_lists_are_replaced_whole_and_roundtrip_through_a_file():
    out = apply("controller_server.ros__parameters.controller_plugins=[A, B]")
    assert out["controller_server"]["ros__parameters"]["controller_plugins"] == ["A", "B"]
    with tempfile.TemporaryDirectory() as d:
        src, dst = os.path.join(d, "a.yaml"), os.path.join(d, "b.yaml")
        yaml.safe_dump(BASE, open(src, "w"))
        pv.main([src, dst, "amcl.ros__parameters.alpha1=0.05"])
        assert yaml.safe_load(open(dst))["amcl"]["ros__parameters"]["alpha1"] == 0.05


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
