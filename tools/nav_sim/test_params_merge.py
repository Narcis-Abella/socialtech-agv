"""Run: python3 -B test_params_merge.py"""
import os
import tempfile

import yaml

import params_merge as pm

BASE = {"planner_server": {"ros__parameters": {"planner_plugins": ["GridBased"], "GridBased": {"plugin": "A", "tolerance": 0.25, "smoother": {"max_its": 1000}}}}, "amcl": {"ros__parameters": {"alpha1": 0.2}}}


def test_new_blocks_are_added_and_existing_values_overridden():
    over = {"planner_server": {"ros__parameters": {"planner_plugins": ["GridBased", "NavFn"], "NavFn": {"plugin": "B", "tolerance": 0.5}, "GridBased": {"tolerance": 0.1}}}}
    out = pm.merge(BASE, over)
    p = out["planner_server"]["ros__parameters"]
    assert p["planner_plugins"] == ["GridBased", "NavFn"]                       # lists are replaced whole
    assert p["NavFn"] == {"plugin": "B", "tolerance": 0.5}
    assert p["GridBased"] == {"plugin": "A", "tolerance": 0.1, "smoother": {"max_its": 1000}}   # siblings kept, nested dict merged
    assert out["amcl"] == BASE["amcl"]


def test_inputs_are_not_modified():
    import copy
    base, over = copy.deepcopy(BASE), {"amcl": {"ros__parameters": {"alpha1": 0.01}}}
    pm.merge(base, over)
    assert base == BASE and over == {"amcl": {"ros__parameters": {"alpha1": 0.01}}}


def test_the_output_does_not_share_structure_with_the_overlay():
    over = {"planner_server": {"ros__parameters": {"NavFn": {"tolerance": 0.5, "extra": [1, 2]}}}}
    out = pm.merge(BASE, over)
    out["planner_server"]["ros__parameters"]["NavFn"]["extra"].append(3)
    out["planner_server"]["ros__parameters"]["NavFn"]["tolerance"] = 9.0
    assert over["planner_server"]["ros__parameters"]["NavFn"] == {"tolerance": 0.5, "extra": [1, 2]}


def test_a_type_change_of_an_existing_scalar_is_refused():
    for bad in ({"amcl": {"ros__parameters": {"alpha1": "x"}}}, {"amcl": {"ros__parameters": {"alpha1": True}}}, {"amcl": {"ros__parameters": {"alpha1": 3}}} if False else {"amcl": {"ros__parameters": {"alpha1": [1]}}}):
        try:
            pm.merge(BASE, bad)
        except ValueError as e:
            assert "alpha1" in str(e)
            continue
        raise AssertionError("must refuse")
    widened = pm.merge(BASE, {"amcl": {"ros__parameters": {"alpha1": 1}}})["amcl"]["ros__parameters"]["alpha1"]
    assert widened == 1.0 and isinstance(widened, float)             # an int given for a float parameter becomes a float (a ROS 2 double refuses an int)


def test_main_writes_the_merged_file():
    with tempfile.TemporaryDirectory() as d:
        b, o, out = (os.path.join(d, n) for n in ("b.yaml", "o.yaml", "out.yaml"))
        yaml.safe_dump(BASE, open(b, "w"))
        yaml.safe_dump({"planner_server": {"ros__parameters": {"NavFn": {"tolerance": 0.5}}}}, open(o, "w"))
        pm.main([b, o, out])
        assert yaml.safe_load(open(out))["planner_server"]["ros__parameters"]["NavFn"]["tolerance"] == 0.5


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
