#!/usr/bin/env python3
"""Self-check for glim_config.py. Run: python3 test_glim_config.py (or pytest)"""
import json
import pathlib
import subprocess
import sys
import tempfile

SCRIPT = pathlib.Path(__file__).with_name("glim_config.py")

# GLIM config files carry // and /* */ comments; "//" inside a string must survive.
DEFAULTS = {
    "config.json": '/* header */\n{\n  "global": {"config_path": ""} // trailing\n}\n',
    "config_preprocess.json": '{\n  // comment\n  "preprocess": {"distance_far_thresh": 100.0, "k": [1, 2], "url": "a//b"}\n}\n',
}


def run(defaults, out, *overlays):
    return subprocess.run([sys.executable, str(SCRIPT), str(defaults), str(out), *map(str, overlays)],
                          capture_output=True, text=True)


def setup(tmp, overlay):
    defaults, out = tmp / "defaults", tmp / "out"
    defaults.mkdir()
    for name, text in DEFAULTS.items():
        (defaults / name).write_text(text)
    path = tmp / "overlay.json"
    path.write_text(json.dumps(overlay))
    return defaults, out, path


def test_merge():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        defaults, out, overlay = setup(tmp, {"config_preprocess.json": {"preprocess": {"distance_far_thresh": 8.0, "k": [3]}}})
        second = tmp / "second.json"
        second.write_text(json.dumps({"config_preprocess.json": {"preprocess": {"distance_far_thresh": 9.0}}}))
        proc = run(defaults, out, overlay, second)
        assert proc.returncode == 0, proc.stderr
        pre = json.loads((out / "config_preprocess.json").read_text())["preprocess"]
        assert pre == {"distance_far_thresh": 9.0, "k": [3], "url": "a//b"}, pre  # later overlay wins, lists replaced
        assert json.loads((out / "config.json").read_text()) == {"global": {"config_path": ""}}  # untouched file copied


def test_unknown_key_fails():
    # GLIM silently ignores unknown parameters; the overlay tool must not.
    for bad in ({"config_preprocess.json": {"preprocess": {"crop_bbox_invert": True}}},
                {"config_preprocess.json": {"nope": {"distance_far_thresh": 1.0}}},
                {"config_missing.json": {"x": {"y": 1}}},
                {"config_preprocess.json": {"preprocess": 5}},
                {"config_preprocess.json": 5}):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = pathlib.Path(tmp)
            defaults, out, overlay = setup(tmp, bad)
            proc = run(defaults, out, overlay)
            assert proc.returncode == 1 and proc.stderr.startswith("error:") and "Traceback" not in proc.stderr, proc
            assert not out.exists(), "nothing written on error"


if __name__ == "__main__":
    test_merge()
    test_unknown_key_fails()
    print("ok")
