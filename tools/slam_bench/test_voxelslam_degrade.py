"""The degrade_bound override changes that one key and nothing else. Run: python3 test_voxelslam_degrade.py"""
import subprocess
from pathlib import Path

SED = Path(__file__).with_name("voxelslam_degrade.sed")
# the shape of the port's config/mid360.yaml around the key (commit 4d06f3e, line 29: `degrade_bound: 10`, indented)
CFG = "    odometry:\n      min_eigen_value: 0.0025\n      degrade_bound: 10\n      other: 3\n    local:\n      min_eigen_value: 0.01\n"


def apply(text):
    return subprocess.run(["sed", "-f", str(SED)], input=text, capture_output=True, text=True, check=True).stdout


def test_only_degrade_bound_changes_and_its_indentation_stays():
    assert apply(CFG) == CFG.replace("degrade_bound: 10\n", "degrade_bound: 100000\n")


def test_applying_it_twice_changes_nothing_more():
    assert apply(apply(CFG)) == apply(CFG)


if __name__ == "__main__":
    test_only_degrade_bound_changes_and_its_indentation_stays()
    test_applying_it_twice_changes_nothing_more()
    print("ok")
