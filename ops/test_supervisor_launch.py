"""supervisor_nodes() of nav_live.launch.py (needs the ROS 2 python packages, so inside the image): python3 ops/test_supervisor_launch.py"""
import importlib.util
import os

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("nav_live_launch", os.path.join(here, "nav_live.launch.py"))
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def nodes(supervisor, prior=None):
    os.environ.pop("SUPERVISOR", None)
    os.environ.pop("SUP_PRIOR", None)
    if supervisor is not None:
        os.environ["SUPERVISOR"] = supervisor
    if prior is not None:
        os.environ["SUP_PRIOR"] = prior
    return launch.supervisor_nodes()


assert nodes(None) == []
for off in ("", "0", "false", "False", "no", "off"):
    assert nodes(off, "/greloc/p.xyz") == [], off                  # the flag is read with the same rule as ops/flags.sh: 0 is off
assert len(nodes("1", "/greloc/prior/m/prior/map.xyz")) == 1
try:
    nodes("1")                                                     # launched by hand with the flag but without nav_live.sh: say why, do not KeyError
    raise AssertionError("expected RuntimeError")
except RuntimeError as e:
    assert "SUP_PRIOR" in str(e) and "nav_live.sh" in str(e), e
print("ok")
