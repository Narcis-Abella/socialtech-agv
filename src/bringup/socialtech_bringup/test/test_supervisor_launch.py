"""supervisor_nodes() of nav_live.launch.py (needs the ROS 2 python packages, so inside the image): python3 src/bringup/socialtech_bringup/test/test_supervisor_launch.py"""
import importlib.util
import os

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("nav_live_launch", os.path.join(here, "..", "launch", "nav_live.launch.py"))
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


# the worker's binaries are looked for along the ament path: a fake prefix has them, another has the Python alone
import tempfile
with tempfile.TemporaryDirectory() as d:
    for prefix, files in (("full", ("register_v2", "libcorr_search.so")), ("pyonly", ("gn_supervisor_node.py",))):
        os.makedirs(os.path.join(d, prefix, "lib", "gn_supervisor"))
        for f in files:
            path = os.path.join(d, prefix, "lib", "gn_supervisor", f)
            open(path, "w").close()
            os.chmod(path, 0o755)
    os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join([os.path.join(d, "pyonly"), os.path.join(d, "full")])    # an overlay in front of the image
    assert launch.supervisor_lib_dir() == os.path.join(d, "full", "lib", "gn_supervisor")
    os.environ["AMENT_PREFIX_PATH"] = os.path.join(d, "pyonly")
    try:
        launch.supervisor_lib_dir()
        raise AssertionError("expected RuntimeError")
    except RuntimeError as e:
        assert "register_v2" in str(e) and "SUPERVISOR=0" in str(e), e
    os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join([os.path.join(d, "pyonly"), os.path.join(d, "full")])

    assert nodes(None) == []
    for off in ("", "0", "false", "False", "no", "off"):
        assert nodes(off, "/map/prior/map.xyz") == [], off          # the flag is read with the same rule as socialtech_bringup/scripts/flags.sh: 0 is off
    n = nodes("1", "/map/prior/map.xyz")
    assert len(n) == 1
    try:
        nodes("1")                                                     # launched by hand with the flag but without nav_live.sh: say why, do not KeyError
        raise AssertionError("expected RuntimeError")
    except RuntimeError as e:
        assert "SUP_PRIOR" in str(e) and "nav_live.sh" in str(e), e
print("ok")
