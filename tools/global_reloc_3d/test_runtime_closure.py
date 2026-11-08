"""The supervisor's import closure is the documented runtime set (README.md section 1): what the robot runs is exactly the documented set. Static: parses the files, runs nothing. python3 -m pytest -q test_runtime_closure.py"""
import ast
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOTS = ('gn_supervisor_node', 'search_v2')            # the node the launch starts and the worker it spawns
RUNTIME = {'corr_cpp', 'correlative_search', 'gn_supervisor_core', 'gn_supervisor_node', 'imu_ready', 'lio_submap', 'livox_frames', 'offline_s2', 'reloc_core', 'reloc_live',
           'search_tiles', 'search_v2', 'verify'}
SETUP = {'prepare_map'}                                # not imported by the runtime, documented in the README


def local_imports(module):
    mods = {os.path.splitext(f)[0] for f in os.listdir(HERE) if f.endswith('.py')}
    out = set()
    for n in ast.walk(ast.parse(open(os.path.join(HERE, module + '.py')).read())):
        if isinstance(n, ast.Import):
            out |= {a.name.split('.')[0] for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.module:
            out.add(n.module.split('.')[0])
    return out & mods


def closure():
    seen, todo = set(), list(ROOTS)
    while todo:
        m = todo.pop()
        if m not in seen:
            seen.add(m)
            todo += local_imports(m)
    return seen


def test_the_runtime_set_is_the_documented_one():
    got = closure()
    assert got == RUNTIME, f"imported by the supervisor but undocumented: {sorted(got - RUNTIME)}; documented but no longer imported: {sorted(RUNTIME - got)} (update README.md section 1 and this test)"


def test_every_module_of_the_directory_is_runtime_or_setup():
    """New scripts do not accumulate here: a module that nobody imports must be a documented setup helper."""
    mods = {os.path.splitext(f)[0] for f in os.listdir(HERE) if f.endswith('.py') and not f.startswith('test_')}
    assert mods == RUNTIME | SETUP, f"undocumented: {sorted(mods - RUNTIME - SETUP)}; documented but absent: {sorted((RUNTIME | SETUP) - mods)}"


def test_the_readme_lists_every_runtime_module():
    text = open(os.path.join(HERE, 'README.md')).read()
    missing = [m for m in RUNTIME if m + '.py' not in text and m not in text]
    assert not missing, missing
