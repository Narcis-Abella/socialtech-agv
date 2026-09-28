#!/usr/bin/env python3
"""Build a GLIM config directory: the installed defaults plus JSON overlays.

Overlays hold only what differs from the defaults: {"<file>.json": {"<section>": {"<param>": value}}}.
Applied in order; a later overlay wins; lists are replaced, not merged. A file, section or parameter
that the defaults lack is an error: GLIM silently ignores unknown parameters, so a typo or a parameter
from another GLIM version would otherwise run with the default instead.

Output files are plain JSON (the defaults' comments are dropped); GLIM reads them the same.

Usage: glim_config.py <defaults_dir> <out_dir> <overlay.json>...
  defaults_dir in the robot image: /opt/ros/jazzy/share/glim/config
"""
import json
import pathlib
import sys


def strip_comments(text):
    """Remove // and /* */ comments outside strings (GLIM's JSON parser accepts them, Python's does not)."""
    out, i, in_string = [], 0, False
    while i < len(text):
        c = text[i]
        if in_string:
            out.append(c)
            if c == "\\":
                out.append(text[i + 1:i + 2])  # empty at end of text: json.loads reports it
                i += 1
            elif c == '"':
                in_string = False
        elif c == '"':
            in_string = True
            out.append(c)
        elif text.startswith("//", i):
            end = text.find("\n", i)
            i = len(text) if end < 0 else end
            continue
        elif text.startswith("/*", i):
            end = text.find("*/", i)
            if end < 0:
                raise ValueError("unterminated /* comment")
            i = end + 2
            continue
        else:
            out.append(c)
        i += 1
    return "".join(out)


def load(path):
    try:
        return json.loads(strip_comments(path.read_text(encoding="utf-8")))
    except (OSError, ValueError) as e:
        sys.exit(f"error: cannot read {path}: {e}")


def apply(configs, overlay, source):
    def is_dict_of_dicts(d):
        return isinstance(d, dict) and all(isinstance(v, dict) for v in d.values())

    if not is_dict_of_dicts(overlay) or not all(is_dict_of_dicts(s) for s in overlay.values()):
        sys.exit(f'error: {source}: expected {{"<file>.json": {{"<section>": {{"<param>": value}}}}}}')
    for name, sections in overlay.items():
        if name not in configs:
            sys.exit(f"error: {source}: {name} is not a GLIM config file in the defaults")
        for section, params in sections.items():
            if section not in configs[name]:
                sys.exit(f"error: {source}: {name} has no section '{section}'")
            for param, value in params.items():
                if param not in configs[name][section]:
                    sys.exit(f"error: {source}: {name}:{section} has no parameter '{param}'")
                configs[name][section][param] = value


def main():
    if len(sys.argv) < 4:
        sys.exit("usage: glim_config.py <defaults_dir> <out_dir> <overlay.json>...")
    defaults, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    configs = {p.name: load(p) for p in sorted(defaults.glob("*.json"))}
    if "config.json" not in configs:
        sys.exit(f"error: {defaults} is not a GLIM config directory (no config.json)")
    for source in map(pathlib.Path, sys.argv[3:]):
        apply(configs, load(source), source)

    try:
        out.mkdir(parents=True, exist_ok=True)
        for name, config in configs.items():
            (out / name).write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    except OSError as e:
        sys.exit(f"error: cannot write {out}: {e}")
    print(f"wrote {len(configs)} config files to {out}")


if __name__ == "__main__":
    main()
