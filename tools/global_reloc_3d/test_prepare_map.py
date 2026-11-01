"""prepare_map.py: the relocalizer prior from the map_clean.ply of a make_map folder."""
import json

import numpy as np
import pytest

import prepare_map


def write_ply(path, P):
    P = np.asarray(P, "<f4")
    path.write_bytes(b"ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n" % len(P) + P.tobytes())


def map_dir(tmp, points, report=None):
    d = tmp / "map"
    d.mkdir()
    write_ply(d / "map_clean.ply", points)
    if report is not None:
        (d / "report.json").write_text(json.dumps(report))
    return d


def test_prepare_map_crops_z_and_voxelizes_the_clean_cloud(tmp_path):
    pts = [[0.01, 0.01, 0.01], [0.05, 0.05, 0.05],    # two points of the same 0.20 m voxel: one prior point, at their centroid
           [1.0, 1.0, 1.0], [2.0, 2.0, 4.9],          # the ceiling of a tall room is kept (z <= 5)
           [3.0, 3.0, -0.6], [4.0, 4.0, 5.1],         # under -0.5 and over 5 m: cropped
           [5.0, 5.0, np.nan]]                        # not finite: cropped
    d = map_dir(tmp_path, pts, {"pipeline": 2})
    prepare_map.main(["--map-dir", str(d), "--out", str(tmp_path / "prior")])
    xyz = np.loadtxt(tmp_path / "prior/map.xyz")
    assert len(xyz) == 3 and np.allclose(sorted(xyz[:, 2]), [0.03, 1.0, 4.9], atol=1e-4)
    m = json.loads((tmp_path / "prior/map_manifest.json").read_text())
    assert (m["raw_points"], m["finite_cropped_points"], m["voxel_points"], m["voxel_m"]) == (7, 4, 3, 0.2)
    assert m["pipeline"] == 2 and m["frame"] == "map" and m["truth_used"] is False and m["map_dir"] == str(d)


@pytest.mark.parametrize("report", [None, {}, {"pipeline": 1}, {"resets": 0}])
def test_prepare_map_refuses_a_folder_without_pipeline_2(tmp_path, report):
    d = map_dir(tmp_path, [[0, 0, 0.5]], report)       # an old folder: its map_clean.ply is the band over occupied cells, no ceiling and no floor
    with pytest.raises(SystemExit) as e:
        prepare_map.main(["--map-dir", str(d), "--out", str(tmp_path / "prior")])
    assert "make_map.sh --reuse-slam" in str(e.value)
    assert not (tmp_path / "prior").exists()
