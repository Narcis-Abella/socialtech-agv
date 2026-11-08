import json
import math
import os
import subprocess
import sys
import tempfile
import textwrap
import threading
import unittest
from pathlib import Path

import numpy as np

from correlative_search import pose
from search_v2 import rank, run_concurrently, source_reach_z, tilt_deg


def row(i, support, T):
    return {'id': i, 'geometry': {'valid': True, 'support_015': support, 'matched_rmse_m': .05},
            'result': {'T_map_lidar_refined': T.tolist()}}


class SearchV2Test(unittest.TestCase):
    def test_reach_z_of_an_empty_crop_is_zero_instead_of_a_numpy_error(self):
        near = np.array([[1., 0., .5], [2., 0., -1.5]])
        far = np.array([[100., 0., 3.]])                              # outside the 15 m search radius: the crop is empty
        self.assertEqual(source_reach_z(near, far, 15.), 1.5)
        self.assertEqual(source_reach_z(far, far, 15.), 0.)           # a near-empty LiDAR frame: the search then rejects it cleanly

    def test_rank_keeps_distinct_hypotheses_only(self):
        a = row('a', .9, pose(0., [0, 0, 0]))
        same = row('same', .8, pose(math.radians(2), [.1, 0, 0]))  # within 0.30 m / 8 deg of a
        far = row('far', .7, pose(0., [5, 0, 0]))
        self.assertEqual([r['id'] for r in rank([far, same, a])], ['a', 'far'])

    def test_the_two_searches_run_at_the_same_time_and_both_results_come_back(self):
        barrier = threading.Barrier(2, timeout=5)                     # each waits for the other: a sequential run would break the barrier

        def search(name):
            barrier.wait()
            return name
        self.assertEqual(run_concurrently(lambda: search('primary'), lambda: search('confirmation')), ('primary', 'confirmation'))

    def test_an_error_in_either_search_is_raised_not_swallowed(self):
        def boom():
            raise ValueError('the search failed')
        with self.assertRaisesRegex(ValueError, 'the search failed'):
            run_concurrently(lambda: 1, boom)
        with self.assertRaisesRegex(ValueError, 'the search failed'):
            run_concurrently(boom, lambda: 1)

    def test_tilt(self):
        self.assertAlmostEqual(tilt_deg(pose(1., [0, 0, 0])), 0., places=6)
        T = np.eye(4)
        c, s = math.cos(math.radians(10)), math.sin(math.radians(10))
        T[1:3, 1:3] = [[c, -s], [s, c]]
        self.assertAlmostEqual(tilt_deg(T), 10., places=6)


class WorkerExitCodeTest(unittest.TestCase):
    """The supervisor decides from the exit code whether a new attempt can help (gn_supervisor_core.is_deterministic)."""
    def worker(self, manifest_frame='local_first_imu_levelled', backend=('--corr-backend', 'python'), points=(5, 5), with_map=True, **env):
        d = tempfile.TemporaryDirectory(); self.addCleanup(d.cleanup)
        run = Path(d.name) / 'in'; run.mkdir()
        rng = np.random.default_rng(0)
        if with_map:
            np.savetxt(run / 'map.xyz', rng.uniform(0., 20., (500, 3)))
        for name, n in zip(('fit', 'check'), points):                         # 5: far under the 50 points of a search
            np.savetxt(run / f'{name}.xyz', rng.uniform(-1., 1., (n, 3)))
        (run / 'manifest.json').write_text(json.dumps({'frame_fit': manifest_frame, 'stamps_s': [0., 3.], 'start_header_s': 0., 'duration_s': 3.}))
        cmd = [sys.executable, '-B', str(Path(__file__).parent / 'search_v2.py'), '--run', str(run), '--out', str(Path(d.name) / 'o'), '--binary', '/nonexistent', *backend]
        return subprocess.run(cmd, capture_output=True, text=True, env={**os.environ, **env})

    def test_a_scene_too_sparse_can_be_retried(self):
        self.assertEqual(self.worker().returncode, 3)

    def test_a_batch_of_one_point_or_none_is_just_as_sparse(self):
        for points in ((1, 200), (200, 1), (0, 200), (200, 0), (0, 0)):
            p = self.worker(points=points)
            self.assertEqual(p.returncode, 3, (points, p.stderr[-200:]))

    def test_sparseness_is_found_before_the_map_is_loaded(self):
        # it is retried: every attempt must not pay for the map and the field of a scene that cannot be searched
        self.assertEqual(self.worker(with_map=False).returncode, 3)

    def test_bad_input_cannot(self):
        self.assertEqual(self.worker(manifest_frame='other').returncode, 1)

    def test_cpp_is_the_default_and_a_missing_library_is_no_silent_python(self):
        p = self.worker(backend=(), points=(200, 200), CORR_SEARCH_LIB='/nonexistent/libcorr_search.so')
        self.assertEqual(p.returncode, 1, p.stderr)
        self.assertIn('libcorr_search', p.stderr)



class ZBandRankTest(unittest.TestCase):
    def row(self, z):
        T = np.eye(4); T[2, 3] = z
        return {'geometry': {'valid': True, 'support_015': .9, 'matched_rmse_m': .05}, 'result': {'T_map_lidar_refined': T.tolist()}}

    def test_a_refined_pose_outside_the_band_is_not_ranked(self):
        import search_v2 as sv
        rows = [self.row(.5), self.row(1.9)]
        self.assertTrue(sv.in_band(rows[0], (.2, .9)))
        self.assertFalse(sv.in_band(rows[1], (.2, .9)))
        self.assertTrue(sv.in_band(rows[1], None))
        self.assertEqual([r['result']['T_map_lidar_refined'][2][3] for r in sv.rank(rows, (.2, .9))], [.5])


FAKE_REGISTER = textwrap.dedent("""\
    #!/usr/bin/env python3
    import json, sys, time
    from pathlib import Path
    a = sys.argv
    assert a[6] == '--init-list', a
    rows = [r for r in Path(a[7]).read_text().splitlines() if r.strip()]
    for i, r in enumerate(rows):
        if i == 1:
            continue                      # an init that failed leaves no file
        if i == 3:
            time.sleep(30)                # still running when the deadline comes
        Path(a[5], f'{i:02d}.json').write_text(json.dumps({'gicp_converged': True, 'first': float(r.split()[0])}))
""")


class RefineAllTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = Path(self.tmp.name)
        self.binary = self.out / 'fake_register'
        self.binary.write_text(FAKE_REGISTER)
        self.binary.chmod(0o755)
        self.T = [np.eye(4) * k for k in (1., 2., 3., 4.)]

    def tearDown(self):
        self.tmp.cleanup()

    def run_it(self, n, timeout_s):
        import search_v2 as sv
        return sv.refine_all(self.binary, 'map', 'src', 'chk', self.T[:n], self.out, 'cand', .3, timeout_s)

    def test_each_result_lands_where_a_lone_run_would_put_it(self):
        log, results, timed_out = self.run_it(3, 20)
        self.assertEqual((log['exit_code'], timed_out), (0, False))
        self.assertEqual([r and r['first'] for r in results], [1., None, 3.])
        self.assertEqual(json.loads((self.out / 'cand_02' / 'result.json').read_text())['first'], 3.)
        self.assertFalse((self.out / 'cand_01').exists())

    def test_a_timeout_keeps_what_was_finished(self):
        log, results, timed_out = self.run_it(4, 1)
        self.assertEqual((log['exit_code'], timed_out), (124, True))
        self.assertEqual([r and r['first'] for r in results], [1., None, 3., None])



class GlobalHypothesesDeadlineTest(unittest.TestCase):
    """What the search keeps when the GICP batch runs out of time, and that it does not even start with no time left."""
    def setUp(self):
        import argparse
        import search_v2 as sv
        from unittest import mock
        self.sv = sv
        self.args = argparse.Namespace(res=.5, yaw_step=2.5, radius=15., top=4, binary='b', gicp_voxel=.3, threads=2)
        cands = [{'score': 1., 'yaw_rad': 0., 'xyz': [0., 0., 0.], 'T_map_source': np.eye(4).tolist()} for _ in range(4)]
        self.patches = [mock.patch.object(sv, 'search', return_value={'candidates': cands}),
                        mock.patch.object(sv, 'score', return_value={'valid': True, 'support_015': .9, 'matched_rmse_m': .05}),
                        mock.patch.object(sv, 'map_tree', return_value=None),
                        mock.patch.object(sv, 'DEADLINE_S', 60.)]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)
        self.ok = {'gicp_converged': True, 'T_map_lidar_refined': np.eye(4).tolist()}

    def run_it(self, results, timed_out, t0_offset=0.):
        import time
        from unittest import mock
        with mock.patch.object(self.sv, 'refine_all', return_value=({'exit_code': 0}, results, timed_out)) as ra:
            out = self.sv.global_hypotheses(None, None, None, 'b', None, 'o', 'm', self.args, None, 'cand', time.monotonic() - t0_offset)
        return out, ra

    def test_finished_candidates_survive_a_batch_timeout_even_after_a_failed_init(self):
        (_, rows, timed_out), _ = self.run_it([dict(self.ok), None, dict(self.ok), None], True)
        self.assertTrue(timed_out)
        self.assertEqual([r['id'] for r in rows], ['cand_00', 'cand_02'])

    def test_without_a_timeout_a_failed_init_is_a_row_without_result(self):
        (_, rows, timed_out), _ = self.run_it([dict(self.ok), None, dict(self.ok), dict(self.ok)], False)
        self.assertFalse(timed_out)
        self.assertEqual([('result' in r) for r in rows], [True, False, True, True])

    def test_no_time_left_starts_no_process(self):
        (_, rows, timed_out), ra = self.run_it([], False, t0_offset=61.)
        self.assertEqual((rows, timed_out), ([], True))
        ra.assert_not_called()


class LoneRefineTimeoutTest(unittest.TestCase):
    def test_a_timed_out_lone_run_has_no_result_even_if_a_file_exists(self):
        import search_v2 as sv
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            b = d / 'slow'
            b.write_text('#!/bin/sh\necho "{" > "$5"\nsleep 30\n')
            b.chmod(0o755)
            log, result = sv.refine(b, 'm', 's', 'c', np.eye(4), d / 'o', .3, 1)
        self.assertEqual((log['exit_code'], result), (124, None))


class StaleBinaryTest(unittest.TestCase):
    """A register_v2 that does not know --init-list (staged before the tools) must stop the search loudly: 'no candidate survived' would make the supervisor retry for nothing."""
    def test_a_binary_that_fails_without_any_result_raises_with_its_message(self):
        import search_v2 as sv
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            b = d / 'old'
            b.write_text('#!/bin/sh\necho "unknown option --init-list" >&2\nexit 1\n')
            b.chmod(0o755)
            with self.assertRaisesRegex(RuntimeError, 'unknown option --init-list'):
                sv.refine_all(b, 'm', 's', 'c', [np.eye(4)], d, 'cand', .3, 20)

    def test_one_failed_init_among_results_is_not_an_error(self):
        import search_v2 as sv
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            b = d / 'fake'
            b.write_text(FAKE_REGISTER)
            b.chmod(0o755)
            log, results, _ = sv.refine_all(b, 'm', 's', 'c', [np.eye(4)] * 3, d, 'cand', .3, 20)
        self.assertEqual([r is not None for r in results], [True, False, True])

    def test_every_init_failing_is_no_hypothesis_not_a_broken_binary(self):
        import search_v2 as sv
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            b = d / 'failing'
            b.write_text('#!/bin/sh\necho "init 0: degenerate" >&2\nexit 1\n')
            b.chmod(0o755)
            log, results, timed_out = sv.refine_all(b, 'm', 's', 'c', [np.eye(4)] * 2, d, 'cand', .3, 20)
        self.assertEqual((results, timed_out, log['exit_code']), ([None, None], False, 1))


if __name__ == '__main__':
    unittest.main()
