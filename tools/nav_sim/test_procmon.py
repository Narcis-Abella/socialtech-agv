import csv
import json
import os
import subprocess
import sys
import tempfile
import unittest

import procmon

HDR = ['wall', 'pid', 'ppid', 'ticks', 'child_ticks', 'rss_kb', 'hwm_kb', 'cmd']


class ProcmonTest(unittest.TestCase):
    def csv(self, rows):
        f = tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False, newline='')
        csv.writer(f).writerows([HDR] + rows)
        f.close()
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_groups(self):
        self.assertEqual(procmon.group('/x/fastlio_mapping --ros-args'), 'fastlio')
        self.assertEqual(procmon.group('python3 /tools/nav_sim/sup_cover.py'), 'sim')
        self.assertEqual(procmon.group('/x/livox_mid360_sim'), 'sim')
        self.assertEqual(procmon.group('python3 gn_supervisor_node.py'), 'supervisor')
        self.assertEqual(procmon.group('sleep 1'), 'other')

    def test_cpu_and_reaped_children(self):
        t = procmon.TICK
        p = self.csv([[0, 1, 0, 0, 0, 1024, 2048, 'fastlio_mapping'], [1, 1, 0, t // 2, 0, 2048, 4096, 'fastlio_mapping'],
                      [0, 2, 0, 0, 0, 1024, 1024, 'python3 search_v2.py'], [1, 2, 0, 0, t, 3072, 3072, 'python3 search_v2.py'],
                      [0, 3, 0, 0, 0, 1024, 1024, 'gzserver'], [1, 3, 0, 5 * t, 0, 9999, 9999, 'gzserver']])
        cpu, rss = procmon.stack_total(procmon.series(p))
        self.assertAlmostEqual(cpu[1], 50. + 100.)        # fastlio half a core + the worker's reaped register_v2; the simulator is out
        self.assertAlmostEqual(rss[1], 2. + 3.)

    def test_sampler_sees_itself_children(self):
        out = tempfile.mktemp(suffix='.csv')
        self.addCleanup(lambda: os.path.exists(out) and os.unlink(out))
        p = subprocess.Popen([sys.executable, procmon.__file__, 'sample', out, '0.2'])
        sl = subprocess.Popen(['sleep', '1.5'])
        sl.wait()
        p.terminate()
        p.wait()
        self.assertTrue(any('sleep 1.5' in r['cmd'] for r in csv.DictReader(open(out))))

    def test_phases(self):
        ev = [{'what': 'state', 'state': 'MONITOR', 'wall': 10}, {'what': 'cover', 'wall': 20}, {'what': 'uncover', 'wall': 40},
              {'what': 'state', 'state': 'RELOCALIZING', 'wall': 41}, {'what': 'state', 'state': 'MONITOR', 'wall': 50}]
        f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
        json.dump({'events': ev}, f)
        f.close()
        self.addCleanup(os.unlink, f.name)
        self.assertEqual([(n, a, b) for n, a, b in procmon.phases(f.name)][1:4], [('monitor', 10, 20), ('covered', 20, 40), ('reloc_after_uncover', 40, 50)])


if __name__ == '__main__':
    unittest.main()
