import unittest
import numpy as np
import tempfile
from pathlib import Path
from search_tiles import distinct, agreement, temporal_evidence, run_one


class SearchPolicyTest(unittest.TestCase):
    def test_distinct_and_agreement(self):
        a = np.eye(4)
        b = np.eye(4)
        b[0, 3] = .21
        self.assertFalse(distinct(a, b))
        self.assertFalse(agreement(a, b))
        b[0, 3] = .31
        self.assertTrue(distinct(a, b))
        c = np.eye(4)
        self.assertTrue(agreement(a, c))

    def test_one_second_ablation_cannot_accept(self):
        self.assertFalse(temporal_evidence([.0,.1,.2,.3,.4,.5,.6,.7,.8,.9],0.,1.)[0])
        self.assertTrue(temporal_evidence([.0,.5,1.,1.5,2.,2.5,2.9],0.,3.)[0])

    def test_single_matcher_call_has_deadline(self):
        with tempfile.TemporaryDirectory() as d:
            sleeper=Path(d)/'sleeper'
            sleeper.write_text('#!/bin/sh\nsleep 2\n')
            sleeper.chmod(0o755)
            self.assertEqual(run_one(sleeper,'a','b','c',.3,Path(d)/'result',.02)['exit_code'],124)


if __name__ == '__main__':
    unittest.main()
