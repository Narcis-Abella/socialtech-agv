"""Run on NX with REGISTER_BINARY pointing to the built aarch64 caller."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np


class SyntheticRegistration(unittest.TestCase):
    def test_global_transform_without_seed(self):
        binary=os.environ.get('REGISTER_BINARY')
        if not binary:
            self.skipTest('set REGISTER_BINARY to the built aarch64 caller')
        rng=np.random.default_rng(7)
        n=1200
        floor=np.column_stack([rng.uniform(-5,5,n),rng.uniform(-4,4,n),rng.normal(0,.01,n)])
        wall_x=np.column_stack([np.full(n,5.),rng.uniform(-4,4,n),rng.uniform(0,3,n)])
        wall_y=np.column_stack([rng.uniform(-5,5,n),np.full(n,-4.),rng.uniform(0,3,n)])
        pillar=np.column_stack([1.2+.3*np.cos(rng.uniform(0,2*np.pi,n)),
                                 1.1+.3*np.sin(rng.uniform(0,2*np.pi,n)),rng.uniform(0,3,n)])
        target=np.vstack([floor,wall_x,wall_y,pillar])
        theta=np.deg2rad(33.)
        R=np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
        t=np.array([4.,-3.,.5])
        source=(target-t)@R + rng.normal(0,.005,target.shape)
        with tempfile.TemporaryDirectory() as d:
            d=Path(d)
            for name,pts in [('map',target),('fit',source),('check',source)]:
                np.savetxt(d/(name+'.xyz'),pts,fmt='%.6f')
            proc=subprocess.run([binary,str(d/'map.xyz'),str(d/'fit.xyz'),str(d/'check.xyz'),'.30',str(d/'result.json')],
                                capture_output=True,text=True,timeout=30)
            self.assertEqual(proc.returncode,0,proc.stdout+'\n'+proc.stderr)
            result=json.loads((d/'result.json').read_text())
            self.assertTrue(result['kiss_valid'])
            self.assertTrue(result['gicp_converged'])
            T=np.array(result['T_map_lidar_refined'])
            self.assertLess(np.linalg.norm(T[:3,3]-t),.10)
            self.assertLess(np.linalg.norm(T[:3,:3]-R),.05)


if __name__=='__main__':
    unittest.main()
