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


class BatchEqualsLoneRuns(unittest.TestCase):
    """--init-list must give, for every matrix, exactly what a lone --init run gives (same preprocessing)."""
    KEYS=('gicp_converged','gicp_iterations','gicp_inliers','gicp_error','T_map_lidar_refined','T_map_lidar_kiss',
          'source_points','target_points','check_points','config')

    def test_every_result_is_the_lone_run_result(self):
        binary=os.environ.get('REGISTER_BINARY')
        if not binary:
            self.skipTest('set REGISTER_BINARY to the built aarch64 caller')
        rng=np.random.default_rng(3)
        n=1500
        target=np.vstack([np.column_stack([rng.uniform(-6,6,n),rng.uniform(-4,4,n),rng.normal(0,.01,n)]),
                          np.column_stack([np.full(n,6.),rng.uniform(-4,4,n),rng.uniform(0,3,n)]),
                          np.column_stack([rng.uniform(-6,6,n),np.full(n,-4.),rng.uniform(0,3,n)])])
        source=target[rng.choice(len(target),2000,replace=False)]-[1.,.5,0.]+rng.normal(0,.005,(2000,3))
        inits=[]
        for dx,dyaw in [(0.,0.),(.3,5.),(-.4,-8.),(.1,15.)]:
            th=np.deg2rad(dyaw)
            T=np.eye(4); T[:2,:2]=[[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]]; T[:3,3]=[1.+dx,.5,0.]
            inits.append(T)
        with tempfile.TemporaryDirectory() as d:
            d=Path(d)
            for name,pts in [('map',target),('fit',source),('check',source)]:
                np.savetxt(d/(name+'.xyz'),pts,fmt='%.6f')
            np.savetxt(d/'list.txt',np.array([T.ravel() for T in inits]))
            (d/'batch').mkdir()
            args=[str(d/'map.xyz'),str(d/'fit.xyz'),str(d/'check.xyz'),'.30']
            proc=subprocess.run([binary,*args,str(d/'batch'),'--init-list',str(d/'list.txt')],capture_output=True,text=True,timeout=60)
            self.assertEqual(proc.returncode,0,proc.stdout+'\n'+proc.stderr)
            for i,T in enumerate(inits):
                np.savetxt(d/f'init{i}.txt',T)
                proc=subprocess.run([binary,*args,str(d/f'lone{i}.json'),'--init',str(d/f'init{i}.txt')],capture_output=True,text=True,timeout=60)
                self.assertEqual(proc.returncode,0,proc.stdout+'\n'+proc.stderr)
                batch=json.loads((d/'batch'/f'{i:02d}.json').read_text())
                lone=json.loads((d/f'lone{i}.json').read_text())
                self.assertEqual({k:batch[k] for k in self.KEYS},{k:lone[k] for k in self.KEYS},f'init {i}')
            self.assertEqual(len(list((d/'batch').glob('*.json'))),len(inits))

    def test_a_bad_list_fails_without_results(self):
        binary=os.environ.get('REGISTER_BINARY')
        if not binary:
            self.skipTest('set REGISTER_BINARY to the built aarch64 caller')
        with tempfile.TemporaryDirectory() as d:
            d=Path(d)
            for name in ('map','fit','check'):
                np.savetxt(d/(name+'.xyz'),np.random.default_rng(1).uniform(-3,3,(200,3)),fmt='%.6f')
            (d/'list.txt').write_text('1 0 0 0 0 1 0 0\n')    # half a matrix
            (d/'batch').mkdir()
            proc=subprocess.run([binary,str(d/'map.xyz'),str(d/'fit.xyz'),str(d/'check.xyz'),'.30',str(d/'batch'),'--init-list',str(d/'list.txt')],
                                capture_output=True,text=True,timeout=60)
            self.assertNotEqual(proc.returncode,0)
            self.assertEqual(list((d/'batch').glob('*')),[])


if __name__=='__main__':
    unittest.main()
