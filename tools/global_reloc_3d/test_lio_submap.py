import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from lio_submap import interpolate, rebase, level_rotation, gravity_up, s3_limit


class LioJoinTest(unittest.TestCase):
    def test_rebase_and_timestamp_guard(self):
        T0=np.eye(4); T0[0,3]=10
        T1=np.eye(4); T1[0,3]=11
        p=np.array([[1.,2.,3.]])
        self.assertTrue(np.allclose(rebase(p,T0,T0),p))
        self.assertTrue(np.allclose(rebase(p,T0,T1),p+[1,0,0]))
        self.assertTrue(np.allclose(interpolate([(1.,T0),(1.1,T1)],1.05)[:3,3],[10.5,0,0]))
        self.assertRaises(ValueError,interpolate,[(1.,T0),(1.3,T1)],1.15)

    def test_level_rotation_adds_no_yaw(self):
        R_tilt=Rotation.from_euler('xyz',[4,-3,0],degrees=True).as_matrix()
        up=R_tilt.T@[0,0,1.]  # +z of the gravity frame seen in the tilted sensor frame
        R=level_rotation(up)
        self.assertTrue(np.allclose(R@up,[0,0,1],atol=1e-12))
        self.assertTrue(np.allclose(R.T@R,np.eye(3),atol=1e-12))
        x=R@[1.,0,0]  # the sensor x axis keeps its heading (tilt only)
        self.assertLess(abs(np.degrees(np.arctan2(x[1],x[0]))),0.2)
        self.assertTrue(np.allclose(level_rotation([0,0,2.]),np.eye(3)))
        self.assertRaises(ValueError,level_rotation,[0,0,-1.])

    def test_gravity_up_rotates_samples_into_first_frame(self):
        # Body yawed 90 deg and tilted after start: samples must be rotated back.
        first=np.eye(4)
        later=np.eye(4); later[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
        odom=[(t,first if t<1 else later) for t in np.arange(0,2.01,.1)]
        up_world=np.array([np.sin(.05),0,np.cos(.05)])
        imu=[(t,(first if t<1 else later)[:3,:3].T@up_world) for t in np.linspace(0,1.99,200) if abs(t-1)>.1]
        up,n,spread,skipped=gravity_up(imu,odom,first,0.,2.)
        self.assertTrue(np.allclose(up,up_world,atol=1e-6))
        self.assertGreater(n,150)
        self.assertLess(spread,1e-3)
        self.assertEqual(skipped,0)
        gap=[(t,T) for t,T in odom if not .4<t<.7]  # 0.3 s odometry gap: samples there are skipped, not fatal
        up,n,spread,skipped=gravity_up(imu,gap,first,0.,2.)
        self.assertGreater(skipped,0)
        self.assertTrue(np.allclose(up,up_world,atol=1e-6))

    def test_s3_limit_uses_own_odometry(self):
        poses=[]
        for x in np.arange(0,4,.5):
            T=np.eye(4); T[0,3]=x; poses.append(T)
        n,reason,path,radius=s3_limit(list(range(len(poses))),poses,2.8,6.)
        self.assertEqual((n,reason),(6,'radius'))
        self.assertAlmostEqual(radius,2.5)
        back=[]
        for x in [0,2,0,2,0,2,0,2]:
            T=np.eye(4); T[0,3]=x; back.append(T)
        self.assertEqual(s3_limit(list(range(8)),back,2.8,6.)[:2],(4,'path'))
        self.assertEqual(s3_limit([0,1],back[:1]*2,2.8,6.)[:2],(2,'duration'))


if __name__=='__main__':
    unittest.main()
