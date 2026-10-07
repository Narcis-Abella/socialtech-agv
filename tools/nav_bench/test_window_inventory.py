"""Point-5 synthetic contract tests. No ROS, candidate or real reference needed."""
import gzip
import struct
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
import window_inventory as w

class Contracts(unittest.TestCase):
    def test_decimal_epoch(self):
        self.assertEqual(w.ns('1780999550.039071'), 1780999550039071000)
        with self.assertRaises(ValueError): w.ns('1.0000000001')

    def test_reference_exact_gap_and_no_substitutes(self):
        ref = w.Reference([0,200000000,500000001], [[0,0,0],[2,0,0],[5,0,0]], [[0,0,0,1]]*3)
        self.assertAlmostEqual(ref.at(100000000)[0][0],1)
        self.assertEqual(ref.at(500000001)[0][0],5)
        for t in [-1,200000001,500000002]:
            with self.assertRaises(ValueError): ref.at(t)
        with self.assertRaises(ValueError): ref.segment(0,500000001)

    def test_gap_equality_duplicates_and_regressions(self):
        a=w.cadence([0,10,20,30,40,55,71,71,70])
        self.assertEqual(a['median_ns'],10)
        self.assertEqual(a['gaps'],[[55,71]])
        self.assertEqual(a['duplicates'],1)
        self.assertEqual(a['regressions'],1)
        self.assertFalse(w.crosses([[10,20]],20,30))
        self.assertTrue(w.crosses([[10,20]],11,21))

    def test_quaternion_sign_and_interpolation(self):
        r=w.Reference([0,100000000],[[0,0,0]]*2,[[0,0,0,1],[0,0,0,-1]])
        np.testing.assert_allclose(abs(r.at(50000000)[1]),[0,0,0,1])

    def test_imu_units_and_variance(self):
        values=np.array([[0,0,0,0,0,1],[0,0,0,0,0,1.1]])
        s=w.imu_stats(values,9.80665)
        self.assertAlmostEqual(s['accel_variance_sum_m2_s4'],.005*9.80665**2)
        self.assertFalse(w.imu_pass(s))
        self.assertTrue(w.imu_pass(w.imu_stats(np.array([[0,0,0,0,0,1]]*10),9.80665)))

    def test_groups_equality_chain_and_revisit(self):
        self.assertEqual(w.components([[0,0],[4,0],[8,0],[13,0]]),[0,0,0,3])

    def test_firewall(self):
        from window_contract import execution_config, validate_method_packet
        c=execution_config('S3','primary')
        self.assertEqual(c['operational_radius_m'],2.8)
        for name in ['reference_csv','split','evaluator_only_prefix_end_ns','stationarity_level','t_input_end_ns','initial_pose']:
            with self.assertRaises(ValueError): validate_method_packet({name:0})
        validate_method_packet({'kind':'imu','sensor_stamp_ns':123,'payload':b'raw'})
        with self.assertRaises(ValueError): execution_config('S3','primary', split='final')


class Inventory(unittest.TestCase):
    def fixture(self,root,imu_gap=False):
        b=root/'hospital_02_r05b'; b.mkdir()
        lidar=[[t+100000000,t,t,0,99999999,99999999,10] for t in range(0,41*w.NS,100000000)]
        imu=[[t,t,0,0,0,0,0,1] for t in range(0,42*w.NS,5000000) if not (imu_gap and 10*w.NS<t<11*w.NS)]
        d={'lidar':lidar,'imu':imu,'sources':[],'frame_ids':{},'topics':[]}
        (b/'sensor-index.json.gz').write_bytes(gzip.compress(w.canonical(d),mtime=0))
        (b/'alidarState.txt').write_text(''.join(f'{w.seconds(t)} 0 0 0 0 0 0 1\n' for t in range(0,42*w.NS,100000000)))
        (b/'map.yaml').write_text('# T_map_world\n# [1,0,0,0]\n# [0,1,0,0]\n# [0,0,1,0]\n# [0,0,0,1]\n')
        (b/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')

    def test_real_generator_on_synthetic_intact_data(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); self.fixture(root)
            rows,excluded,audit=w.generate_dataset(root,'hospital_02_r05b')
            rows2,e2,a2=w.generate_dataset(root,'hospital_02_r05b')
            self.assertTrue(rows)
            self.assertEqual(w.canonical(rows),w.canonical(rows2))
            self.assertEqual(excluded,e2)
            self.assertEqual(w.canonical(audit),w.canonical(a2))
            self.assertEqual(w.serialize(rows),w.serialize(rows2))
            self.assertEqual({r['split'] for r in rows},{'heldout'})
            self.assertEqual({r['independent_block_count'] for r in rows},{1})
            self.assertTrue(all(r['eligibility_status']=='insufficient_independent_coverage' for r in rows))
            self.assertEqual(len({r['window_id'] for r in rows}),len(rows))
            for r in rows:
                if r['scenario']=='S1': self.assertEqual(r['scan_count'],1)
                if r['scenario']=='S3':
                    self.assertIsNone(r['t_input_end_ns'])
                    self.assertEqual(r['runtime_status'],'pending_lio_and_physical_postcheck')
            for lev in ('A','B'):
                for scenario,variant in [('S1','primary'),('S2','primary'),('S3','primary')]:
                    rr=sorted([r for r in rows if (r['stationarity_level'],r['scenario'],r['variant_id'])==(lev,scenario,variant)],key=lambda r:r['t_input_start_ns'])
                    for x,y in zip(rr,rr[1:]):
                        self.assertLessEqual(x['evaluator_only_support_end_ns'],y['evaluator_only_support_start_ns'])

    def test_gap_exclusion_and_reconditioning(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d); self.fixture(root,True)
            rows,excluded,_=w.generate_dataset(root,'hospital_02_r05b')
            self.assertTrue(any(e[-1]=='imu_gap' for e in excluded))
            for r in rows:
                if r['scenario']!='S3':
                    self.assertFalse(w.crosses([[10*w.NS,11*w.NS]],r['t_imu_start_ns'],r['t_input_end_ns']))
                    if r['t_input_start_ns']>=11*w.NS: self.assertGreaterEqual(r['t_input_start_ns'],13*w.NS)

    def test_empty_csv_has_schema_and_domains(self):
        data,cols=w.serialize([])
        self.assertIn(b'window_id',data)
        self.assertIn('executor_stop_reason',cols)
        self.assertEqual(len(data.splitlines()),1)
        with self.assertRaises(ValueError): w.serialize([{'unlisted':1}])

    def test_split_rounding_revisits_and_exact_guard(self):
        def row(i,x,t,end):
            return dict(window_id=str(i),evaluator_only_ref_x_m=x,evaluator_only_ref_y_m=0,
                        t_input_start_ns=t,_support_start=t,_support_end=end,eligibility_status='eligible',exclusion_reason='')
        rows=[row(0,0,0,w.NS),row(1,10,36*w.NS,37*w.NS),row(2,20,80*w.NS,81*w.NS)]
        w.group_split(rows,'eco01_1x')
        self.assertEqual([r['split'] for r in rows],['tuning','final','final'])
        self.assertEqual(rows[1]['exclusion_reason'],'')  # exactly 35 s
        rows=[row(0,0,0,w.NS),row(1,10,35*w.NS,36*w.NS),row(2,20,80*w.NS,81*w.NS)]
        w.group_split(rows,'eco01_1x')
        self.assertEqual(rows[1]['exclusion_reason'],'temporal_guard')
        rows.append(row(3,0,200*w.NS,201*w.NS)); w.group_split(rows,'eco01_1x')
        self.assertEqual(rows[0]['spatial_block_id'],rows[3]['spatial_block_id'])
        self.assertEqual(rows[3]['split'],'tuning')

    def test_cdr_readers(self):
        from window_sensor_index import custom_timing,imu_values
        frame=b'livox_frame\0'; head=b'\0\1\0\0'+struct.pack('<iII',1,2,len(frame))+frame
        pts=b''.join(struct.pack('<IfffBBBB',t,1,2,3,4,5,6,0) for t in [0,99000000])
        raw=head+struct.pack('<QIB3sI',1000000002,2,0,b'\0'*3,2)+pts
        vals,f=custom_timing(raw)
        self.assertEqual(vals,[1000000002,1000000002,0,99000000,99000000,2])
        self.assertEqual(f,'livox_frame')
        self.assertEqual(custom_timing(raw[:-1])[0],vals)
        with self.assertRaises(ValueError): custom_timing(raw[:-2])
        data=[0.]*37; data[13:16]=[1,2,3]; data[25:28]=[4,5,6]
        stamp,values,_=imu_values(head+struct.pack('<37d',*data))
        self.assertEqual(values,[1,2,3,4,5,6]); self.assertEqual(stamp,1000000002)

if __name__=='__main__': unittest.main()
