"""Post-run truth scoring. Keep this module out of executor/method containers."""
import argparse
from decimal import Decimal
import json
import math
from pathlib import Path

import numpy as np


def classify(accepted, xy_error_m=None, yaw_error_rad=None, negative=False):
    """Return the point-4 primary class from a completed attempt and evaluator truth."""
    if accepted:
        if xy_error_m is None or yaw_error_rad is None:
            raise ValueError('accepted trial needs evaluator-computed errors')
        if not math.isfinite(xy_error_m) or not math.isfinite(yaw_error_rad):
            raise ValueError('accepted trial errors must be finite')
        wrapped_yaw=math.atan2(math.sin(yaw_error_rad),math.cos(yaw_error_rad))
        return ('correct_accept' if xy_error_m<=.30 and abs(wrapped_yaw)<=math.radians(8)
                else 'wrong_accept')
    return 'correct_reject' if negative else 'false_reject_timeout'


def reference_at_ns(stamp_ns, times_ns, poses):
    """Interpolate evaluator truth only across a <=0.20 s bracket."""
    times=np.asarray(times_ns,dtype=np.int64)
    values=np.asarray(poses,dtype=float)
    if times.ndim!=1 or values.shape!=(len(times),3) or not len(times):
        raise ValueError('invalid reference arrays')
    i=int(np.searchsorted(times,stamp_ns))
    if i<len(times) and times[i]==stamp_ns:
        return tuple(float(v) for v in values[i])
    if i==0 or i==len(times):return None
    width=int(times[i]-times[i-1])
    if width<=0 or width>200_000_000:return None
    fraction=(stamp_ns-int(times[i-1]))/width
    delta=math.atan2(math.sin(values[i,2]-values[i-1,2]),
                     math.cos(values[i,2]-values[i-1,2]))
    return (float((1-fraction)*values[i-1,0]+fraction*values[i,0]),
            float((1-fraction)*values[i-1,1]+fraction*values[i,1]),
            math.atan2(math.sin(values[i-1,2]+fraction*delta),
                       math.cos(values[i-1,2]+fraction*delta)))


def score_observer(records,times_ns,poses,expected_scans,negative=False,
                   frontend_counts=None,expected_input_frames=None):
    """Score a completed observer log; missing scan-aligned updates invalidate it."""
    if expected_scans<1:raise ValueError('expected_scans must be positive')
    scored=[r for r in records if 'particle_concentration' in r]
    scan_rows=[r for r in records if r.get('input_kind') in ('fit','check')]
    accepts=[r['acceptance_event'] for r in records if 'acceptance_event' in r]
    complete=(len(scan_rows)==expected_scans and len(scored)==expected_scans
              and all(r.get('terminal_event')!='no_scan_aligned_amcl_update' for r in records))
    frontend_valid=None
    if frontend_counts is not None:
        if expected_input_frames is None:raise ValueError('expected_input_frames required with frontend counts')
        frontend_valid=(frontend_counts.get('window_lidar_received')==expected_input_frames
            and frontend_counts.get('fit_published',0)+frontend_counts.get('check_published',0)==expected_scans
            and frontend_counts.get('projection_error')==0
            and frontend_counts.get('insufficient_imu')==0
            and frontend_counts.get('excess_s1_frames',0)==0
            and (frontend_counts.get('conditioning') or {}).get('reason')=='ok')
        complete=complete and frontend_valid
    terminal=next((r['terminal_event'] for r in reversed(records) if r.get('terminal_event') in ('timeout','operational_failure')),None)
    result={'expected_scans':expected_scans,'observed_scans':len(scan_rows),
            'scored_scans':len(scored),'complete_stream':complete,
            'frontend_verified':frontend_valid,
            'max_concentration':max((float(r['particle_concentration']) for r in scored),default=None),
            'accepted':bool(accepts),'terminal_event':terminal,'outcome':None,
            'acceptance_errors':[],'final_hypothesis_error':None}
    for event in accepts:
        if negative:
            result['acceptance_errors'].append({'xy_m':None,'yaw_deg':None,
                                                'class':'wrong_accept'})
            continue
        truth=reference_at_ns(int(event['sensor_stamp_ns']),times_ns,poses)
        if truth is None:
            result['outcome']='invalid_unscorable';return result
        pose=event['pose']
        xy=math.hypot(pose[0]-truth[0],pose[1]-truth[1])
        yaw=math.atan2(math.sin(pose[2]-truth[2]),math.cos(pose[2]-truth[2]))
        result['acceptance_errors'].append({'xy_m':xy,'yaw_deg':abs(math.degrees(yaw)),
                                           'wall_elapsed_s':event.get('wall_elapsed_s'),
                                           'class':classify(True,xy,yaw)})
    if scored and not negative:
        final=scored[-1]
        truth=reference_at_ns(int(final['sensor_stamp_ns']),times_ns,poses)
        if truth is not None:
            pose=final['pose']
            result['final_hypothesis_error']={
                'xy_m':math.hypot(pose[0]-truth[0],pose[1]-truth[1]),
                'yaw_deg':abs(math.degrees(math.atan2(math.sin(pose[2]-truth[2]),
                                                     math.cos(pose[2]-truth[2]))))}
    if not complete or (not accepts and terminal!='timeout'):
        result['outcome']='invalid_unscorable'
    elif accepts:
        result['outcome']=('wrong_accept' if any(e['class']=='wrong_accept' for e in result['acceptance_errors'])
                           else 'correct_accept')
    else:
        result['outcome']=classify(False,negative=negative)
    return result


def main():
    parser=argparse.ArgumentParser(description='Evaluator-only score for a completed AMCL development trial')
    parser.add_argument('observer_jsonl',type=Path)
    parser.add_argument('reference',type=Path)
    parser.add_argument('map_yaml',type=Path)
    parser.add_argument('--expected-scans',type=int,required=True)
    parser.add_argument('--expected-input-frames',type=int)
    parser.add_argument('--frontend-log',type=Path)
    args=parser.parse_args()
    from pose_eval import read_map_yaml,reference_map
    _,poses=reference_map(args.reference,read_map_yaml(args.map_yaml)['T_map_world'])
    with args.reference.open(encoding='utf-8') as stream:
        times=[int(Decimal(line.split()[0])*1_000_000_000)
               for line in stream if line.strip() and not line.startswith('#')]
    records=[json.loads(line) for line in args.observer_jsonl.read_text().splitlines() if line.strip()]
    frontend=None
    if args.frontend_log is not None:
        marker='scan diagnostics '
        diagnostics=[json.loads(line.split(marker,1)[1])
                     for line in args.frontend_log.read_text().splitlines() if marker in line]
        if not diagnostics:raise ValueError('frontend log has no scan diagnostics')
        frontend=diagnostics[-1]
    result=score_observer(records,times,poses,args.expected_scans,
                          frontend_counts=frontend,expected_input_frames=args.expected_input_frames)
    result['classification_policy']='EXPLORATORY — POST-RUN EVALUATOR ONLY'
    print(json.dumps(result,sort_keys=True,allow_nan=False))


if __name__=='__main__':main()
