"""Evaluator-only reproducible real-observation inventory, protocol point 5.

python3 -B window_inventory.py --input-root INPUT --output NEW_DIRECTORY
INPUT/{eco01_1x,hospital_02_r05b}/ contains sensor-index.json.gz,
alidarState.txt, map.yaml and map.pgm. No candidate/ROS/odometry is executed.
Private output is NEVER a method/executor input. See window_contract.py.
"""
import argparse
import csv
from decimal import Decimal
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys
from collections import Counter
import numpy as np

VERSION='1.0'
PROTOCOL='31cd64ff0034072a76699546ab47281a47359871'
DATASETS=('eco01_1x','hospital_02_r05b')
NS=10**9
GAP=200_000_000
CONFIG={'schema_version':1,'generator_version':VERSION,'protocol_commit':PROTOCOL,'seed':None,
        'levels':{'A':[.02,1.],'B':[.05,2.]},'gyro_p95':.02,'gyro_max':.05,'accel_variance_sum':.04,
        'accel_scale':9.80665,'conditioning_ns':2*NS,'gap_factor':[3,2],'reference_gap_ns':GAP,
        'spatial_m':5.,'guard_ns':35*NS,'minimum_blocks':30,'target_starts':100,
        's2_durations_ns':[NS,3*NS,5*NS],'s3_acquisition_ns':30*NS,'s3_radius_m':2.8,'s3_path_m':6.,
        'interval':'[start,end)','quantile':'linear','variance_ddof':1,
        's3_continuous_artifact':None,'reference_offset_ns':0,'units_evidence':'Livox native g/rad_s; installed lddc + converter; report 04',
        'reference_time_convention':'filtered scan end; no empirical offset correction'}


def canonical(x): return json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def digest(x): return hashlib.sha256(x).hexdigest()
def ns(s):
    v=Decimal(s)*NS
    if not v.is_finite() or v!=v.to_integral_value(): raise ValueError('Timestamp not representable in ns')
    return int(v)
def seconds(t): return None if t is None else f'{t//NS}.{t%NS:09d}'


def cadence(times):
    d=np.diff(np.asarray(times,dtype=np.int64)); positive=d[d>0]
    if not len(positive): raise ValueError('No positive cadence')
    median=float(np.median(positive))
    return {'first_ns':int(times[0]),'last_ns':int(times[-1]),'count':len(times),'median_ns':median,
            'max_delta_ns':int(d.max()),'duplicates':int((d==0).sum()),'regressions':int((d<0).sum()),
            'gaps':[[int(times[i]),int(times[i+1])] for i in np.flatnonzero(d>1.5*median)]}


def crosses(gaps,start,end):
    # Gap interiors are open: exact valid endpoint samples remain admissible.
    return any(start<b and end-1>a for a,b in gaps)


class Reference:
    def __init__(self,t,p,q):
        self.t=np.asarray(t,dtype=np.int64); self.p=np.asarray(p,dtype=float); self.q=np.asarray(q,dtype=float)
        norms=np.linalg.norm(self.q,axis=1)
        if len(t)<2 or np.any(np.diff(self.t)<=0) or not np.isfinite(self.p).all() or not np.isfinite(self.q).all() or np.any(norms==0):
            raise ValueError('Invalid reference')
        self.q=self.q/norms[:,None]
        self.gaps=[[int(self.t[i]),int(self.t[i+1])] for i in np.flatnonzero(np.diff(self.t)>GAP)]

    def at(self,t):
        j=int(np.searchsorted(self.t,t))
        if j<len(self.t) and self.t[j]==t: return self.p[j],self.q[j],0
        if j==0 or j==len(self.t): raise ValueError('reference_outside')
        gap=int(self.t[j]-self.t[j-1])
        if gap>GAP: raise ValueError('reference_gap')
        f=(t-int(self.t[j-1]))/gap; q0=self.q[j-1]; q1=self.q[j]
        dot=float(q0@q1)
        if dot<0: q1=-q1; dot=-dot
        if dot>.9995: q=(1-f)*q0+f*q1; q=q/np.linalg.norm(q)
        else:
            angle=math.acos(np.clip(dot,-1,1))
            q=(math.sin((1-f)*angle)*q0+math.sin(f*angle)*q1)/math.sin(angle)
        return (1-f)*self.p[j-1]+f*self.p[j],q,gap

    def segment(self,start,last):
        a=self.at(start); b=self.at(last)
        if crosses(self.gaps,start,last+1): raise ValueError('reference_gap')
        i=int(np.searchsorted(self.t,start,side='right'))
        j=int(np.searchsorted(self.t,last,side='left'))
        p=np.vstack([a[0],self.p[i:j],b[0]]); q=np.vstack([a[1],self.q[i:j],b[1]])
        times=np.r_[start,self.t[i:j],last]
        return p,q,max(a[2],b[2],int(np.max(np.diff(times),initial=0)))


def load_reference(path,map_path):
    rows=[line.split() for line in path.read_text().splitlines() if line.strip() and not line.startswith('#')]
    if any(len(r)<8 for r in rows): raise ValueError('Reference schema')
    t=[ns(r[0]) for r in rows]; p=np.array([[float(x) for x in r[1:4]] for r in rows]); q=np.array([[float(x) for x in r[4:8]] for r in rows])
    text=map_path.read_text(); tail=text.split('# T_map_world',1)
    if len(tail)!=2: raise ValueError('Missing T_map_world')
    matrix=np.array([json.loads(s) for s in re.findall(r'#\s*(\[[^\n]+\])',tail[1])[:4]])
    if matrix.shape!=(4,4) or not np.allclose(matrix[3],[0,0,0,1]) or not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-5) or np.linalg.det(matrix[:3,:3])<0:
        raise ValueError('Invalid rigid map transform')
    # q remains in world for invariant stationarity tests. Export mapped orientation separately.
    return Reference(t,p@matrix[:3,:3].T+matrix[:3,3],q),matrix


def quat_product(a,b):
    av,bv=np.array(a[:3]),np.array(b[:3]); aw,bw=a[3],b[3]
    return np.r_[aw*bv+bw*av+np.cross(av,bv),aw*bw-av@bv]


def matrix_quat(m):
    # Symmetric eigensystem avoids trace singularities; deterministic quaternion sign.
    r=m[:3,:3]; K=np.array([[r[0,0]-r[1,1]-r[2,2],r[1,0]+r[0,1],r[2,0]+r[0,2],r[2,1]-r[1,2]],
        [r[1,0]+r[0,1],r[1,1]-r[0,0]-r[2,2],r[2,1]+r[1,2],r[0,2]-r[2,0]],
        [r[2,0]+r[0,2],r[2,1]+r[1,2],r[2,2]-r[0,0]-r[1,1],r[1,0]-r[0,1]],
        [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],np.trace(r)]])/3
    q=np.linalg.eigh(K)[1][:,-1]
    return -q if q[np.argmax(abs(q))]<0 else q


def imu_stats(values,scale):
    if len(values)<2 or not np.isfinite(values).all(): raise ValueError('imu_invalid')
    norms=np.linalg.norm(values[:,:3],axis=1)
    return {'gyro_norm_p95_rad_s':float(np.quantile(norms,.95)), 'gyro_norm_max_rad_s':float(norms.max()),
            'accel_variance_sum_m2_s4':float(np.var(values[:,3:]*scale,axis=0,ddof=1).sum()),'imu_count':len(values)}

def imu_pass(s):
    return s['gyro_norm_p95_rad_s']<=CONFIG['gyro_p95'] and s['gyro_norm_max_rad_s']<=CONFIG['gyro_max'] and s['accel_variance_sum_m2_s4']<=CONFIG['accel_variance_sum']


def components(xy):
    parent=list(range(len(xy))); xy=np.asarray(xy)
    def find(i):
        while parent[i]!=i: parent[i]=parent[parent[i]]; i=parent[i]
        return i
    for i in range(len(xy)):
        for j in range(i):
            if np.linalg.norm(xy[i]-xy[j])<5:
                a,b=find(i),find(j); parent[max(a,b)]=min(a,b)
    return [find(i) for i in range(len(xy))]


def group_split(rows,dataset):
    if not rows: return
    roots=components([[r['evaluator_only_ref_x_m'],r['evaluator_only_ref_y_m']] for r in rows])
    def ids(roots,prefix):
        labels={k:prefix+min(r['window_id'] for r,x in zip(rows,roots) if x==k) for k in set(roots)}
        return [labels[k] for k in roots]
    spatial=ids(roots,'sp-'); temporal=list(range(len(rows)))
    def root(par,i):
        while par[i]!=i: par[i]=par[par[i]]; i=par[i]
        return i
    def join(par,i,j):
        a,b=root(par,i),root(par,j); par[max(a,b)]=min(a,b)
    effective=list(range(len(rows)))
    for i,r in enumerate(rows):
        for j,s in enumerate(rows[:i]):
            overlap=r['_support_start']<s['_support_end'] and s['_support_start']<r['_support_end']
            if overlap: join(temporal,i,j)
            if overlap or roots[i]==roots[j]: join(effective,i,j)
    tp=ids([root(temporal,i) for i in range(len(rows))],'tp-')
    ef=ids([root(effective,i) for i in range(len(rows))],'ef-')
    order=sorted(set(ef),key=lambda k:(min(r['t_input_start_ns'] for r,x in zip(rows,ef) if x==k),k))
    tune=set(order[:(3*len(order)+9)//10]) if dataset==DATASETS[0] else set()
    for r,sp,t,e in zip(rows,spatial,tp,ef):
        r.update(spatial_block_id=sp,temporal_block_id=t,effective_block_id=e,split='heldout' if dataset==DATASETS[1] else 'tuning' if e in tune else 'final')
    tuning=[r for r in rows if r['split']=='tuning']
    for r in rows:
        if r['split']=='final' and any(max(r['_support_start']-s['_support_end'],s['_support_start']-r['_support_end'])<35*NS for s in tuning):
            r['exclusion_reason']='temporal_guard'; r['eligibility_status']='excluded'


def generate_dataset(root,dataset):
    base=root/dataset
    packed=(base/'sensor-index.json.gz').read_bytes(); data=json.loads(gzip.decompress(packed))
    lidar=data['lidar']; imu=data['imu']
    lt=np.array([r[1] for r in lidar],dtype=np.int64); it=np.array([r[1] for r in imu],dtype=np.int64)
    first=np.array([r[2]+r[3] for r in lidar],dtype=np.int64); last=np.array([r[2]+r[4] for r in lidar],dtype=np.int64)
    iv=np.array([r[2:] for r in imu],dtype=float)
    ref,T=load_reference(base/'alidarState.txt',base/'map.yaml'); tq=matrix_quat(T)
    lc,ic=cadence(lt),cadence(it)
    for times,c in ((lt,lc),(it,ic)):
        cuts=np.r_[0,np.flatnonzero(np.diff(times)>1.5*c['median_ns'])+1,len(times)]
        c['max_continuous_ns']=max(int(times[b-1]-times[a]) for a,b in zip(cuts[:-1],cuts[1:]))
        c['gap_count']=len(c['gaps'])
    if any(c[k] for c in (lc,ic) for k in ('duplicates','regressions')) or np.any(np.diff(first)<=0) or np.any(np.diff(last)<=0) or np.any(last<first):
        raise ValueError('Nonmonotonic sensor input: inspect before inventory')
    sources={p.name:digest(p.read_bytes()) for p in [base/'alidarState.txt',base/'map.yaml',base/'map.pgm']}
    sources.update(sensor_index_sha256=digest(gzip.decompress(packed)),bag_sources=data['sources'],bag_sha256=None)
    source_hash=digest(canonical(sources)); confighash=digest(canonical(CONFIG))
    codehash=digest(Path(__file__).read_bytes())
    def valid(start,end):
        if start<it[0] or end-1>it[-1]: raise ValueError('imu_outside')
        if start<first[0] or end-1>last[-1]: raise ValueError('lidar_outside')
        if crosses(lc['gaps'],start,end): raise ValueError('lidar_gap')
        if crosses(ic['gaps'],start,end): raise ValueError('imu_gap')
        ref.segment(start,end-1)
    def certificate(start,end,level):
        valid(start,end); p,q,bracket=ref.segment(start,end-1)
        dis=float(np.linalg.norm(p-p[0],axis=1).max())
        angle=float(np.degrees(2*np.arccos(np.clip(np.abs(q@q[0]),0,1))).max())
        i,j=np.searchsorted(it,[start,end]); stats=imu_stats(iv[i:j],CONFIG['accel_scale'])
        if dis>CONFIG['levels'][level][0] or angle>CONFIG['levels'][level][1]: raise ValueError('reference_not_stationary')
        if not imu_pass(stats): raise ValueError('imu_not_stationary')
        return dis,angle,bracket,stats
    rows=[]; excluded=[]; attempted=Counter()
    variants=[('S1','primary',None),('S2','1s',NS),('S2','primary',3*NS),('S2','5s',5*NS),('S3','primary',30*NS)]
    for level in ('A','B'):
        next_start={'S1':-1,'S2':-1,'S3':-1}
        for k,start0 in enumerate(first):
            start=int(start0); accepted_families=set()
            for scenario,variant,duration in variants:
                key=f'{level}/{scenario}/{variant}'; attempted[key]+=1
                warm=start if scenario=='S3' else start-2*NS
                end=int(last[k])+1 if scenario=='S1' else start+duration
                if warm<next_start[scenario]:
                    excluded.append([level,scenario,variant,start,'reserved_support_overlap']); continue
                try:
                    # Only certify S3 initialization now; actual LIO prefix is runtime evidence.
                    certend=start+2*NS if scenario=='S3' else end
                    dis,ang,bracket,stats=certificate(warm,certend,level)
                    a=int(np.searchsorted(it,warm,side='right'))-1
                    b=int(np.searchsorted(it,(end if scenario!='S3' else min(end,int(it[-1])))-1))
                    # S3 reserves the full possible budget, irrespective of private truth prefix.
                    support_end=max(end,int(it[b])+1) if b<len(it) else end
                    if scenario=='S1': scans=[k]
                    else: scans=list(range(k,int(np.searchsorted(last,end,side='left'))))
                    if not scans: raise ValueError('no_complete_frame')
                    pose_time=int(last[k]) if scenario=='S1' else certend-1
                    pose,quat,gap=ref.at(pose_time)
                    anchor,_,_=ref.at(start)
                    quat=quat_product(tq,quat)
                    fitend=None if scenario in ('S1','S3') else start+(duration*2)//3
                    row=dict(dataset_id=dataset,scenario=scenario,variant_id=variant,stationarity_level=level,
                        t_imu_start_ns=warm,t_input_start_ns=start,t_input_end_ns=None if scenario=='S3' else end,
                        t_imu_end_ns=None if scenario=='S3' else int(it[b])+1,t_conditioning_end_ns=start+2*NS if scenario=='S3' else start,
                        t_fit_end_ns=fitend,t_verify_start_ns=fitend,imu_certified=True,imu_check_status='pass',
                        lidar_gap_flag=False,imu_gap_flag=False,reference_gap_flag=False,timing_verified=True,
                        eligibility_status='prepared_runtime_pending' if scenario=='S3' else 'eligible',exclusion_reason='',
                        runtime_status='pending_lio_and_physical_postcheck' if scenario=='S3' else 'not_executed',
                        input_scan_ids_hash=digest(canonical(scans if scenario!='S3' else [k])),
                        source_manifest_sha256=source_hash,scenario_config_sha256=confighash,generator_version=VERSION,
                        generator_source_sha256=codehash,schema_version=1,protocol_commit=PROTOCOL,seed=None,
                        exposure_config_id=f'{scenario}/{variant}',planning_radius_m=2.8 if scenario=='S3' else None,
                        hard_radius_m=3.0 if scenario=='S3' else None,scan_count=len(scans) if scenario!='S3' else None,
                        evaluator_only_ref_pose_time_ns=pose_time,evaluator_only_ref_stationarity_displacement_m=dis,
                        evaluator_only_ref_stationarity_angle_deg=ang,evaluator_only_ref_bracket_gap_s=bracket/NS,
                        _support_start=int(it[a]),_support_end=support_end,_anchor=anchor.tolist(),_scan_start=k)
                    for name,value in zip(('x','y','z'),pose): row[f'evaluator_only_ref_{name}_m']=float(value)
                    for name,value in zip(('qx','qy','qz','qw'),quat): row[f'evaluator_only_ref_{name}']=float(value)
                    row.update({f'evaluator_only_{name}':value for name,value in stats.items()})
                    if scenario=='S3':
                        # Evaluator diagnostic only. No executor consumes this ending.
                        length=0.; radius=0.; prefix=start; reason='acquisition_limit'
                        for scan in scans:
                            t=int(last[scan]); stop=t+1
                            try:
                                valid(start,stop)
                                pp,_,_=ref.segment(prefix,t)
                            except ValueError as e: reason=str(e); break
                            segment_length=float(np.linalg.norm(np.diff(pp[:,:2],axis=0),axis=1).sum())
                            rad=float(np.linalg.norm(pp[:,:2]-anchor[:2],axis=1).max())
                            if max(radius,rad)>2.8 or length+segment_length>6:
                                reason='reference_planning_radius' if rad>2.8 else 'reference_planning_path'; break
                            prefix=t; radius=max(radius,rad); length+=segment_length
                        row.update(evaluator_only_prefix_end_ns=prefix+1,evaluator_only_max_radius_m=radius,
                            evaluator_only_path_m=length,evaluator_only_duration_s=(prefix-start)/NS,evaluator_only_stop_reason=reason)
                    identity={name:row[name] for name in ('dataset_id','scenario','variant_id','stationarity_level','input_scan_ids_hash','source_manifest_sha256','scenario_config_sha256','generator_source_sha256','schema_version','protocol_commit')}
                    identity.update({name:row[name] for name in row if name.startswith('t_') and name.endswith('_ns')})
                    row['window_id']='w-'+digest(canonical(identity)); row['duplicate_family_id']=f'{dataset}:{start}:{scenario}'
                    rows.append(row); accepted_families.add(scenario)
                except ValueError as e: excluded.append([level,scenario,variant,start,str(e)])
            for scenario in accepted_families:
                reserve=start+30*NS if scenario=='S3' else start+5*NS if scenario=='S2' else int(last[k])+1
                j=int(np.searchsorted(it,min(reserve-1,int(it[-1]))))
                # Require next left bracket to follow this right bracket strictly.
                next_start[scenario]=int(it[min(j+1,len(it)-1)])+1
    # Group using START anchors, not scored end poses. Restore score fields afterwards.
    scored=[(r['evaluator_only_ref_x_m'],r['evaluator_only_ref_y_m']) for r in rows]
    for r in rows: r['evaluator_only_ref_x_m'],r['evaluator_only_ref_y_m']=r['_anchor'][:2]
    group_split(rows,dataset)
    for r,xy in zip(rows,scored): r['evaluator_only_ref_x_m'],r['evaluator_only_ref_y_m']=xy
    summaries=[]
    for scenario,variant,_ in variants:
        cell=[r for r in rows if r['scenario']==scenario and r['variant_id']==variant]
        counts={lev:len({r['effective_block_id'] for r in cell if r['stationarity_level']==lev and r['eligibility_status']!='excluded'}) for lev in ('A','B')}
        chosen='A' if counts['A']>=30 else 'B'
        for r in cell:
            r['selected_level']=r['stationarity_level']==chosen
            r['independent_block_count']=counts[r['stationarity_level']]
            if r['eligibility_status']!='excluded' and counts[r['stationarity_level']]<30:
                r['eligibility_status']='insufficient_independent_coverage'
        chosenrows=[r for r in cell if r['selected_level'] and not r['exclusion_reason']]
        summaries.append(dict(scenario=scenario,variant_id=variant,selected_level=chosen,blocks_by_level=counts,
            selected_windows=len(chosenrows),runtime_pending=scenario=='S3',
            eligibility_status='insufficient_independent_coverage' if counts[chosen]<30 else 'adequate_inventory',
            windows_by_level={lev:sum(r['stationarity_level']==lev and not r['exclusion_reason'] for r in cell) for lev in ('A','B')},
            exclusions_by_level={lev:dict(Counter(e[-1] for e in excluded if e[:3]==[lev,scenario,variant])) for lev in ('A','B')},
            splits={s:sum(r['split']==s for r in chosenrows) for s in ('tuning','final','heldout')},
            start_min_ns=min((r['t_input_start_ns'] for r in chosenrows),default=None),start_max_ns=max((r['t_input_start_ns'] for r in chosenrows),default=None)))
    for r in rows:
        r['evaluator_only_support_start_ns']=r.pop('_support_start'); r['evaluator_only_support_end_ns']=r.pop('_support_end')
        r['evaluator_only_anchor_map_xy']=r.pop('_anchor')[:2]; r['evaluator_only_first_scan_ordinal']=r.pop('_scan_start')
        for k,v in list(r.items()):
            if k.endswith('_ns') and (k.startswith('t_') or k.startswith('evaluator_only_prefix') or k.startswith('evaluator_only_ref_pose')):
                r[k[:-3]+'_s']=seconds(v)
    # Nearest differences are DIAGNOSTICS only, never nearest-pose scoring/offset correction.
    end_header=lt+np.array([r[4] for r in lidar],dtype=np.int64)
    def nearest_delta(times):
        out=[]
        for t in ref.t:
            j=int(np.searchsorted(times,t)); options=[int(times[i]) for i in (j-1,j) if 0<=i<len(times)]
            out.append(int(t)-min(options,key=lambda x:(abs(x-int(t)),x)))
        return {'min_ns':min(out),'median_ns':float(np.median(out)),'max_ns':max(out),'within_1us':sum(abs(x)<=1000 for x in out),'count':len(out)}
    audit=dict(dataset_id=dataset,sources=sources,topics=data['topics'],frame_ids=data['frame_ids'],lidar=lc,imu=ic,
        reference=cadence(ref.t),reference_gaps=ref.gaps,
        ref_minus_nearest_point_end=nearest_delta(last),ref_minus_nearest_header_plus_maxoffset=nearest_delta(end_header),
        header_minus_timebase_ns={'min':int((lt-first).min()),'max':int((lt-first).max()),'median':float(np.median(lt-first))},
        frame_span_ns={'min':int((last-first).min()),'max':int((last-first).max()),'median':float(np.median(last-first))},
        accel_norm_raw_median=float(np.median(np.linalg.norm(iv[:,3:],axis=1))),
        storage_minus_header_ns={name:{'min':min(int(r[0]-r[1]) for r in values),'max':max(int(r[0]-r[1]) for r in values),'median':float(np.median([r[0]-r[1] for r in values]))} for name,values in [('lidar',lidar),('imu',imu)]},
        summary=summaries,attempted=dict(attempted),exclusions=dict(Counter(e[-1] for e in excluded)),
        continuous_variant={'variant_id':'S3_continuous_rebased','status':'unavailable_no_sensor_odometry_artifact'})
    return rows,excluded,audit


# Explicit empty-inventory schema is mandatory; no rows still has a usable CSV header.
BASE_COLUMNS='''window_id dataset_id scenario variant_id exposure_config_id input_scan_ids_hash
stationarity_level selected_level imu_certified imu_check_status lidar_gap_flag imu_gap_flag reference_gap_flag timing_verified
eligibility_status exclusion_reason spatial_block_id temporal_block_id effective_block_id split independent_block_count duplicate_family_id
planning_radius_m hard_radius_m scan_count source_manifest_sha256 scenario_config_sha256 generator_version generator_source_sha256
schema_version protocol_commit seed runtime_status
 t_imu_start_ns t_input_start_ns t_input_end_ns t_imu_end_ns t_conditioning_end_ns t_fit_end_ns t_verify_start_ns
 evaluator_only_ref_x_m evaluator_only_ref_y_m evaluator_only_ref_z_m evaluator_only_ref_qx evaluator_only_ref_qy evaluator_only_ref_qz evaluator_only_ref_qw
 evaluator_only_ref_pose_time_ns evaluator_only_ref_stationarity_displacement_m evaluator_only_ref_stationarity_angle_deg evaluator_only_ref_bracket_gap_s
 evaluator_only_gyro_norm_p95_rad_s evaluator_only_gyro_norm_max_rad_s evaluator_only_accel_variance_sum_m2_s4 evaluator_only_imu_count
 evaluator_only_support_start_ns evaluator_only_support_end_ns evaluator_only_anchor_map_xy evaluator_only_first_scan_ordinal
 evaluator_only_prefix_end_ns evaluator_only_max_radius_m evaluator_only_path_m evaluator_only_duration_s evaluator_only_stop_reason
 executor_input_start_ns executor_input_end_ns executor_imu_start_ns executor_conditioning_end_ns executor_stop_reason'''.split()
COLUMNS=sorted(BASE_COLUMNS+[k[:-3]+'_s' for k in BASE_COLUMNS if k.endswith('_ns') and
                            (k.startswith('t_') or k.startswith('evaluator_only_prefix') or k.startswith('evaluator_only_ref_pose'))])


def serialize(rows):
    columns=COLUMNS
    unknown={k for r in rows for k in r}-set(columns)
    if unknown: raise ValueError(f'Undeclared columns: {unknown}')
    f=io.StringIO(newline=''); writer=csv.DictWriter(f,fieldnames=columns,lineterminator='\n'); writer.writeheader()
    for row in rows:
        writer.writerow({k:'' if v is None else str(v).lower() if isinstance(v,bool) else canonical(v).decode() if isinstance(v,list) else v for k,v in row.items()})
    return f.getvalue().encode(),columns


def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument('--input-root',type=Path,required=True); ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args(); rows=[]; exclusions=[]; audits=[]
    for dataset in DATASETS:
        rr,ee,aa=generate_dataset(a.input_root,dataset); rows+=rr; exclusions += [[dataset,*e] for e in ee]; audits.append(aa)
    rows.sort(key=lambda r:(DATASETS.index(r['dataset_id']),r['t_input_start_ns'],r['scenario'],r['variant_id'],r['window_id']))
    data,columns=serialize(rows)
    metadata={'config':CONFIG,'config_sha256':digest(canonical(CONFIG)),'generator_source_sha256':digest(Path(__file__).read_bytes()),
              'runtime':{'python':sys.version.split()[0],'numpy':np.__version__},'windows_csv_sha256':digest(data),'row_count':len(rows),'columns':columns,
              'domains':{k:'executor' if k.startswith('executor_') else 'admin' if k in {'window_id','dataset_id','scenario','variant_id','exposure_config_id','input_scan_ids_hash','generator_version','generator_source_sha256','schema_version','protocol_commit','seed','source_manifest_sha256','scenario_config_sha256'} else 'evaluator_only' for k in columns},
              'executor_fields':'unfilled; generated from actual allowed input/odometry later','method_columns':[], 'audits':audits}
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'windows.csv').write_bytes(data)
    (a.output/'windows.metadata.json').write_bytes(canonical(metadata)+b'\n')
    (a.output/'exclusions.jsonl.gz').write_bytes(gzip.compress(b''.join(canonical(e)+b'\n' for e in exclusions),mtime=0))
    print(json.dumps({'rows':len(rows),'sha256':digest(data),'summaries':{d['dataset_id']:d['summary'] for d in audits}},indent=2))

if __name__=='__main__': main()
