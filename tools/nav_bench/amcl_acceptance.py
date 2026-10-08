"""Sensor/map-only observable acceptance checks for global AMCL.

This module deliberately accepts in-memory sensor/map products only. It does
not import the point-5 inventory, reference reader or evaluator.
"""
from dataclasses import dataclass
import math

import numpy as np


XY_CLUSTER_M = 0.20
YAW_CLUSTER_RAD = math.radians(5.0)
DISTINCT_XY_M = 0.30
DISTINCT_YAW_RAD = math.radians(8.0)


def angle_error(a, b):
    """Signed shortest angular difference a-b in [-pi, pi]."""
    return math.atan2(math.sin(a-b), math.cos(a-b))


@dataclass(frozen=True)
class Scan:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass(frozen=True)
class Occupancy:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    occupied_threshold: int = 65


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    yaw: float
    weight: float


@dataclass(frozen=True)
class BeamCoverage:
    passes: bool
    finite_count: int
    span_rad: float


@dataclass(frozen=True)
class GateResult:
    passes: bool
    reason: str
    particle_concentration: float
    primary_support: float
    runner_up_support: float
    finite_beams: int
    beam_span_rad: float


@dataclass(frozen=True)
class AcceptanceEvent:
    sensor_stamp_ns: int
    wall_elapsed_s: float
    pose: tuple
    confirmation_count: int


def _particle_array(particles):
    p=np.asarray(particles,dtype=float)
    if p.ndim!=2 or p.shape[1]!=4 or not len(p) or not np.isfinite(p).all():
        raise ValueError('particles must be finite Nx4 [x,y,yaw,weight]')
    if np.any(p[:,3]<0) or p[:,3].sum()<=0:
        raise ValueError('particle weights must be nonnegative and nonzero')
    return p


def concentration(particles, pose):
    """Normalized particle weight inside the fixed AMCL pose neighborhood."""
    p=_particle_array(particles)
    x,y,yaw=map(float,pose)
    close=(np.hypot(p[:,0]-x,p[:,1]-y)<=XY_CLUSTER_M)
    close &= np.abs(np.arctan2(np.sin(p[:,2]-yaw),np.cos(p[:,2]-yaw)))<=YAW_CLUSTER_RAD
    return float(p[close,3].sum()/p[:,3].sum())


def _candidate_at(p, seed):
    dx=p[:,0]-p[seed,0]; dy=p[:,1]-p[seed,1]
    dyaw=np.arctan2(np.sin(p[:,2]-p[seed,2]),np.cos(p[:,2]-p[seed,2]))
    members=(np.hypot(dx,dy)<=XY_CLUSTER_M)&(np.abs(dyaw)<=YAW_CLUSTER_RAD)
    weights=p[members,3]
    if not len(weights) or weights.sum()==0:
        return None, members
    x=float(np.average(p[members,0],weights=weights)); y=float(np.average(p[members,1],weights=weights))
    s=float(np.sum(weights*np.sin(p[members,2]))); c=float(np.sum(weights*np.cos(p[members,2])))
    return Candidate(x,y,math.atan2(s,c),float(weights.sum()/p[:,3].sum())),members


def particle_modes(particles, max_modes=5):
    """Find deterministic particle modes from high-weight and stratified seeds.

    Evaluating every particle as a seed was too slow for a live Orin observer.
    The bounded seed set is the 128 highest-weight samples plus 128 evenly
    spaced samples (deduplicated); every selected seed is compared with all
    remaining particles using the exact pose-neighborhood gate.
    """
    p=_particle_array(particles); n=len(p); remaining=np.ones(n,dtype=bool); modes=[]
    budget=min(128,n)
    high=np.argsort(-p[:,3],kind='stable')[:budget]
    spread=np.linspace(0,n-1,min(128,n),dtype=np.int64)
    seeds=np.unique(np.concatenate((high,spread)))
    weights=p[:,3];total_weight=float(weights.sum())
    for _ in range(min(max_modes,n)):
        active_seeds=seeds[remaining[seeds]]
        if not len(active_seeds):break
        dx=p[active_seeds,0,None]-p[None,:,0];dy=p[active_seeds,1,None]-p[None,:,1]
        yaw_delta=p[active_seeds,2,None]-p[None,:,2]
        near=(dx*dx+dy*dy<=XY_CLUSTER_M**2)&(np.cos(yaw_delta)>=math.cos(YAW_CLUSTER_RAD))
        near &= remaining[None,:]
        masses=near@weights
        max_mass=float(masses.max());tied=np.flatnonzero(masses==max_mass)
        seed_pos=min(tied,key=lambda j:(float(p[active_seeds[j],0]),float(p[active_seeds[j],1]),
                                        float(p[active_seeds[j],2]),int(active_seeds[j])))
        seed=int(active_seeds[seed_pos]);members=near[seed_pos]
        w=weights[members];mass=float(w.sum())
        if mass<=0:break
        x=float(np.average(p[members,0],weights=w));y=float(np.average(p[members,1],weights=w))
        s=float(np.sum(w*np.sin(p[members,2])));c=float(np.sum(w*np.cos(p[members,2])))
        modes.append(Candidate(x,y,math.atan2(s,c),mass/total_weight))
        remaining[members]=False
    return modes


def beam_coverage(scan):
    ranges=np.asarray(scan.ranges,dtype=float)
    if (ranges.ndim!=1 or not len(ranges) or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment) or scan.angle_increment<=0
            or not math.isfinite(scan.range_min) or not math.isfinite(scan.range_max)
            or scan.range_min<0 or scan.range_max<=scan.range_min):
        raise ValueError('scan ranges must be a nonempty vector')
    angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
    count=int(valid.sum())
    span=float(angles[valid].max()-angles[valid].min()) if count else 0.
    return BeamCoverage(count>=50 and span>=math.pi/2,count,span)


def split_s1_scan(scan):
    """Return deterministic 70/30 fit and check scans from one S1 frame.

    The periodic integer-ratio assignment distributes each partition over the
    complete angular sweep. AMCL receives only the fit scan; the observer gets
    the check scan. This preserves the deliberately weak same-frame protocol.
    """
    ranges=np.asarray(scan.ranges,dtype=float)
    index=np.arange(len(ranges),dtype=np.int64)
    fit=((index*7)//10)!=(((index+1)*7)//10)
    fit_ranges=np.where(fit,ranges,np.inf)
    check_ranges=np.where(fit,np.inf,ranges)
    return (Scan(fit_ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max),
            Scan(check_ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max))


def map_support(scan, pose, occupancy, tolerance_m=.15):
    """Fraction of finite scan endpoints near known occupied map cells."""
    ranges=np.asarray(scan.ranges,dtype=float); angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
    if not valid.any():
        return 0.
    rr=ranges[valid]; aa=angles[valid]
    c,s=math.cos(float(pose[2])),math.sin(float(pose[2]))
    mx=float(pose[0])+c*rr*np.cos(aa)-s*rr*np.sin(aa)
    my=float(pose[1])+s*rr*np.cos(aa)+c*rr*np.sin(aa)
    # Apply inverse OccupancyGrid origin pose, then convert to cell-center coordinates.
    ox,oy=mx-occupancy.origin_x,my-occupancy.origin_y
    co,so=math.cos(occupancy.origin_yaw),math.sin(occupancy.origin_yaw)
    local_x=co*ox+so*oy
    local_y=-so*ox+co*oy
    grid=np.asarray(occupancy.data)
    if grid.ndim!=2 or min(grid.shape,default=0)<=0 or not math.isfinite(occupancy.resolution) or occupancy.resolution<=0:
        raise ValueError('invalid occupancy grid')
    radius=int(math.ceil(tolerance_m/occupancy.resolution))+1
    base_ix=np.floor(local_x/occupancy.resolution).astype(int)
    base_iy=np.floor(local_y/occupancy.resolution).astype(int)
    hit=np.zeros(len(local_x),dtype=bool)
    for ixoff in range(-radius,radius+1):
        for iyoff in range(-radius,radius+1):
            ix=base_ix+ixoff; iy=base_iy+iyoff
            inside=(ix>=0)&(ix<grid.shape[1])&(iy>=0)&(iy<grid.shape[0])
            dx=np.maximum(np.abs(local_x-(ix+.5)*occupancy.resolution)-occupancy.resolution/2,0.)
            dy=np.maximum(np.abs(local_y-(iy+.5)*occupancy.resolution)-occupancy.resolution/2,0.)
            near=np.hypot(dx,dy)<=tolerance_m+1e-12
            hit |= inside & near & (grid[np.clip(iy,0,grid.shape[0]-1),np.clip(ix,0,grid.shape[1]-1)]>=occupancy.occupied_threshold)
    return float(hit.sum()/len(hit))


def evaluate(scan, particles, pose, occupancy):
    """Apply fixed AMCL acceptance gates. Inputs contain no evaluator data."""
    modes=particle_modes(particles,5); conc=concentration(particles,pose)
    beams=beam_coverage(scan); primary=map_support(scan,pose,occupancy)
    runner=None
    for candidate in modes:
        if math.hypot(candidate.x-pose[0],candidate.y-pose[1])>DISTINCT_XY_M or abs(angle_error(candidate.yaw,pose[2]))>DISTINCT_YAW_RAD:
            support=map_support(scan,(candidate.x,candidate.y,candidate.yaw),occupancy)
            runner=support if runner is None else max(runner,support)
    margin=primary-runner if runner is not None else float('-inf')
    checks=[(conc>=.90,'particle_concentration'),(beams.passes,'beam_coverage'),
            (primary>=.70,'map_support'),(runner is not None and margin>=.10,'runner_up_margin')]
    failed=next((name for ok,name in checks if not ok),None)
    return GateResult(failed is None,failed or 'accepted',conc,primary,
                      runner,beams.finite_count,beams.span_rad)


class AcceptanceTracker:
    """Require the point-4 distinct-batch temporal confirmation for S2/S3."""
    def __init__(self, scenario):
        if scenario not in ('S1','S2','S3'):
            raise ValueError('unknown scenario')
        self.scenario=scenario; self.samples=[]; self.last_stamp=None; self.event=None

    def observe(self, sensor_stamp_ns, wall_elapsed_s, pose, gate):
        if self.event is not None:
            return self.event
        if self.last_stamp is not None and sensor_stamp_ns<=self.last_stamp:
            raise ValueError('scan update stamps must strictly increase')
        self.last_stamp=sensor_stamp_ns
        if not gate.passes:
            self.samples=[]
            return None
        sample=(int(sensor_stamp_ns),float(wall_elapsed_s),tuple(map(float,pose)))
        self.samples.append(sample)
        if self.scenario=='S1':
            self.event=AcceptanceEvent(sample[0],sample[1],sample[2],1)
            return self.event
        min_count,min_span=(2,1_000_000_000) if self.scenario=='S2' else (3,5_000_000_000)
        if len(self.samples)<min_count:
            return None
        first=self.samples[0]; last=self.samples[-1]
        if last[0]-first[0]<min_span:
            return None
        if not any(b[0]-a[0]>=500_000_000 for i,a in enumerate(self.samples) for b in self.samples[i+1:]):
            return None
        for i,(_,_,a) in enumerate(self.samples):
            for _,_,b in self.samples[i+1:]:
                if math.hypot(b[0]-a[0],b[1]-a[1])>.20 or abs(angle_error(b[2],a[2]))>math.radians(5):
                    self.samples=[sample]
                    return None
        self.event=AcceptanceEvent(last[0],last[1],last[2],len(self.samples))
        return self.event


def timeout_outcome(accepted, elapsed_s, timeout_s=60.):
    if accepted:
        return 'accepted'
    return 'timeout' if elapsed_s>=timeout_s else 'running'
