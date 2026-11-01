"""Sensor-only interface contract. Deliberately imports no evaluator or file reader.

This is a schema boundary, not a process sandbox. A later runner must supply
original message bytes and deny candidate access to private evaluator artifacts.
"""
from types import MappingProxyType


def execution_config(scenario, variant, **extra):
    if extra:
        raise ValueError('Only fixed scenario/variant identifiers accepted')
    durations={'1s':1_000_000_000,'primary':3_000_000_000,'5s':5_000_000_000}
    if scenario not in ('S1','S2','S3'): raise ValueError('Unknown scenario')
    if scenario=='S2' and variant not in durations: raise ValueError('Unknown variant')
    if scenario=='S1' and variant!='primary': raise ValueError('Unknown variant')
    if scenario=='S3' and variant not in ('primary','S3_continuous_rebased'): raise ValueError('Unknown variant')
    return MappingProxyType(dict(scenario=scenario,variant_id=variant,conditioning_ns=2_000_000_000,
        acquisition_ns=(durations[variant] if scenario=='S2' else 30_000_000_000 if scenario=='S3' else None),
        lidar_message_limit=1 if scenario=='S1' else None,deadline_wall_ns=60_000_000_000,
        operational_radius_m=2.8 if scenario=='S3' else None,path_limit_m=6.0 if scenario=='S3' else None,
        readiness_limit_ns=5_000_000_000 if scenario=='S3' else None,
        deskew='none' if scenario=='S1' else 'sensor_only',
        fit_fraction=(7,10) if scenario=='S1' else (2,3),seed=None))


def validate_method_packet(packet):
    if set(packet)!={'kind','sensor_stamp_ns','payload'}:
        raise ValueError('Private or unknown packet field')
    if packet['kind'] not in ('lidar','imu','local_odometry'): raise ValueError('Unknown sensor kind')
    if type(packet['sensor_stamp_ns']) is not int or not isinstance(packet['payload'],bytes):
        raise ValueError('Require original integer sensor time and serialized sensor payload')
    return packet
