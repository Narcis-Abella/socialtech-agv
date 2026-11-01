#!/usr/bin/env python3
"""Run one isolated DEVELOPMENT global-AMCL trial on an NX with Docker.

The method container receives only the map and sensor frontend, never a
reference, inventory CSV or saved initial pose. Each call creates a new output
directory and retains both containers and logs. Run sequentially on an idle NX.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time


def command(args,*,timeout=25,check=True):
    result=subprocess.run(args,text=True,capture_output=True,timeout=timeout)
    if check and result.returncode:
        raise RuntimeError(f'{args!r}: {result.returncode}\n{result.stdout}\n{result.stderr}')
    return result


def docker_ros(container,*ros_args,timeout=25):
    return command(['docker','exec',container,'bash','-lc',
        'source /opt/ros/jazzy/setup.bash; source /opt/socialtech/setup.bash; '+
        ' '.join(ros_args)],timeout=timeout)


def wait_service(container,name,deadline_s=40):
    end=time.monotonic()+deadline_s
    while time.monotonic()<end:
        result=docker_ros(container,'ros2','service','list',timeout=10)
        if name in result.stdout.splitlines():return
        time.sleep(1)
    raise TimeoutError(f'{name} unavailable in {container}')


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('trial_id','bag','map_pgm','map_yaml','tools_dir','replay_qos','output_root'):
        ap.add_argument(name)
    ap.add_argument('--input-start-ns',type=int,required=True)
    ap.add_argument('--input-end-ns',type=int,required=True)
    ap.add_argument('--scenario',choices=('S1','S2'),default='S2')
    ap.add_argument('--start-offset-s',type=float,required=True)
    ap.add_argument('--playback-duration-s',type=float,default=5.)
    ap.add_argument('--staged-prefix-end-offset-s',type=float,
                    help='DEVELOPMENT throughput control: finish IMU-only prefix before LiDAR-only 1x replay')
    ap.add_argument('--rate',type=float,default=.25)
    ap.add_argument('--domain',type=int,required=True)
    ap.add_argument('--imu-median-ns',type=float,required=True)
    ap.add_argument('--height-m',type=float,default=1.)
    ap.add_argument('--band-min-m',type=float,default=.10)
    ap.add_argument('--band-max-m',type=float,default=1.80)
    ap.add_argument('--fov-deg',type=float,default=360.)
    ap.add_argument('--fov-center-deg',type=float,default=0.)
    ap.add_argument('--increment-deg',type=float,default=.5)
    ap.add_argument('--range-min-m',type=float,default=.5)
    ap.add_argument('--range-max-m',type=float,default=30.)
    ap.add_argument('--particles',type=int,default=2000)
    ap.add_argument('--max-beams',type=int,default=180)
    ap.add_argument('--recovery-alpha-fast',type=float,default=0.)
    ap.add_argument('--recovery-alpha-slow',type=float,default=0.)
    ap.add_argument('--nomotion-trigger',choices=('scan_delay','pose_ack'),default='scan_delay')
    ap.add_argument('--prescan-relay',action='store_true',
                    help='serialize each stationary scan after a no-motion service response')
    ap.add_argument('--rmw',choices=('rmw_cyclonedds_cpp','rmw_fastrtps_cpp'),
                    default='rmw_cyclonedds_cpp')
    ap.add_argument('--cyclone-config',type=Path)
    args=ap.parse_args()
    if (args.input_end_ns<=args.input_start_ns or args.rate<=0 or args.particles<1
            or (args.staged_prefix_end_offset_s is not None and not
                args.start_offset_s<args.staged_prefix_end_offset_s<args.start_offset_s+args.playback_duration_s)
            or not 0<=args.recovery_alpha_slow<=args.recovery_alpha_fast<=1):
        ap.error('invalid interval, rate or particle count')
    root=Path(args.output_root);root.mkdir(exist_ok=True)
    out=root/args.trial_id
    out.mkdir()  # fail rather than overwrite earlier evidence
    settings=vars(args).copy()
    if args.cyclone_config is not None:
        settings['cyclone_config']=str(args.cyclone_config)
        settings['cyclone_config_sha256']=file_sha256(args.cyclone_config)
    settings['schema_version']=1
    settings['classification_policy']='DEVELOPMENT / EXPLORATORY — NOT FINAL EVALUATION'
    settings['tool_hashes']={name:file_sha256(Path(args.tools_dir)/name)
        for name in ('sensor_relative_scan.py','amcl_ros_observer.py','amcl_acceptance.py',
                     'amcl_nomotion_driver.py','amcl_prescan_relay.py','amcl_topic_probe.py',
                     'launch/global_amcl_relative_scan.launch.py','config/amcl.yaml')}
    (out/'config.json').write_text(json.dumps(settings,sort_keys=True,indent=2)+'\n')
    run_name=f'reloc-{args.trial_id}-run';bag_name=f'reloc-{args.trial_id}-bag'
    env=['-e',f'ROS_DOMAIN_ID={args.domain}','-e','ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
         '-e',f'RMW_IMPLEMENTATION={args.rmw}']
    local_env={
        'GLOBAL_RELOC_MAP':'/map/map.yaml',
        'GLOBAL_RELOC_SCENARIO':args.scenario,
        'INPUT_START_NS':args.input_start_ns,'INPUT_END_NS':args.input_end_ns,
        'FIT_END_NS':args.input_end_ns,'IMU_MEDIAN_NS':args.imu_median_ns,
        'SENSOR_HEIGHT_HYPOTHESIS_M':args.height_m,
        'MIN_OBSTACLE_HEIGHT':args.band_min_m,'MAX_OBSTACLE_HEIGHT':args.band_max_m,
        'SCAN_FOV_DEG':args.fov_deg,'SCAN_FOV_CENTER_DEG':args.fov_center_deg,
        'SCAN_ANGLE_INCREMENT_DEG':args.increment_deg,
        'SCAN_RANGE_MIN':args.range_min_m,'SCAN_RANGE_MAX':args.range_max_m,
        'SCAN_INPUT_RELIABILITY':'reliable',
        'AMCL_MIN_PARTICLES':args.particles,'AMCL_MAX_PARTICLES':args.particles,
        'AMCL_MAX_BEAMS':args.max_beams,
        'AMCL_RECOVERY_ALPHA_FAST':args.recovery_alpha_fast,
        'AMCL_RECOVERY_ALPHA_SLOW':args.recovery_alpha_slow}
    local_env['NOMOTION_TRIGGER']=args.nomotion_trigger
    local_env['GLOBAL_RELOC_PRESCAN_RELAY']=str(args.prescan_relay).lower()
    for key,value in local_env.items():env.extend(['-e',f'{key}={value}'])
    cyclone_mount=[]
    if args.cyclone_config is not None:
        if args.rmw!='rmw_cyclonedds_cpp':ap.error('Cyclone config requires Cyclone RMW')
        env.extend(['-e','CYCLONEDDS_URI=file:///cyclone.xml'])
        cyclone_mount=['-v',f'{args.cyclone_config.resolve()}:/cyclone.xml:ro']
    method=['docker','run','-d','--name',run_name,'--network','host',*env,
        *cyclone_mount,
        '-v',f'{Path(args.map_pgm).resolve()}:/map/map.pgm:ro',
        '-v',f'{Path(args.map_yaml).resolve()}:/map/map.yaml:ro',
        '-v',f'{Path(args.tools_dir).resolve()}:/nb:ro',
        'socialtech:robot-nav','bash','-lc',
        'source /opt/ros/jazzy/setup.bash; source /opt/socialtech/setup.bash; '+
        'python3 /nb/run_launch_description.py /nb/launch/global_amcl_relative_scan.launch.py']
    def player_command(name,offset_s,duration_s,topics):
        return ['docker','run','-d','--name',name,'--network','host',
            '-e',f'ROS_DOMAIN_ID={args.domain}','-e','ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
            '-e',f'RMW_IMPLEMENTATION={args.rmw}',
            *(['-e','CYCLONEDDS_URI=file:///cyclone.xml',*cyclone_mount]
              if args.cyclone_config is not None else []),
            '-v',f'{Path(args.bag).resolve()}:/bag:ro',
            '-v',f'{Path(args.replay_qos).resolve()}:/qos.yaml:ro',
            'socialtech:robot-nav','bash','-lc',
            'source /opt/ros/jazzy/setup.bash; source /opt/socialtech/setup.bash; '
            f'ros2 bag play /bag --start-offset {offset_s} '
            f'--playback-duration {duration_s} --rate {args.rate} '
            '--clock --start-paused --topics '+ ' '.join(topics)+' '
            '--qos-profile-overrides-path /qos.yaml --disable-keyboard-controls']
    staged=args.staged_prefix_end_offset_s is not None
    main_offset=args.staged_prefix_end_offset_s if staged else args.start_offset_s
    main_duration=args.start_offset_s+args.playback_duration_s-main_offset
    player=player_command(bag_name,main_offset,main_duration,
                          ['/livox/lidar'] if staged else ['/livox/lidar','/livox/imu'])
    started=[]
    try:
        command(method);started.append(run_name)
        wait_service(run_name,'/reinitialize_global_localization')
        if staged:
            prefix_name=f'reloc-{args.trial_id}-prefix'
            prefix_duration=main_offset-args.start_offset_s
            command(player_command(prefix_name,args.start_offset_s,prefix_duration,
                                   ['/livox/imu']))
            started.append(prefix_name)
            wait_service(prefix_name,'/rosbag2_player/resume')
            time.sleep(3)
            docker_ros(run_name,'ros2','service','call','/reinitialize_global_localization',
                       'std_srvs/srv/Empty','"{}"')
            docker_ros(prefix_name,'ros2','service','call','/rosbag2_player/resume',
                       'rosbag2_interfaces/srv/Resume','"{}"')
            command(['docker','wait',prefix_name],timeout=max(20.,prefix_duration/args.rate+20.))
            for _ in range(20):
                logs=command(['docker','logs',run_name],check=False,timeout=5)
                if '"reason": "ok"' in logs.stdout+logs.stderr:break
                time.sleep(.5)
            else:raise RuntimeError('staged IMU conditioning did not complete before LiDAR replay')
        command(player);started.append(bag_name)
        wait_service(bag_name,'/rosbag2_player/resume')
        time.sleep(3)  # permit DDS discovery of all already-running subscribers
        if not staged:
            docker_ros(run_name,'ros2','service','call','/reinitialize_global_localization',
                       'std_srvs/srv/Empty','"{}"')
        docker_ros(bag_name,'ros2','service','call','/rosbag2_player/resume',
                   'rosbag2_interfaces/srv/Resume','"{}"')
        limit=time.monotonic()+max(75.,args.playback_duration_s/args.rate+65.)
        while time.monotonic()<limit:
            result=command(['docker','exec',run_name,'cat','/tmp/amcl-observer.jsonl'],
                           check=False,timeout=5)
            if any('"terminal_event": "timeout"' in line or '"acceptance_event"' in line
                   for line in result.stdout.splitlines()):
                break
            time.sleep(2)
    finally:
        for name in reversed(started):
            command(['docker','stop','-t','10',name],check=False,timeout=20)
            logs=command(['docker','logs',name],check=False,timeout=15)
            (out/f'{name}.log').write_text(logs.stdout+logs.stderr)
        if run_name in started:
            for source,target in (('/tmp/amcl-observer.jsonl','observer.jsonl'),
                                  ('/tmp/amcl-topic-probe.jsonl','topic-probe.jsonl')):
                try:
                    copied=command(['docker','cp',f'{run_name}:{source}',str(out/target)],
                                   check=False,timeout=60)
                    if copied.returncode:
                        (out/f'{target}.copy-error.txt').write_text(copied.stderr)
                except subprocess.TimeoutExpired as exc:
                    (out/f'{target}.copy-error.txt').write_text(str(exc)+'\n')
    print(json.dumps({'trial':args.trial_id,'output':str(out),
                      'containers':started},sort_keys=True))


if __name__=='__main__':main()
