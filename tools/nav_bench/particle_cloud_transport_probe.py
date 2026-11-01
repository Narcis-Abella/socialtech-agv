#!/usr/bin/env python3
"""Isolate ROS/DDS delivery of AMCL-sized ParticleCloud messages.

Two independent processes use the same publisher and subscriber QoS as Nav2
AMCL and the external observer. No map, bag, localization or truth is involved.
"""
import json
import multiprocessing as mp
import time


def receive(queue,duration_s):
    import rclpy
    from nav2_msgs.msg import ParticleCloud
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init();node=Node('particle_transport_receiver')
    received=[]
    qos=QoSProfile(depth=16,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(ParticleCloud,'/transport_probe_particle_cloud',
                             lambda m:received.append(len(m.particles)),qos)
    until=time.monotonic()+duration_s
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
    queue.put(received)
    node.destroy_node();rclpy.shutdown()


def main():
    import rclpy
    from nav2_msgs.msg import Particle,ParticleCloud
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    q=mp.Queue();child=mp.Process(target=receive,args=(q,12.))
    child.start();time.sleep(1)
    rclpy.init();node=Node('particle_transport_sender')
    qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
    pub=node.create_publisher(ParticleCloud,'/transport_probe_particle_cloud',qos)
    deadline=time.monotonic()+5
    while pub.get_subscription_count()<1 and time.monotonic()<deadline:
        time.sleep(.1)
    matched=pub.get_subscription_count()
    for count in (2000,3000,5000,10000):
        message=ParticleCloud()
        message.header.frame_id='map'
        message.particles=[Particle(weight=1./count) for _ in range(count)]
        for _ in range(3):
            pub.publish(message);time.sleep(.4)
    node.destroy_node();rclpy.shutdown()
    child.join(timeout=15)
    if child.is_alive():
        print(json.dumps({'error':'subscriber_timeout','matched':matched}));return
    received=q.get(timeout=2)
    print(json.dumps({'matched':matched,'sent_per_size':3,'sizes':[2000,3000,5000,10000],
                      'received_particle_counts':received},sort_keys=True))


if __name__=='__main__':main()
