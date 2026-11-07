"""The real local_frame_aligner.py against a fake localizer and a fake clear service: it aligns at the first map -> odom, leaves drift for the next stop, realigns at once only on a jump that stays, and clears the local costmap
after realigning. python3 ops/test_local_frame_aligner.py (needs ROS 2 and nav2_msgs); runs on its own domain."""
import math
import os
import subprocess
import sys
import time

os.environ["ROS_DOMAIN_ID"] = "91"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import rclpy  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav2_msgs.srv import ClearEntireCostmap  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
proc = subprocess.Popen([sys.executable, "-B", os.path.join(HERE, "local_frame_aligner.py"), "--ros-args", "-p", "clear_delay_s:=0.1", "-p", "still_s:=0.5", "-p", "persist_s:=0.3"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
rclpy.init()
n = rclpy.create_node("aligner_test")
br = TransformBroadcaster(n)
static, cleared = [], []
n.create_subscription(TFMessage, "/tf_static", lambda m: static.extend(t for t in m.transforms if t.child_frame_id == "odom_aligned"), QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_service(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap", lambda req, res: (cleared.append(time.time()), res)[1])
x = [0.0]


def spin(seconds, yaw, moving=False):
    t0 = time.time()
    while time.time() - t0 < seconds:
        if moving:
            x[0] += 0.03
        st = n.get_clock().now().to_msg()
        t = TransformStamped(); t.header.stamp = st; t.header.frame_id = "map"; t.child_frame_id = "odom"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = 3.0, -2.0, 0.57
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        b = TransformStamped(); b.header.stamp = st; b.header.frame_id = "odom"; b.child_frame_id = "base_footprint"; b.transform.translation.x = x[0]; b.transform.rotation.w = 1.0
        br.sendTransform([t, b])
        rclpy.spin_once(n, timeout_sec=.05)


def last_yaw():
    r = static[-1].transform.rotation
    assert (r.x, r.y) == (0, 0)
    assert all(v == 0 for v in (static[-1].transform.translation.x, static[-1].transform.translation.y, static[-1].transform.translation.z))
    assert static[-1].header.frame_id == "odom"
    return 2 * math.atan2(r.z, r.w)


def near(a, deg):
    return abs(math.atan2(math.sin(a + math.radians(deg)), math.cos(a + math.radians(deg)))) < 1e-6


try:
    spin(4, math.radians(30), moving=True)
    assert static and near(last_yaw(), 30), "the first map -> odom must align at once, moving or not"     # map -> odom_aligned has no rotation
    k, c = len(static), len(cleared)
    spin(2, math.radians(38), moving=True)                                  # drift while driving: it waits for a stop, however far it went under the jump limit
    assert len(static) == k and len(cleared) == c, "realigned while driving on a drift"
    spin(2, math.radians(38))                                               # stopped: now
    assert len(static) > k and near(last_yaw(), 38), "did not realign once parked"
    assert len(cleared) > c, "marks of the old alignment were left in the local costmap"
    k, c = len(static), len(cleared)
    spin(2, math.radians(38.5))                                             # parked, under its limit
    assert len(static) == k, "realigned parked under the limit"
    spin(0.1, math.radians(70), moving=True)                                # one wild sample while driving: an outlier, not a jump
    spin(1, math.radians(38.5), moving=True)
    assert len(static) == k, "a single outlier realigned and cleared"
    spin(2, math.radians(70), moving=True)                                  # a jump that stays (FAST-LIO restarted): at once, driving or not
    assert len(static) > k and near(last_yaw(), 70), "did not realign on a jump"
    k = len(static)
    spin(2, math.radians(-175))
    assert len(static) > k and near(last_yaw(), -175), last_yaw()
    k = len(static)
    spin(2, math.radians(178))                                              # 7 deg apart through the +-pi wrap: parked, realigned
    assert len(static) > k and near(last_yaw(), 178), "the wrap hid a 7 deg change"
    k = len(static)
    spin(1.5, math.radians(178.5))                                          # 0.5 deg: nothing
    assert len(static) == k
finally:
    proc.terminate()
    proc.wait(timeout=10)
print("ok")
