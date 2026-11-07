"""Functional test of socialtech_costmap/DecayObstacleLayer's height-band clearing, on a stand-alone costmap node (node /costmap, topic /costmap/costmap).
usage: band_test.py EXPECT_BAND(0|1)   EXPECT_BAND=0 is the stock behaviour (a 2D ray clears whatever it passes over), 1 the band rule.

The cloud comes from a virtual Mid-360 at 0.57 m (elevation -7 to +30 deg, azimuth +-25 deg) in a world with a wall at x = 6 m, a floor, and a box or a person at x = 3 m, so
what the layer sees is what the lidar can see: a 25 cm box 1.9 m away is below the lowest ray (0.57 - 1.9 tan 7 deg = 0.34 m) and the rays that hit the wall behind it pass over it.
A cell is lethal when the published value is 100."""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

EXPECT_BAND = bool(int(sys.argv[1]))
LIDAR_Z, WALL_X = 0.57, 6.0
CELL = (2.92, 0.0)                            # the lidar sees the front face of the box (x = 2.9), not its middle
BOX = (2.9, 3.1, -0.1, 0.1, 0.0, 0.25)       # x0, x1, y0, y1, z0, z1: a 25 cm box
PERSON = (2.9, 3.1, -0.1, 0.1, 0.0, 1.7)
AZ = np.radians(np.arange(-25, 25.01, 0.5)); EL = np.radians(np.arange(-7, 30.01, 0.5))
D = np.stack(np.meshgrid(AZ, EL), -1).reshape(-1, 2)
DIR = np.stack([np.cos(D[:, 1]) * np.cos(D[:, 0]), np.cos(D[:, 1]) * np.sin(D[:, 0]), np.sin(D[:, 1])], 1)


def cast(sx, obj):
    """Points (sensor frame, axes of the map) of every ray's first hit within 12 m, and how many of them are on `obj`."""
    o = np.array([sx, 0.0, LIDAR_Z]); t_best = np.full(len(DIR), np.inf); on_obj = np.zeros(len(DIR), bool)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (WALL_X - sx) / DIR[:, 0]; p = o + DIR * t[:, None]; ok = (t > 0.3) & (np.abs(p[:, 1]) <= 4) & (p[:, 2] >= 0) & (p[:, 2] <= 2); t_best = np.where(ok & (t < t_best), t, t_best)
        t = -LIDAR_Z / DIR[:, 2]; ok = (t > 0.3) & (DIR[:, 2] < 0); t_best = np.where(ok & (t < t_best), t, t_best)
        if obj is not None:
            lo = np.array([obj[0], obj[2], obj[4]]); hi = np.array([obj[1], obj[3], obj[5]])
            a, b = (lo - o) / DIR, (hi - o) / DIR; tn = np.minimum(a, b).max(1); tf = np.maximum(a, b).min(1)
            ok = (tn <= tf) & (tf > 0.3); tb = np.maximum(tn, 0.3); hit = ok & (tb < t_best); on_obj = hit; t_best = np.where(hit, tb, t_best)
    keep = np.isfinite(t_best) & (t_best < 12)
    return DIR[keep] * t_best[keep, None], int(on_obj.sum())


rclpy.init(); n = rclpy.create_node("band_test"); br = TransformBroadcaster(n); grid = {"m": None}
for topic in ("/costmap", "/costmap/costmap"): n.create_subscription(OccupancyGrid, topic, lambda m: grid.update(m=m), 1)   # the stand-alone node publishes on one of the two
pub = n.create_publisher(PointCloud2, "/cloud", qos_profile_sensor_data)
state = {"x": -1.0, "obj": None, "zerr": 0.0}; results = []


def tf(stamp, child, x, z):
    t = TransformStamped(); t.header.stamp = stamp; t.header.frame_id = "map"; t.child_frame_id = child
    t.transform.translation.x = x; t.transform.translation.z = z; t.transform.rotation.w = 1.0; return t


def publish_pair():
    """TF and cloud carry the SAME stamp: a cloud stamped between two TF samples is interpolated and puts the sensor half way."""
    stamp = n.get_clock().now().to_msg()
    br.sendTransform([tf(stamp, "base_footprint", state["x"], state["zerr"]), tf(stamp, "lidar", state["x"], LIDAR_Z + state["zerr"])])   # zerr: the localizer's z estimate is off, everything lands zerr higher in map
    pts, _ = cast(state["x"], state["obj"])
    pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id="lidar"), pts.astype(np.float32)))


def run(seconds):
    t0 = time.time()
    while time.time() - t0 < seconds: publish_pair(); rclpy.spin_once(n, timeout_sec=0.05); time.sleep(0.05)


def lethal(x, y):
    m = grid["m"]
    if m is None: return None
    i = int((x - m.info.origin.position.x) / m.info.resolution); j = int((y - m.info.origin.position.y) / m.info.resolution)
    return m.data[j * m.info.width + i] == 100


def lethal_count():
    m = grid["m"]
    return None if m is None else sum(v == 100 for v in m.data)


def check(name, ok, detail=""):
    results.append(ok); print(f"{'PASS' if ok else 'FAIL'}  {name} {detail}")


run(2)   # the costmap only activates once map -> base_footprint exists
for tr in ("configure", "activate"):
    subprocess.run(["ros2", "lifecycle", "set", "/costmap", tr], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); run(1)
print("costmap state:", subprocess.run(["ros2", "lifecycle", "get", "/costmap"], capture_output=True, text=True).stdout.strip())

# the scenario must be physical: the box is seen from 4 m and is not seen from 1.9 m (the lowest ray passes over it)
check("0a the box is seen from 4 m", cast(-1.0, BOX)[1] > 0, f"({cast(-1.0, BOX)[1]} rays)")
check("0b the box is NOT seen from 1.9 m", cast(1.0, BOX)[1] == 0, f"({cast(1.0, BOX)[1]} rays)")
check("0c the person is seen from 1.9 m", cast(1.0, PERSON)[1] > 0, f"({cast(1.0, PERSON)[1]} rays)")

state.update(x=-1.0, obj=BOX); run(4)
m = grid["m"]; print("grid:", None if m is None else f"{m.info.width}x{m.info.height} res {m.info.resolution} origin {m.info.origin.position.x},{m.info.origin.position.y}, lethal cells {sum(v == 100 for v in m.data)}")
check("1 box seen from 4 m: marked", lethal(*CELL) is True)
state.update(x=1.0); run(4); got = lethal(*CELL)
check(f"2 robot 1.9 m from the box (the box left the lidar's view): {'kept (band rule)' if EXPECT_BAND else 'cleared by the rays that pass over it (stock)'}", got is EXPECT_BAND)
state.update(obj=None); run(4); got = lethal(*CELL)
check(f"3 the box is gone but the lidar cannot see that from 1.9 m: {'still marked (known limit)' if EXPECT_BAND else 'already clear (stock)'}", got is EXPECT_BAND)
state.update(x=-1.0); run(4); check("4 robot back at 4 m, rays at the box's heights pass through it: cleared", lethal(*CELL) is False)
state.update(x=1.0, obj=PERSON); run(4); check("5 a person (1.7 m) seen from 1.9 m: marked", lethal(*CELL) is True)
state.update(obj=None); run(4); check("6 the person leaves, the rays at its heights pass through: cleared at once", lethal(*CELL) is False)
state.update(x=-1.0, obj=BOX, zerr=0.20); run(4); check("7 box seen from 4 m while z is 0.20 m too high (the floor lands above the cut, the band of the box sits 0.20 m high): marked", lethal(*CELL) is True)
state.update(zerr=0.0, obj=None); run(8); n_leth = lethal_count()
check("8 z back to normal, box gone, seen from 4 m: the rays at the true heights (below the old band) free what the wrong z marked", lethal(*CELL) is False and n_leth <= 150, f"({n_leth} lethal cells; the wall alone is ~110)")
print("RESULT", "ALL PASS" if all(results) else "FAILURES"); rclpy.shutdown(); sys.exit(0 if all(results) else 1)
