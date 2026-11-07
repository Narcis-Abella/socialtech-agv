"""Checks of the merged Nav2 parameters (no ROS needed): python3 ops/test_nav2_params.py"""
import os

import yaml

from merge_params import merge

here = os.path.dirname(os.path.abspath(__file__))
merged = {}
for path in (os.path.join(here, "..", "docker", "ros_nav", "socialtech_nav", "config", "nav2_params.yaml"), os.path.join(here, "nav2_robot_overrides.yaml")):
    with open(path) as f:
        merge(merged, yaml.safe_load(f))
local = merged["local_costmap"]["local_costmap"]["ros__parameters"]
glob = merged["global_costmap"]["global_costmap"]["ros__parameters"]
monitor = merged["collision_monitor"]["ros__parameters"]

# Spin and BackUp check poses in local_frame against the local costmap's cells without transforming (nav2_behaviors spin.cpp), and docking its trajectory in fixed_frame (controller.cpp): the same frame.
# Never map: a frozen localizer would freeze the pose without a TF error and RPP would keep driving (5.03 m in the sim against 0.46 m)
assert merged["behavior_server"]["ros__parameters"]["local_frame"] == merged["docking_server"]["ros__parameters"]["fixed_frame"] == local["global_frame"] == "odom_aligned"
assert local["resolution"] == 0.025, local["resolution"]

# The global costmap clears a cell only with rays that went through the heights it was marked at: a 2D scan loses the height, so a low object that leaves the Mid-360's field of view
# (-7 to +52 deg, 0.57 m up: a 25 cm box is out of it nearer than ~2.6 m) would be cleared by the rays that pass over it and hit the wall behind (the layer's test: socialtech_costmap/test).
# Nav2's VoxelLayer was tried for this and cleared badly in view: a ray only unmarks the voxels it crosses, and a column stays occupied while any voxel of it is marked
g, gs = glob["obstacle_layer"], glob["obstacle_layer"]["cloud"]
assert g["plugin"] == "socialtech_costmap/DecayObstacleLayer" and g["height_band_clearing"] is True and g["observation_sources"] == "cloud", g
assert gs["data_type"] == "PointCloud2" and gs["topic"] == "/cloud_registered_body" and gs["clearing"] and gs["marking"], gs
assert gs["min_obstacle_height"] < 0.0, gs   # the source keeps the floor: the rays that end on it are the evidence that frees a low object (marking is cut by the layer's own min_obstacle_height)
assert g["observation_expiry"] == 60.0 and "observation_expiry" not in local["obstacle_layer"], "what the lidar cannot see any more is remembered for 60 s (Narcis, 2026-11-04), then freed"
assert gs["sensor_frame"] == "lidar_level", "the ray starts at the lidar (0.57 m), not at the IMU body the cloud is published in (0.526 m)"

# Collision Monitor off is a pass-through only with its sources off too: an enabled source without fresh data orders a stop whatever the polygons say (collision_monitor_node.cpp process)
assert not any(monitor[p]["enabled"] for p in monitor["polygons"]), "polygons still enabled"
assert not any(monitor[s]["enabled"] for s in monitor["observation_sources"]), "a source can still stop the robot"
# odom is not at floor level and FAST-LIO's z drifts in it: the scan can sit below 0 in the local costmap's frame, so its filters are open (the real cut in height is scan.yaml). In map the base is on the floor: defaults
assert local["obstacle_layer"]["min_obstacle_height"] == local["obstacle_layer"]["scan"]["min_obstacle_height"] == -100.0, local["obstacle_layer"]
assert local["obstacle_layer"]["max_obstacle_height"] == local["obstacle_layer"]["scan"]["max_obstacle_height"] == 100.0, local["obstacle_layer"]
assert g["min_obstacle_height"] == 0.10 and g["max_obstacle_height"] == gs["max_obstacle_height"] == 1.8, g   # the band of scan.yaml (0.10-1.8 m): above it the cloud would mark door lintels (2.00 m in the sim world)
assert "expected_update_rate" not in local["obstacle_layer"]["scan"], "the costmap in odom needs no staleness limit: RPP's own plan transform stops it"
print("ok")
