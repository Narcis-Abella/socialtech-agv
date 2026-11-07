costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint
    rolling_window: false
    width: 20
    height: 20
    origin_x: -10.0
    origin_y: -10.0
    resolution: 0.05
    track_unknown_space: false
    robot_radius: 0.2
    update_frequency: 5.0
    publish_frequency: 5.0
    always_send_full_costmap: true
    plugins: [obstacle_layer]
    obstacle_layer:
      plugin: socialtech_costmap/DecayObstacleLayer
      enabled: true
      min_obstacle_height: 0.10
      max_obstacle_height: 2.0
      @BAND@
      observation_sources: cloud
      cloud: {topic: /cloud, data_type: PointCloud2, marking: true, clearing: true, raytrace_max_range: 10.0, raytrace_min_range: 0.0, obstacle_max_range: 10.0, obstacle_min_range: 0.0, min_obstacle_height: -0.5, max_obstacle_height: 2.0}   # the source keeps the floor: its rays are the evidence that frees a low object; marking is cut at the layer's 0.10
