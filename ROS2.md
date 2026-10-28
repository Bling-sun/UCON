# ROS 2 simulation and data flow

The supported workspace targets Ubuntu 24.04 and ROS 2 Jazzy. All 12 packages
under `src/` are native ROS 2 packages. ROS 1 packages, launch files, generated
Python messages, and unused ROS 1 rendering code have been removed.

Use `./build.sh`, `./run_simulation.sh`, and `./scripts/test.sh` from the repository
root. No sibling workspace is needed. For direct `ros2` commands, source
`/opt/ros/jazzy/setup.bash` followed by `install_ucon1/local_setup.bash`.

## Default simulation

`ros2 launch fapp_planner sim_planner.launch.py` starts the complete stack:

```text
map_generator -> /map_generator/obj_truth -> perception -> /box_edge, /states
map_generator -> local_sensing_node -> /cloud_registered_static -> perception -> /static_map
/static_map -> planner occupancy map -> /grid_map/occupancy_inflate -> static_map_display
fapp_planner -> /planning/trajectory -> traj_server -> /position_cmd
/position_cmd -> poscmd_2_odom -> /Odometry
```

The map has a 30 x 30 m footprint, 60 random static objects, 36 moving objects,
and separate boundary walls. Its default seed is `3728542744`. Moving objects
publish at 20 Hz. The local CPU renderer has an 8 m sensor horizon. Full height
truth boxes are selected within a 4.5 m sphere about the drone: an obstacle is
included when its AABB intersects that sphere. These two ranges serve different
inputs and can therefore differ.

`2D Goal Pose` publishes `/move_base_simple/goal`, consumed only by the planner.
The manual target height defaults to 0.4 m. The map generator's optional
insertion input is a separate `/map_generator/add_obstacle` PoseStamped topic;
it is not bound to the RViz navigation tool.

## Truth mode (default)

The map generator computes each object's complete bounds from the same points
published on `/map_generator/obj_cloud`. `/map_generator/obj_truth` carries
those bounds and finite-difference velocities with the same sample timestamp.
The fixed array order is the stable object ID; objects are not reordered or
deleted by the generator.

Perception selects objects in `truth_range` and directly draws their bounds,
with 0.05 m padding for the rendered 0.1 m voxels. No DBSCAN, association or EKF
filtering is applied in this mode. Lost markers receive explicit DELETE
updates. `/states` uses truth positions and velocities, retaining the planner's
existing planar projection to z = 0.4 m and zero vertical velocity. Covariance
is zero in this mode; it must not be treated as a sensor uncertainty estimate.

Static observations are rendered independently from the static map and sent to
`/cloud_registered_static`. Perception forwards that cloud to `/static_map`
and `/map_ros`, preserving its acquisition timestamp. Static points are not
removed merely because a moving obstacle overlaps them.

## Mixed-cloud perception

Run `./run_simulation.sh use_ground_truth:=false` to enable dynamic detection
and EKF tracking from `/cloud_registered_body`. The renderer still publishes
truth for the simulator, but the perception node does not subscribe to it.

The mixed cloud retains points from -2 to 5 m height. World-frame input is not
transformed by the drone pose a second time. Motion estimation uses actual
sample intervals; observed bounds drive boxes independently of filtered motion
states. Tracked object bounds are removed from the cloud sent to static mapping.
Dynamic classification, occlusion, clustering and missed observations affect
this mode, so its output differs from the default truth-assisted simulation.

To run perception with external point clouds and odometry, use
`ros2 launch perception perception.launch.py` and adapt its topics, frame IDs
and height filters. Physical sensor drivers are not part of the tested simulation.

## Display

RViz uses `world` as its fixed frame. Moving obstacles are dark blue. Static
geometry is grey; once occupancy touches a connected object, its complete
surface is colored muted teal by `/ucon_sim/static_display`. This is a display
aid using known simulator geometry, not an expansion of the planner's map.
Each surface is drawn once to avoid coincident point clouds flickering.

Raw cloud, raw/inflated occupancy and DBSCAN layers are available as optional
debug views. Their default visibility is off. The drone mesh is installed by
the `poscmd_2_odom` package itself.

## Additional launches

- `ros2 launch perception sim_perception.launch.py`: map, drone, perception and RViz.
- `ros2 launch plan_env sim_grid_map.launch.py`: standalone occupancy-map node.
- `ros2 launch ucon_sim sim.launch.py`: a small kinematic input/monitor smoke demo,
  separate from the default randomized scene and full planner.

## FAPP reference

The comparison used [FAPP commit dbd8173](https://github.com/arclab-hku/FAPP/tree/dbd8173bd1518419b5e33faa7eecb75ed9d9cdcd).
Its [mapping implementation](https://github.com/arclab-hku/FAPP/blob/dbd8173bd1518419b5e33faa7eecb75ed9d9cdcd/src/mot_mapping/src/mapping_manger.cpp)
receives separated moving-object points on `/map_generator/obj_cloud` and
draws current matched detection bounds while estimating motion with an EKF.
UCON's default mode instead takes per-object truth bounds directly; the
optional mixed-cloud mode detects from static and dynamic points together.
These input conditions must be distinguished when comparing performance.
