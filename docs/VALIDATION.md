# Reproduction validation

Validation date: 2026-10-28.

## Environment

The cleaned source tree was copied to a separate directory without `build/`,
`install/`, logs, or any generated files. All 12 packages were built with an
empty environment except the system PATH, home directory and UTF-8 locale.
No sibling UCON workspace was sourced. The generated install tree had no
symlinks to the original working directory.

| Component | Tested version |
| --- | --- |
| OS | Ubuntu 24.04.3 LTS, x86_64 |
| ROS | Jazzy |
| Python | 3.12 |
| Compiler | GCC 13 |
| `ros-jazzy-rclcpp` | `28.1.18-1noble.20260412.041931` |
| `ros-jazzy-pcl-conversions` | `2.6.4-1noble.20260412.045759` |
| PCL development package | `1.14.0+dfsg-1` |
| Eigen development package | `3.4.0-4build0.1` |
| OpenCV development package | `4.6.0+dfsg-13.1ubuntu1` |
| NumPy | Ubuntu package `1:1.26.4+ds-6ubuntu1` |
| SciPy | Ubuntu package `1.11.4-6build1` |

`scripts/install_dependencies.sh` installs supported distribution packages;
it does not lock the apt repository to these exact versions. The CI workflow
repeats the build and headless tests in the official Jazzy ROS base container.
Current CI results are available in the repository's Actions tab.

## Checks passed

`./build.sh` built all 12 packages. `./scripts/test.sh` passed 16 test cases:

| Test | Behavior verified |
| --- | --- |
| `test_demo_recording.py` (5 cases) | Piecewise polynomial evaluation, exclusion of goal switches and stationary objects, rejection of actual contact, deduplication of obstacle IDs |
| `test_graph_discovery.py` (3 cases) | Wait for all subscriber names to resolve; preserve unexpected and duplicate subscribers for strict validation |
| `test_grid_map_inflation.py` (2 cases) | Persistent occupancy does not blink; stale occupancy and inflation expire together |
| `test_ground_truth_perception.py` | 36 overlapping objects keep separate IDs; complete bounds, 4.5 m range, drone movement, marker deletion and independent static input |
| `test_mapping_frames.py` | World points remain invariant under drone motion; body points are transformed once |
| `test_perception_extent.py` | Full-height boxes, measured velocity at varying sample rates, removal from the static map and marker cleanup in mixed-cloud mode |
| `test_scene_seed.py` | Fresh generators with the same seed produce identical static point clouds; a different seed changes the map |
| `test_simulation_smoke.py` | Complete default scene starts; a navigation goal produces a trajectory and drone movement without changing map geometry |
| `test_static_map_display.py` | Whole-object tint, stable updates, occupancy removal and late-subscriber data |

The optimizer's intersection handling was corrected so a hit for a previous
control point cannot cause an uninitialized intersection to be consumed for
the current control point. Test subprocesses are cleaned up after each run.

The simulation smoke test waits for ROS graph node names as well as endpoint
discovery before checking that the navigation goal has exactly one subscriber,
`fapp_planner_node`. DDS can report a matching endpoint with the temporary name
`_NODE_NAME_UNKNOWN_` before its ROS node metadata arrives. This caused the
[initial demo-update CI failure](https://github.com/Bling-sun/UCON/actions/runs/40672197317).
The wait remains bounded at 20 seconds; unknown names are not accepted as valid,
and unexpected or duplicate subscribers still fail the test.

## Demonstration

`assets/demo.gif` is a selected successful run recorded from the actual RViz
display by `scripts/record_demo.py`. It uses seed `3728542744`, 60 static objects,
72 moving objects and truth mode with a 4.5 m range. The normal interactive
launch remains at 60 static / 36 moving objects. The script uses a private ROS
domain and sends the next goal only after reaching the previous one (0.35 m
tolerance). Camera framing is closer than the previous demonstration.

The raw video lasts 23.8 seconds. Applying `setpts=(PTS-STARTPTS)/2` produces
an 11.9-second GIF: 238 frames at 20 fps, 800 x 520 pixels, about 8 MB.
The run reaches all four goals, travels 32.97 m and has 16 successful safety
replans. These replans are responses to predicted conflicts, not physical
collisions, and their count is not a count of distinct avoided obstacles.

The recorder saves time-stamped odometry, complete object bounds/velocities,
polynomial trajectory coefficients and goal events in `log/demo/telemetry.json`.
It interpolates odometry to truth timestamps and computes center-to-AABB
Euclidean distance. In this clip, 477 truth samples have a maximum gap of
0.055 s; the minimum dynamic distance is 0.564 m. The minimum center-to-static-
point distance is 0.314 m. These are sampled geometric checks, not continuous
collision certification of the drone mesh.

To distinguish avoidance from a goal-induced turn, the offline check excludes
goal changes and evaluates each replaced polynomial against the **subsequently
observed** obstacle positions for up to two seconds, stopping at the old plan's
end. It finds two distinct moving objects for which retaining the previous
plan would enter a 0.25 m buffer while the actual flown path stays outside:

| Generator object ID | Time after recording starts | Previous-plan clearance | Flown clearance at the same instant |
| --- | --- | --- | --- |
| 9 | 9.02 s | 0.000 m | 0.669 m |
| 17 | 13.97 s | 0.192 m | 0.761 m |

IDs are stable indices in the generator truth array. This retrospective check
supports the selected demonstration; it is not a prediction-accuracy benchmark.
Five moving objects pass within 1.5 m, but proximity alone is not counted as
avoidance. The GIF caption and UAV label are display annotations.

The script rejects runs unless all four goals are reached, at least two distinct
avoidance conflicts are verified, at least three moving objects pass within
1.5 m, and sampled dynamic/static clearances stay above 0.25 m. It also checks
telemetry gaps and successful safety replans. Failed runs leave their raw data
under `log/demo/` without replacing the published GIF. Published metrics and
the GIF SHA-256 are in [`assets/demo.metrics.json`](../assets/demo.metrics.json).

## Limits

This verifies building and running the simulation, not the paper's real-world
sensor experiments. Truth mode bypasses dynamic detection and uncertainty
estimation. ROS scheduling and the time at which goals arrive can change moving
obstacle positions and planned trajectories; fixed seeds reproduce initial
geometry, not bit-identical runtime trajectories.
