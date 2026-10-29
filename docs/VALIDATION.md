# Reproduction validation

Validation date: 2026-10-29.

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

`./build.sh` built all 12 packages. `./scripts/test.sh` passed 23 test cases:

| Test | Behavior verified |
| --- | --- |
| `test_demo_recording.py` (12 cases) | Piecewise polynomial evaluation, exclusion of goal switches and stationary objects, rejection of actual contact, deduplication of obstacle IDs, remaining-plan coverage, stopping at future goal changes, body/voxel bounds, crossings between samples, interior odometry and complete recording coverage |
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
display by `scripts/record_demo.py`: seed `3728542750`, 110 static objects,
72 moving objects and truth mode with a 4.5 m range. Recording uses a private
ROS domain and sends each goal only after reaching the previous one (0.35 m
tolerance). The demo uses a 3.5 m/s velocity limit, a 0.2 s safety-replan
interval and a 1.5 m dynamic-clearance trigger. Normal interactive defaults
remain 60/36 objects, seed `3728542744`, 3.5 m/s, 0.5 s and 1.0 m respectively.
These settings are explicit recorder arguments and launch parameters.

The raw video lasts 23.83 seconds. `setpts=(PTS-STARTPTS)/2` produces an
11.90-second GIF: 238 frames at 20 fps, 800 x 520 pixels, about 7.23 MB.
The run reaches all four goals and travels 33.00 m. Its 70 successful safety
replans respond to predicted conflicts; they do not count physical collisions
or distinct avoided obstacles. The overhead view and orange body ring help
separate the UAV from its teal flown path and red planned path.

The recorder saves time-stamped odometry, complete object bounds/velocities,
polynomial trajectory coefficients and goal events in `log/demo/telemetry.json`.
It encloses the displayed drone mesh in a 0.21 m radius sphere and expands
obstacle AABBs by 0.05 m on each axis to cover displayed voxels. For each pair
of truth frames, it checks the obstacle motion envelope against the drone's
motion envelope, including every interior odometry sample. Telemetry must
bracket the entire recorded interval. This is a conservative geometric check
of recorded motion, including linear/held motion between frames, rather than
certification of unobserved motion.

The minimum dynamic body-clearance lower bound is **0.203 m**. The sampled
static body-clearance lower bound, after subtracting the drone radius and
voxel half-diagonal, is **0.105 m**. Center-to-dynamic-AABB and center-to-static
point minima are 0.509 m and 0.401 m. The 477 truth frames have a maximum gap
of 0.056 s; the maximum odometry gap is 0.019 s. Frames around the closest
dynamic and static encounters were also visually inspected.

To distinguish avoidance from a goal-induced turn, an offline check excludes
goal changes and evaluates each replaced polynomial against subsequently
observed obstacle positions until the old plan ends or the next goal changes.
It identifies four distinct moving objects for which the previous plan would
enter a 0.25 m buffer while the actual path stays outside:

| Generator object ID | Time after recording starts | Previous-plan clearance | Flown clearance at the same instant |
| --- | --- | --- | --- |
| 1 | 6.97 s | 0.000 m | 0.622 m |
| 33 | 9.77 s | 0.092 m | 1.360 m |
| 63 | 10.62 s | 0.000 m | 1.217 m |
| 6 | 20.82 s | 0.017 m | 0.509 m |

IDs are stable truth-array indices. This retrospective check supports the
selected demonstration; it is not a prediction-accuracy benchmark. Eleven
moving objects pass within 1.5 m; proximity alone is not counted as avoidance.

The recorder rejects runs unless all four goals are reached, at least two
distinct avoidance conflicts are verified, and at least three moving objects
pass within 1.5 m. It requires dynamic body clearance of at least 0.15 m,
nonnegative sampled static body clearance, center clearances above 0.25 m,
bounded telemetry gaps and successful safety replans. Rejected runs retain
raw data under `log/demo/` without replacing the published GIF. Metrics and
the GIF SHA-256 are in [`assets/demo.metrics.json`](../assets/demo.metrics.json).

## Limits

This verifies building and running the simulation, not the paper's real-world
sensor experiments. Truth mode bypasses dynamic detection and uncertainty
estimation. ROS scheduling and the time at which goals arrive can change moving
obstacle positions and planned trajectories; fixed seeds reproduce initial
geometry, not bit-identical runtime trajectories.
