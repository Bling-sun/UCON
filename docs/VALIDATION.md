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

`./build.sh` built all 12 packages. `./scripts/test.sh` passed eight test cases:

| Test | Behavior verified |
| --- | --- |
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

## Demonstration

`assets/demo.gif` is recorded from the actual RViz display by
`scripts/record_demo.py`, using the default seed, 60 static objects, 36 moving
objects and truth mode with a 4.5 m range. The script uses a separate ROS domain,
sends real planner goals, and checks that the drone travels before producing
the GIF. The recorded run lasts 24 seconds and travels approximately 23.8 m;
the GIF has 240 frames at 800 x 520 pixels (approximately 5 MB).
The caption, UAV label and sensing-range circle are display annotations.

## Limits

This verifies building and running the simulation, not the paper's real-world
sensor experiments. Truth mode bypasses dynamic detection and uncertainty
estimation. ROS scheduling and the time at which goals arrive can change moving
obstacle positions and planned trajectories; fixed seeds reproduce initial
geometry, not bit-identical runtime trajectories.
