# UCON — ROS 2 Jazzy

[![ROS 2 build and tests](https://github.com/Bling-sun/UCON/actions/workflows/ros2.yml/badge.svg)](https://github.com/Bling-sun/UCON/actions/workflows/ros2.yml)

无人机动态避障仿真，包含动态障碍物感知、占据地图、轨迹规划和 RViz 可视化。
本版本由 UCON1 工作区整理，完整源码在仓库内，不依赖其他 UCON 目录、ROS 1、CUDA 或外部传感器。

**默认是动态障碍物真值模式**：仿真器直接提供障碍物边界和运动状态，只显示无人机 4.5 米范围内的完整框。
这适合检查规划与显示效果；评估点云感知算法时，请使用下文的混合点云模式。

## 运行演示

![UCON ROS 2 动态避障仿真：无人机运动与 4.5 米范围内的动态真值框](assets/demo.gif)

实际 ROS 2 / RViz 运行录像，使用 60 个静态障碍物、36 个动态障碍物和默认随机种子。
青色圆环是以无人机为中心的 4.5 米感知球的水平截面；范围内显示完整动态真值框。
演示中的目标由脚本依次发送到与 **2D Goal Pose** 相同的话题。

## 环境与安装

- Ubuntu 24.04、ROS 2 Jazzy、系统 Python 3.12、C++17。
- 使用 CPU 点云渲染；图形界面需要 RViz2，自动测试可以无窗口运行。
- 首次编译建议预留 8 GB 内存；内存不足时降低下文的编译并行度。

先按 [ROS 2 Jazzy 官方安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)安装 ROS 2，
然后在新终端执行：

```bash
git clone https://github.com/Bling-sun/UCON.git
cd UCON
./scripts/install_dependencies.sh
./build.sh
./run_simulation.sh
```

安装脚本针对 Ubuntu 24.04 安装系统依赖，需要 `sudo`。不要在 Conda 环境中编译。
构建输出只写入当前仓库的 `build_ucon1/`、`install_ucon1/` 和 `log/`。
构建和启动脚本会清理其他 ROS 工作区的路径变量，避免意外引用旧版本。

内存有限时：

```bash
UCON_BUILD_WORKERS=1 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 ./build.sh
```

## 操作与参数

在 RViz 中选择 **2D Goal Pose**，点击并拖动设置目标；默认飞行高度为 0.4 米。
目标点只发送给规划器，不会在目标位置添加障碍物。按启动终端中的 `Ctrl+C` 停止。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `static_obstacles` | `60` | 随机静态障碍物数量（场地边界墙另外生成） |
| `dynamic_obstacles` | `36` | 动态障碍物数量 |
| `map_seed` | `3728542744` | 初始场景随机种子 |
| `use_ground_truth` | `true` | 直接读取动态障碍物真值；`false` 启用点云检测和 EKF |
| `truth_range` | `4.5` | 真值模式的感知球半径，单位米 |
| `target_z` | `0.4` | 手动目标的飞行高度，单位米 |
| `rviz` | `true` | 是否启动可视化窗口 |

```bash
# 无窗口运行
./run_simulation.sh rviz:=false

# 使用混合点云检测动态障碍物，不读取动态真值生成框
./run_simulation.sh use_ground_truth:=false

# 指定场景；相同种子与数量可重建相同初始地图
./run_simulation.sh static_obstacles:=60 dynamic_obstacles:=36 map_seed:=3728542744 truth_range:=4.5
```

障碍物边界进入感知球时显示整个框，完全离开后删除；范围会随无人机移动。
动态障碍物为深蓝色；静态障碍物为灰色，被占据地图检测到后整块显示为青灰色。
静态障碍物整块着色使用已知仿真几何辅助显示，不代表传感器观测到了全部表面。

## 验证与复现范围

```bash
./scripts/test.sh
```

测试使用独立的 `ROS_DOMAIN_ID=181`，不会向默认域中的仿真发送目标。
如果该域已被其他程序使用，可运行 `UCON_TEST_DOMAIN=182 ./scripts/test.sh`。
测试覆盖坐标系、占据膨胀层消退、完整高度跟踪、真值框及范围进出、静态着色、完整仿真启动和目标话题。
CI 使用相同的构建与测试入口。

重新录制演示（可选，需要已完成编译）：

```bash
sudo apt-get install ffmpeg xvfb python3-yaml
source /opt/ros/jazzy/setup.bash
source install_ucon1/local_setup.bash
python3 scripts/record_demo.py
```

录制使用独立的虚拟显示和 ROS 域 182，输出 `assets/demo.gif`；原始视频及日志位于 `log/demo/`。
可以用 `--domain` 更换空闲域、`--seconds` 指定录制时长。

固定种子保证本环境下初始几何可重复。动态运动和规划使用实时 ROS 调度，运行时长、系统负载及手动目标的点击时刻不同，
轨迹和消息时序可能不同；此仓库不声称默认真值仿真复现论文中的真实传感器实验或逐位相同的轨迹。
更详细的数据流见 [ROS2.md](ROS2.md)，验证记录见 [docs/VALIDATION.md](docs/VALIDATION.md)。

## 目录

```text
src/perception/          真值输入、点云检测、跟踪与感知可视化
src/obj_state_msgs/      障碍物状态消息
src/planner/             占据地图、路径搜索、轨迹优化和规划器
src/simulation/          ROS 2 地图生成、局部点云渲染、无人机模型及控制消息
src/ucon_sim/            仿真启动与静态地图显示
scripts/                依赖安装和自动测试入口
tests/                  回归及端到端测试
```

ROS 1 专用包、catkin 配置、旧 launch/RViz 文件和生成缓存已移除。仓库不包含本地构建产物、日志或个人配置。

## 论文与来源

[UCON: Uncertainty-aware Navigation with Historical Re-association in Dynamic Environments](https://arxiv.org/abs/2609.29419)

```bibtex
@article{sun2026ucon,
  title={UCON: Uncertainty-aware Navigation with Historical Re-association in Dynamic Environments},
  author={Sun, Bing and Lin, Yue and Yuan, Yongsheng and Liu, Yang and Wang, Dong and Lu, Huchuan},
  journal={arXiv preprint arXiv:2609.29419},
  year={2026}
}
```

感知和规划代码参考 [FAPP](https://github.com/arclab-hku/FAPP)。代码中保留原作者与第三方组件的版权、许可声明；
ROS 2 迁移后的默认仿真设置与上游实验条件不同，详见 [ROS2.md](ROS2.md)。
