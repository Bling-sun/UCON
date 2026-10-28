# 参数与录制

基本安装与启动见 [README](../README.md#快速开始)。以下命令均在仓库根目录执行。

## 构建选项

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
默认录制 110 个静态、72 个动态障碍物，种子为 `3728542750`，最多 40 秒，到达全部目标后结束；`--speed 2` 控制播放倍速。
可以用 `--domain` 更换空闲域、`--seconds` 设置最长录制时间、`--static-obstacles`、`--dynamic-obstacles` 和 `--map-seed` 调整场景。
脚本保存同步真值、里程计、轨迹及核验结果；仅在目标到达、多障碍物避让和采样间距检查均通过后覆盖 GIF，
并生成带 GIF 校验和的 `assets/demo.metrics.json`。未通过的运行会保留日志并返回失败。

固定种子保证本环境下初始几何可重复。动态运动和规划使用实时 ROS 调度，运行时长、系统负载及手动目标的点击时刻不同，
轨迹和消息时序可能不同；此仓库不声称默认真值仿真复现论文中的真实传感器实验或逐位相同的轨迹。
更详细的数据流见 [ROS2.md](../ROS2.md)，验证记录见 [VALIDATION.md](VALIDATION.md)。

