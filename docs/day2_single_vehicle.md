# Day 2：独立单车 Local Planner 最小版本

## 1. 原工程入口

原 APA 模型中的真正入口是：

```text
Local_Planner/path_planning_alg
```

该 MATLAB Function 嵌入 `F515_APA_Seres_V3p24_offline.slx` 的 Stateflow XML。其内部包含嵌套的 `HybridAStar`、`HybridAStar_Update`、`CalcNextNode` 和终点解析扩展逻辑。`ClassHybridAStar.m` 本身只是类框架，不能作为完整搜索器直接调用。

Day 2 以这些核心算法为参考，重写为独立函数接口，而不是复制停车专用的完整 MATLAB Function。

## 2. 提取和隔离结果

### 保留的核心思想

- 连续车辆状态 `[x,y,theta]`。
- XY/航向离散栅格查重。
- 前进和倒车两组扩展方向。
- 离散前轮转角采样。
- `VehicleDynamic` 运动学自行车模型。
- 中间状态碰撞检查。
- `g+h` 代价、反向行驶惩罚和转角变化惩罚。
- 父节点回溯生成最终路径。

### 已隔离的停车专用逻辑

- 车位识别和 `ClassSlot`。
- 泊入/泊出状态机、APA 主状态和重规划状态。
- `Geo_Planning1`、`GeoPlanningSubFcn`。
- 车位质量比较、`Planner_Core` 缓存和候选车位选择。
- 超声波停止器、MEB 输入和自由空间总线。
- Moco、挡位控制和整车 APA 接口。
- `get_planner_global` 的 persistent 全局状态。

## 3. 新接口

```matlab
path = summon_hybrid_astar(start_state, goal_state, map, ...
    vehicle_config, planner_config);
```

输入：

- `start_state = [x,y,theta]`。
- `goal_state = [x,y,theta]`。
- `map`：`summon_map` 生成的边界和静态障碍结构。
- `vehicle_config`：车辆尺寸、轴距、最大转角和安全裕度。
- `planner_config`：集中式搜索参数。

输出：

- `path.x`、`path.y`、`path.theta`。
- `path.direction`：`-1` 倒车、`+1` 前进、首点为 `0`。
- `path.valid`。
- `path.error_code`：`0` 成功，`1` 输入错误，`2` 起点碰撞，`3` 终点碰撞，`4` Open 为空，`5` 达到扩展上限。
- `path.search_statistics`：扩展、生成、碰撞裁剪、查重裁剪、Open 峰值和耗时。

## 4. 文件说明

| 文件 | 作用 |
|---|---|
| `summon_vehicle_state.m` | 显式参数车辆状态和运动学适配器 |
| `summon_map.m` | 地图配置解析和边界整理 |
| `summon_hybrid_astar.m` | 单车 Hybrid A* 及核心局部函数 |
| `summon_collision_check.m` | 旋转矩形越界和静态障碍检查 |
| `summon_plot_path.m` | 路径图和车辆外形可视化 |
| `run_single_vehicle_demo.m` | 读取 Day 1 场景并规划 vehicle_001 |

## 5. 参数策略

Day 1 已确认的车辆尺寸和 APA 参考参数仍集中在 `config/vehicle_config.json`、`config/map_config.json` 和 `config/planner_config.json`。Day 2 新增的搜索分辨率、扩展上限和终点容差集中在 `planner_config.search`，并标记为原型假设/TODO；没有散落在算法代码中。

## 6. 测试状态

已执行 `tests/check_day2_single_vehicle.ps1`，通过文件、接口、核心局部函数、配置和中文注释静态检查。

当前环境已使用 MATLAB R2023a 实际执行：

```matlab
addpath(genpath('F:/study/postgraduate/联培/路径规划论文/代码/辅助/9.11/matlab'));
path = run_single_vehicle_demo();
```

实际结果：

- `valid = 1`，`error_code = 0`。
- `expanded_nodes = 593`。
- `generated_nodes = 10656`。
- `collision_pruned_nodes = 0`。
- `elapsed_sec ≈ 4.085590`。
- 输出图：`outputs/day2_single_vehicle.png`。

该结果只验证独立单车搜索原型，不代表原 APA Simulink 模型仿真通过。当前版本不包含多车协同、时空碰撞、轨迹优化和控制器。
