# Day 1：工程扫描与召集场景定义

## 1. 执行边界m

本日只完成工程盘点、接口梳理、配置集中化和最小场景定义，不实现路径搜索、轨迹优化、控制器或多车协同算法。

原始 APA 工程保持只读。新代码只能通过适配器访问原工程函数，不能覆盖原始 `.slx`、`.m` 或生成文件。

论文资料中的 V-Hybrid A* 作为算法背景：未来状态使用 `(x, y, yaw, t)`，高优先级车辆可转换为低优先级车辆的动态障碍物。论文中的路径优化和速度优化不属于本日范围。

## 2. 原工程结构扫描

原始工程根目录：`F:\study\postgraduate\联培\路径规划论文\代码\APA`

关键文件和模块：

| 模块 | 位置/状态 | 结论 |
|---|---|---|
| 核心模型 | `F515_APA_Seres_V3p24_offline.slx` | 保持只读 |
| Local_Planner | 模型内 `Local_Planner/path_planning_alg` | MATLAB Function 嵌入 Stateflow XML，不是独立 `.m` |
| Hybrid A* | 模型内嵌套 `HybridAStar`、`HybridAStar_Update`、`CalcNextNode` | 可参考，不能直接作为独立召集接口 |
| ClassHybridAStar | `PlanningSubFcn/ClassHybridAStar.m` | 当前只有类框架，不是完整搜索器 |
| VehicleDynamic | `PlanningSubFcn/VehicleDynamic.m` | 单轨车辆运动学，可通过适配器复用 |
| VehicleCollisionCheck | `PlanningSubFcn/VehicleCollisionCheck.m` | 强依赖 APA 车位、全局配置和枚举，必须隔离 |
| Planner_Core | 模型内 `Planner_Core` 子系统 | 候选路径比较、缓存和路径管理，不属于召集搜索核心 |
| Geo_Planning1 | 模型内 `Geo_Planning1` 子系统 | 停车几何规划/后处理，暂不接入召集原型 |
| GeoPlanningSubFcn | `GeoPlanningSubFcn/` | 停车位、泊入/泊出和几何候选路径逻辑，暂不复用 |
| PlanningSubFcn | `PlanningSubFcn/` | 含车辆、路径、碰撞和 Hybrid A* 相关基础函数 |

## 3. Local_Planner 当前接口

### 3.1 当前输入

模型内 `path_planning_alg` 的输入按功能分组如下：

- 底盘：`Vel`、`Gear_Pos`。
- APA 状态：`APA_State`、`APA_Dir_tmp_In`、`APA_HandleSlotSW_In`、`APA_Stp_Num`、`APA_Stp_Now`、`APA_Ctrl_SubState`、`Replan_Req_Type`、`Third_Replan_Type`。
- 定位：`Veh_Pos`、`Navi_Pos_Reset_Idx`。
- 自由空间/边界：`FrSpc_Detected_Pts`、`FrSpc_Detected_Pts_N`、`FrSpc_Undetected_Cluster`、`BDPT_Traj_Pos_Num`、`BD_Pos_Reset_Idx`。
- 车位：`Slot_Pts_List`、`Edge_Enter_List`、`Tar_Pos_List`、`Slot_Track_Stopper_List`、`Slot_Status_List`、`Slot_Sub_Status_List`、`Slot_ID_List`、`Slot_N`、`Target_Slot_ID`、`Visual_Stopper_List`、`Visual_Stopper_N`、`SPT_Pos_Reset_Idx`、`slot_source`。
- 碰撞反馈：`MEB_Collision_Flag`、`MEB_Collision_US_ID`。

### 3.2 当前输出

- `Path_Valid_List`：每个候选车位的路径有效标志。
- `Path_Error_Type_List`：路径失败原因。
- `PathPts_X_List`、`PathPts_Y_List`、`PathPts_Tht_List`：路径点。
- `PathPts_OdoStp_List`：带方向的里程步长。
- `PathPts_delta_List`：转角/转向相关输出。
- `PathPts_N_List`：每条路径的有效点数。
- `Open_List`、`OpenValid_List`、`Close_List`、`CloseValid_List`：Hybrid A* 搜索调试记录。
- `gres_list`：每个候选车位采用的栅格分辨率。
- `delta_space`：跨周期保留的空间量。

### 3.3 关键数据结构

模型函数内部会构造：

- `st_chassis`：速度和挡位。
- `st_bd`：自由空间边界、轨迹点和坐标版本。
- `st_slot`：车位、目标位姿、车位状态和停止器。
- `st_state`：APA 主状态、泊车方向、重规划状态。
- `st_navi`：车辆位姿和坐标版本。
- `st_meb`：碰撞/脱困反馈。
- `st_obs`：边界线、分离点、停止器和虚拟边界。
- `map_bound`：搜索区域边界。
- `hybrid_node`：连续位姿、离散栅格索引、父节点、运动量和代价。
- `ClassPath`：路径点、路径长度、有效标志、误差码和路径来源。

召集原型 Day 1 只保留统一的 `VehicleState`、`SummonGoal`、`RoadBoundary` 和 `ScenarioConfig` 概念，不复制 APA 车位结构。

## 4. 参数盘点

### 4.1 已确认车辆参数

来源：`PlanningSubFcn/get_planner_global.m`。

- 车宽：1.855 m。
- 车长：4.763 m。
- 轴距：2.780 m。
- 后轴到车尾：1.007 m。
- 前轴到车头：0.976 m。
- 最小转弯半径：5.5 m。
- 最大前轮转角：0.4680017179 rad，即 26.81452324 deg。
- 默认前/后/侧安全裕度：0.2/0.2/0.2 m。

### 4.2 已确认 APA 地图/搜索参考参数

- 运动分辨率：0.1 m。
- XY 栅格分辨率：0.5 m。
- 航向栅格分辨率：5 deg。
- 运动步长：6.0 m。
- 碰撞检测步长：0.5 m。
- 航向范围：[-pi, pi]。
- 地图边缘像素：480。
- 地图边长参数：24 m。
- 转角采样数量：20。
- 启发函数权重：5。

这些参数目前只进入 `vehicle_config.json`、`map_config.json` 或 `planner_config.json` 的参考字段，不代表召集规划已经采用它们。

### 4.3 尚未确认的参数

召集道路边界、真实车辆初始位姿、召集点、时间分辨率、速度/加速度采样、安全时间裕度和静态障碍物均未从 APA 工程中确认，已在配置文件中标记为原型假设或 TODO。

## 5. 复用与隔离边界

### 可以复用，但必须适配

- `VehicleDynamic.m`：适配为无 APA 全局状态的车辆运动学接口。
- 车辆角点/矩形几何函数：适配为显式传入车辆尺寸。
- SAT 几何检测基础：只复用纯几何函数，不能复用车位语义。
- `ClassPath` 的路径数据表达方式：仅作为未来 MATLAB 输出格式参考。

### 必须隔离的停车专用逻辑

- `VehicleCollisionCheck.m` 中的车位边界、停止器、车位类别和 APA 碰撞类型。
- `ClassSlot`、`EnumSlotClass`、泊入/泊出状态和车位目标位姿处理。
- `Geo_Planning1` 与 `GeoPlanningSubFcn` 的车位几何规划。
- `Planner_Core` 的候选车位比较、缓存和 APA 重规划状态。
- `get_planner_global.m` 的 persistent APA 状态；召集配置不能依赖它作为隐式全局变量。
- Moco 控制接口和 APA 状态机。

## 6. 新工程结构

```text
summon_planner/
├── docs/       架构、接口和每日记录
├── config/     集中式 JSON 配置
├── matlab/     MATLAB 代码
├── tests/      单元测试
└── outputs/    可视化和测试输出
```

## 7. 当前输入输出与未来接口

当前 Day 1 输入是四个 JSON 配置：车辆、地图、规划器和场景。当前输出只有场景可视化图和配置检查结果，不生成路径。

未来 MATLAB 接口建议：

```matlab
scenario = loadSummonScenario("config/scenario_001.json");
plan = planSummonTrajectory(scenario);  % Day 1 不实现
```

未来 C++ 接口建议：

```cpp
ScenarioConfig scenario = LoadScenario("config/scenario_001.json");
TrajectoryPlan plan = planner.plan(scenario); // reserved
```

两类接口都应显式传递配置和状态，不依赖 APA 的 persistent global。

## 8. Day 1 测试边界

已执行的测试是 JSON 结构、三车数量、唯一 ID、起点/召集点完整性、边界闭合性、姿态差异和可视化文件存在性检查。由于 MATLAB 启动配置权限问题，本日不运行 Simulink 仿真，也不声称原模型仿真通过。
