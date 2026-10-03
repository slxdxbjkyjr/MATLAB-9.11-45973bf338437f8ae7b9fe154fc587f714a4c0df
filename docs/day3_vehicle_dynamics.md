# Day 3：带速度和时间的单车运动状态

## 1. 原 VehicleDynamic 检查

原文件：`F:\study\postgraduate\联培\路径规划论文\代码\APA\PlanningSubFcn\VehicleDynamic.m`。

原函数输入为当前 `[x,y,theta]`、有符号距离 `D`、前轮转角 `delta` 和轴距 `WB`，输出下一时刻 `[x,y,theta]`。小转角使用直线近似，非零转角使用圆弧模型。它不包含速度、加速度和时间，也不负责速度或转角约束。

Day 3 新函数没有覆盖原文件，而是新增 `summon_vehicle_dynamic.m`。

## 2. 新状态和单位

```text
state = [x, y, theta, v, t]
```

| 量 | 单位 | 说明 |
|---|---|---|
| x, y | m | 地面坐标 |
| theta | rad | 车头航向角，输出归一化到 [-pi, pi) |
| v | m/s | 纵向速度，可通过 v_min/v_max 支持前进或倒车 |
| t | s | 累积时间 |
| delta | rad | 前轮转角 |
| a | m/s^2 | 纵向加速度 |
| dt | s | 时间步长 |
| L | m | 轴距 |
| curvature | 1/m | `tan(delta)/L` |

建议后续搜索节点扩展为结构体：

```matlab
state.x; state.y; state.theta; state.v; state.t;
state.parent_id; state.g_cost; state.h_cost; state.f_cost;
```

当前 `summon_vehicle_dynamic` 使用固定顺序数值向量，搜索节点元数据仍由上层节点结构管理。

## 3. 统一接口

```matlab
[next_state, travelled_distance, curvature, valid_flag] = ...
    summon_vehicle_dynamic(current_state, delta, a, dt, L, ...
    v_max, v_min, a_min, a_max, delta_max);
```

运动逻辑：

1. `v_next = v + a*dt`。
2. `travelled_distance = (v + v_next)/2 * dt`。
3. `curvature = tan(delta)/L`。
4. 直线时按航向积分；转弯时按圆弧积分。
5. `t_next = t + dt`。
6. 若速度、加速度、转角或输入不满足约束，返回当前状态、零距离、零曲率和 `valid_flag=false`。

## 4. 配置来源

Day 3 的默认动力学约束集中在 `config/planner_config.json` 的 `dynamics` 节：

- `dt = 0.1 s`；
- `v_min = 0 m/s`、`v_max = 5 m/s`；
- `a_min = -2 m/s^2`、`a_max = 2 m/s^2`；
- `delta_max = 0.4680017179 rad`，来自 APA 车辆最大转角。

速度和加速度范围目前是召集原型假设，已标记 TODO，不能视为实车标定值。

## 5. 测试和输出

测试文件：`tests/test_vehicle_dynamic.m`，采用 `matlab.unittest.TestCase`，覆盖直行加速、直行减速、左转、右转、零转角、超速、超转角和加速度越界。

轨迹图生成函数：`tests/generate_vehicle_dynamic_plot.m`。

Day 3 运动学图：`outputs/day3_vehicle_dynamic.png`。

MATLAB R2023a 实际测试结果：

```text
8 Passed, 0 Failed, 0 Incomplete
trajectory_samples = 56
final_state = [9.6753, 3.7442, 0.2005, 0.5000, 5.5000]
```

本日仍未接入多车时空碰撞、速度优化或 Simulink 控制器。

## 6. Day 2 召集路径接入

新增 `tests/generate_day3_summon_plot.m`，默认读取 Day 2 场景中的
`vehicle_002`，调用 `summon_hybrid_astar` 生成空间路径，再调用
`summon_vehicle_dynamic` 为路径点生成受速度和加速度约束的时间参数。

输出文件：

- `outputs/day3_summon_path_vehicle_002.png`：Day 3 召集空间路径图；
- `outputs/day3_summon_vt_vehicle_002.png`：带方向速度的 v-t 图；
- `outputs/day3_summon_vehicle_002.mat`：路径、时间和速度数组。

该版本仍是单车原型。速度参数化不改变 Day 2 的空间路径，也不代表实车标定；
真实召集阶段的速度、加速度和倒车速度约束仍需后续实测数据确认。

### 6.1 目标点停车修正

`generate_day3_summon_plot.m` 现在根据路径剩余距离计算允许的停车速度
`sqrt(2*abs(a_min)*remaining_distance)`，在目标点将速度设置为 `0 m/s`，
并重新按相邻路径点平均速度积分时间。因此生成的 v-t 曲线在终点归零，表示车辆
到达召集点后停车。运行输出会打印 `terminal_speed=0.000000 m/s` 作为验收依据。

### 6.2 速度优化与物理连续性

速度不再直接跳变到巡航值。程序采用基于路径弧长的前向/后向约束传播：
前向传播限制 `a <= a_max`，后向传播限制 `a >= a_min`；前进和倒车方向
切换位置被设置为停车点。每段速度通过 `summon_vehicle_dynamic` 反算时间并校核，
最终输出会打印加速度范围，例如 `acceleration_range=[-2.000000, 2.000000]`。

### 6.3 换挡时间

`config/planner_config.json` 的 `dynamics.direction_change_time_s` 设为 `0.5 s`。
每次前进/倒车切换时，车辆先在换挡点保持 `v=0` 等待该时间，再进入下一段；
换挡等待不会被错误地计入车辆加速度，v-t 图中会显示水平的零速等待段。
