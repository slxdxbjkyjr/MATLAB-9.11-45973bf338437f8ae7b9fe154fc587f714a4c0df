# Day 4：单车 V-Hybrid A* 核心节点扩展

## 1. 节点状态

Day 4 将普通 Hybrid A* 的连续状态从 `[x,y,theta]` 扩展为：

```text
[x, y, theta, v, t]
```

节点额外保存加速度、前轮转角、父节点和 `g/h/f` 代价，并保存五维离散索引：

```text
x_index, y_index, yaw_index, velocity_index, time_index
```

时间和速度索引进入 Closed 键，因此同一位置但不同时间、速度或航向不会被错误合并。

## 2. 新增文件

| 文件 | 作用 |
|---|---|
| `vhybrid_node.m` | 节点结构和五维索引字段 |
| `vhybrid_expand_node.m` | 3 个加速度 × 3 个转角采样、运动学推进、真实目标启发和静态碰撞裁剪 |
| `vhybrid_open_set.m` | 按最小 `f_cost` 取节点的 Open 集合 |
| `vhybrid_closed_set.m` | 五维状态索引查重的 Closed 集合 |
| `vhybrid_cost.m` | 距离、速度变化、航向变化、目标距离和时间代价 |
| `run_vhybrid_astar_demo.m` | Day 1 `vehicle_001` 单车搜索 Demo |

## 3. 节点扩展

每个节点采样：

- 加速度：`[-a_max, 0, +a_max]`；
- 前轮转角：`[-delta_max, 0, +delta_max]`。

每个组合通过 Day 3 的 `summon_vehicle_dynamic` 计算下一状态，再通过 `summon_collision_check` 检查道路边界和静态障碍物。当前场景没有多车动态障碍物。

## 4. 搜索流程

1. 初始化起点节点并放入 Open。
2. 取出 `f_cost` 最小节点。
3. 用五维离散索引加入 Closed。
4. 判断位置和航向终点容差。
5. 扩展 9 个控制组合。
6. 对未关闭节点计算代价并加入 Open。
7. 达到目标时沿 `parent_id` 回溯路径。
8. Open 为空、节点数超限或搜索时间超限时返回错误码。

错误码：`0` 成功，`2` 起点碰撞，`3` 终点碰撞，`4` Open 为空，`5` 最大节点数，`6` 最大搜索时间。

## 5. 代价函数

累计代价至少包含：

- 带方向行驶距离；
- 速度变化；
- 航向变化；
- 时间增量；
- 到目标位置/航向的启发距离；
- 到目标时间的统计项。

所有离散分辨率、搜索上限和权重集中在 `config/planner_config.json` 的 `vhybrid` 节，当前新增值标记为原型假设/TODO。

## 6. 输出

统一路径输出增加：

```text
path.x, path.y, path.theta, path.v, path.t, path.direction
path.valid, path.error_code, path.search_statistics
```

图像输出为 `outputs/day4_vhybrid_astar.png`，包含搜索节点、最终路径和路径速度/时间曲线。

## 7. 测试

`tests/test_vhybrid_core.m` 使用 `matlab.unittest.TestCase`，覆盖节点字段、五维查重、9 控制组合、代价项、Open 最小代价选择和 Demo 集成。当前阶段不包含多车动态障碍物、轨迹优化和控制器。
