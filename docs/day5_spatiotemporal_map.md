# Day 5：X-Y-T 时空资源块与占用接口

## 完成内容与范围

本日新增独立时空资源模块，保存 `occupancy(ix,iy,it)`、占用类型、所有者、原因和时间层坐标；支持永久静态膨胀、车辆旋转矩形、时间插值、层内扫掠、多占用者查询以及四视图。Day4 的搜索器、车辆运动学和静态几何适配器直接复用，没有接入动态搜索。没有实现 Day6 顺序避碰或论文第四节轨迹优化。

## 文件与调用关系

| 文件 | 职责 |
|---|---|
| `st_occupancy_map.m` | 通过集中配置创建三维资源数组和归属注册表 |
| `validate_st_occupancy_config.m` | 范围/分辨率/半开约定/搜索时间步兼容性及内存上限检查 |
| `inflate_vehicle_occupancy.m` | 后轴中心参考的非对称矩形，包含车辆已有裕度和额外安全距离 |
| `mark_static_obstacles.m` | 边界和静态障碍沿所有时间层占用 |
| `mark_trajectory_occupancy.m` | 插值、时间层切段、平移/旋转扫掠和归属写入 |
| `query_occupancy.m` | 查询占用、类型、外部所有者ID、块索引及全部重叠归属 |
| `st_resource_index.m` | 标记和查询共用的半开索引转换 |
| `st_vehicle_cells.m` | 分离轴定理判断整个栅格块与旋转矩形相交 |
| `st_polyline_distance.m` | 到多边形全部线段的最短距离，包含长边中部 |
| `st_write_occupancy.m` | 维护主所有者及重叠块完整归属，不覆盖已有信息 |
| `plot_st_occupancy.m` | 真正的XY/XT/YT切片及三维示意 |
| `tests/test_st_occupancy_map.m` | 独立的MATLAB类测试 |
| `tests/generate_st_occupancy_plot.m` | 用Day1车辆2/3和Day4真实轨迹演示接口 |
| `tests/validate_day5_spatiotemporal.m` | 运行Day5及Day3/4回归，静态分析并生成验证JSON |

```text
planner_config → validate_st_occupancy_config → st_occupancy_map
vehicle_config → inflate_vehicle_occupancy
summon_map → mark_static_obstacles → st_polyline_distance → st_write_occupancy
Day4 path → mark_trajectory_occupancy → st_vehicle_cells → st_write_occupancy
query_occupancy → st_resource_index → 所有者/原因查询
plot_st_occupancy → XY、XT、YT、三维占用图
```

## 配置和坐标约定

空间、时间边界和 `dx/dy/dt` 集中在 `planner_config.json.st_occupancy`。原型设置：空间 `[0,30)×[0,20) m`，时间 `[0,60) s`，`dx=dy=0.5 m`，`dt=0.5 s`。资源数组为 `60×40×120`，共288000块。`vhybrid.time_step_s=0.5 s`，与dt相等；以后可取dt的正整数倍。非整数分辨率跨度和不兼容时间步会报错，不悄悄追加大小不一致的末块。

每个资源块是 `[x_edges(ix),x_edges(ix+1))`、`[y_edges(iy),y_edges(iy+1))`、`[t_edges(it),t_edges(it+1))`。`x_coords/y_coords/t_coords` 只表示中心。查询使用floor，内部边界属于后一块，最大边界不属于地图。`t=0` 是第一时间块 `[0,0.5)`，中心为0.25s。浮点吸附与最后一块保护在共用转换函数处理。

`safety_distance_m=0` 表示本日不额外增加安全距离；已有车辆配置中前/后/侧0.2m安全裕度仍生效。扫掠最大时间间隔0.05s、最大车角运动间隔0.1m。设置标记为原型假设，可集中调整。

## 元数据与查询接口

```matlab
st = st_occupancy_map(planner_config, geometry);
st = mark_static_obstacles(st, geometry, vehicle_config);
st = mark_trajectory_occupancy(st, path, vehicle_config, 'vehicle_002');
[occupied, type, id, index, detail] = query_occupancy(x,y,t,st);
```

`occupied`为logical；`type`为`free/boundary/static_obstacle/dynamic_vehicle/multiple_owners/out_of_bounds`；`id`为首个占用者外部ID；`index=[ix,iy,it]`。越界或非有限坐标返回占用=true、永久不可行原因；无效维度索引不得用于数组访问。

地图中的 `owner_type` 是uint8类别码（0自由、1边界、2静态、3动态、4多归属）；`owner_id` 是uint32注册表索引。`owners(k).id/type/reason`保存可读ID和类型。`reason`数组保存与类别对应的原因码，字符串说明在`reason_labels`。普通块使用稠密主归属；重叠块在`overlaps`中保存全部注册表索引。查询的 `detail.owner_ids/owner_types/reasons`提供全部归属，`block_bounds`给出实际区间，`permanent`指出有无永久静态/边界占用。

例如两个车辆同时占据同一块，首个车辆ID不会丢失，查询返回`multiple_owners`并列出两个ID；动态车辆进入静态障碍时，静态ID也保留。地图和重叠信息均为MATLAB值结构，复制后修改不会污染原图。

## 静态膨胀

车辆位置使用后轴中心。局部矩形前向长度是 `wheelbase+front_axle_to_front+front_margin+extra`，后向是 `rear_axle_to_rear+rear_margin+extra`，半宽是 `width/2+side_margin+extra`。不能把后轴当作矩形中心，也不能直接用总车长的一半。

三维资源没有yaw维，静态占用采用该非对称矩形最远角点的全航向保守包络半径 `hypot(max(front,rear),half_width)`。障碍到任意边的距离小于半径即禁行；边界向内收缩相同半径，再加入半个空间块对角线，保证整块的后轴参考点安全。这比针对已知航向的精确矩形判断更保守。支持简单凹边界，不把静态障碍限制为四个顶点；自交多边形和带孔地图暂不支持。

## 动态插值、矩形栅格化与扫掠

输入是Nx5 `[x,y,theta,v,t]`或带相同字段的Day4路径结构。状态必须有限、速度非负、时间严格递增；单点轨迹允许。重复/倒序时间、失配字段或失败路径明确报错，不排序/删除点来隐藏异常。时间必须完整位于配置半开范围，后轴参考点不能越出空间范围。

先对yaw解环绕，沿相邻最短角差插值，再线性插值x/y/v/t。每个轨迹段在时间层边界切开；每层检查中心、端点和中间姿态。矩形与整个空间块按四轴SAT相交，接触也占用，所以不会因“块中心未进入车身”而漏掉相交块。

扫掠采样数同时由时间和车角位移上界决定。给定一小段参考点移动D、航向变化A、最远车角半径R，任何车角移动量不超过 `D+R*abs(A)`。对n个等间隔取样，最近样本最大误差不超过 `(D+R*abs(A))/(2*n)`。在每个样本的局部纵横矩形上增加这段误差，可保守覆盖连续线性插值的整个扫掠，包含旋转穿越。

这保证覆盖的是输入轨迹的插值模型。占用模块不会为稀疏数据推断真实车辆加速度或重新验证运动学；Day4密集输出通过既有模型验收后再传入，Day6仍需对候选轨迹做同一运动学推进。

## 时间端点策略

`trajectory_end_policy=release`：只标记输入轨迹覆盖的时间块；端点按半开约定进入自己的块，之后的块释放。每个被标记时间块保守占满整层，因此开始前/结束后最多一个dt范围仍可能判定占用。末点恰在层边界时，前一层含扫掠末端，后一层含真实末点，这是刻意的保守处理。同空间、不同时间若落在不同时间块不冲突；若落在同一块仍可保守判冲突。

`hold_until_t_max`：要求末速度为零，停车矩形持续占用至地图时间范围结束。该选项为Day6召集点驻车预留；默认release用于本日“轨迹时间范围”验收。不能把release理解为真实车辆停车后消失。

## 图像和运行

```matlab
project = 'F:\study\postgraduate\联培\路径规划论文\代码\辅助\day 4\MATLAB-9.11-45973bf338437f8ae7b9fe154fc587f714a4c0df';
addpath(fullfile(project,'matlab','single_vehicle'));
addpath(fullfile(project,'tests'));
results = runtests(fullfile(project,'tests','test_st_occupancy_map.m'));
report = generate_st_occupancy_plot();
% 完整验收：Day5测试 + Day3/4回归 + 静态分析 + 真实轨迹占用图
validation = validate_day5_spatiotemporal();
```

图的左上是固定时间块的XY占用，右上是固定y块的XT切片，左下是固定x块的YT切片，右下是X-Y-T三维示意。黑色表示永久边界膨胀、灰色表示静态障碍膨胀、车辆使用独立颜色、红色表示多归属。色标列出车辆ID。XT/YT是实际切片，不使用沿另一轴求any的投影；三维点抽样只影响显示，不影响占用查询。

Demo读取Day1的车辆2/3，分别执行原Day4搜索，均从t=0开始，再写入同一时空图。轨迹允许冲突，图展示占用与冲突接口，不是Day6避碰结果。原场景静态障碍为空，图只展示边界静态占用；独立测试通过矩形/长边/凹边界构造静态障碍验收，没有向原场景添加障碍。

输出为 `outputs/day5_st_occupancy.png`、`day5_st_occupancy.mat` 与本次测试报告。MAT保存资源图、来源路径和图示报告，支持查询复核。

## 测试结果、当前问题和Day6建议

最终测试计数、实际运行版本、耗时和原工程文件校验见 `outputs/day5_validation.json`。本次执行MATLAB R2023a，测试覆盖元数据、半开边界及浮点邻界、整数时间步、静态连续性、时间分离、多归属、后轴旋转矩形、薄块相交、平移/旋转扫掠、角度环绕、端点驻车策略及错误输入。

2026-10-04在用户指定F盘目标目录执行，实际结果如下：

| 验证项 | 结果 |
|---|---|
| Day5时空资源测试 | 54/54通过 |
| 已有Day3运动学回归 | 8/8通过 |
| 已有Day4核心回归 | 17/17通过 |
| 总计 | 79通过，0失败，0未完成 |
| Day5核心11个函数Code Analyzer | 0条消息 |
| 复制到目标目录的16个指定文件 | 哈希一致 |
| 除planner_config外任务开始已有的67个文件 | 全部哈希不变，包括已有Day4源码和输出 |

车辆2轨迹时间范围为0–10.7606389825s，车辆3为0–18.1639205525s。两条独立轨迹写入后，共180715个占用块、1125个多归属块；其中226块含两辆动态车，其余为静态边界保守包络与动态车身重叠。它们是占用接口报告的冲突候选，不能将资源块数当成真实碰撞次数。

原始配置字节备份保存在工作区 `day5_implementation/deployment_backup/planner_config_before_day5.json`。配置仅新增`st_occupancy`并更新stage，原`search/dynamics/vhybrid`与功能开关保持原值；未接入动态搜索。完整文件保护记录见 `outputs/day5_source_integrity.json`。已有Git工作区改动被保留，未提交或推送。

当前问题：静态全航向包络会保守拒绝部分靠墙姿态，与Day4按真实yaw精确检查不同；时间量化也会引入同层假阳性；尚未接入动态搜索。Day6必须区分“已膨胀的静态后轴禁行图”和“动态车身占用”：前者可查询后轴参考点，后者必须检查自车矩形及运动段，不能把单点动态查询当整车碰撞，也不能再次用自车矩形对静态膨胀图做同样膨胀。未来可保留原静态几何进行按yaw的精确复核。

下一天：通过适配器把高优先级轨迹加入搜索，明确停车驻留策略，检查加速/减速中间状态，并实现车辆2/3的顺序规划；保持论文第四节轨迹优化和控制器在当前范围之外。
