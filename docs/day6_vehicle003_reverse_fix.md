# vehicle_003 倒车起步与初始冲突修复（2026-10-08）

## 完成内容与根因

修复前 `scenario_001.json` 的车2起点为 `[27,17,pi,0,0]`，车3起点为
`[24,15,0,0,0]`，车3终点为 `[4,17,0,0,0]`。原失败报告是错误码9
`dynamic_start_collision`，扩展节点、采样控制均为0；本次失败并非搜索超时。

车型宽1.855m，每辆车侧向裕度0.2m，安全矩形宽2.255m。两车起点的
纵向车身投影相交，横向后轴间距2m，实际车身净距仅0.145m；各侧0.2m
裕度叠加要求至少0.4m净距，因此安全矩形重叠0.255m。真实车身尚未接触，
但已违反当前安全配置。瞬时碰撞报告的 `physical_collision=false` 与
`safety_rectangle_collision=true` 可以同时成立。实际扫掠资源图还共享22块。
倒车、加速、调搜索预算均不能消除已有的t=0安全冲突。保留起点拒绝检查。

另外，旧搜索仅初始化前进挡根节点。Day6要求立即运动，又禁止该根生成
换挡等待边，车3因此不能直接选择从静止倒车起步。这是独立的搜索限制。

## 修改文件与行为

| 文件 | 修改作用 |
| --- | --- |
| `config/planner_config.json` | 增加 `vhybrid.initial_gear_mode="auto"`，原 `initial_gear=1` 保留为首选插入顺序 |
| `config/scenario_001.json` | 根据用户最新确认，将车3起点y从15m改为13.5m，其余场景参数保持原值 |
| `matlab/single_vehicle/summon_vhybrid_astar.m` | 真正静止时同时初始化前进/倒车根；共享一次搜索，由挡位键区分；增加选择结果和初始冲突证据 |
| `matlab/single_vehicle/check_trajectory_conflict.m` | 单一时刻诊断不使用高车未来最大速度产生的扫掠不确定膨胀；区间检测、资源预留保持原判据 |
| `matlab/single_vehicle/run_day6_sequential_planning_demo.m` | 支持完整场景副本和独立输出目录；控制台打印初始冲突证据或成功倒车统计，保留分阶段计时 |
| `config/scenario_vehicle003_reverse_feasible.json` | 独立对照场景，仅把车3起点y设为13.5m，保留其余起终点、车型与裕度 |
| `matlab/single_vehicle/run_vehicle003_reverse_demo.m` | 调用同一Day6 Demo和搜索器验证独立场景；不手工指定路径或起步挡位 |
| `tests/test_reverse_initial_gear.m` | 14项自动选挡、固定覆盖、非零速度、节点预算、原始安全冲突及粗栅格冲突测试 |
| `tests/test_reverse_search.m` | 原换挡测试夹具显式使用fixed模式，继续验证运行中停稳后等待0.5s |
| `tests/validate_day6_sequential.m` | 将新增测试加入原Day6验收入口 |

自动选挡只发生在起步前，两个根节点均为原位姿、v=0、原t=0、g=0，且没有
虚构等待边。显式 `options.initial_gear` 可以锁定挡位；`fixed` 模式兼容旧语义。
任何非零速度起点（包括1e-7m/s）不允许瞬间自动反向。运行中换挡仍要先停车，
并静止等待 `dynamics.direction_change_time_s=0.5s`，期间继续检查占用。

## MATLAB实测

原有212项测试、新增14项测试全部通过（226/226）。完整对照Demo由真实搜索
生成轨迹，已输出轨迹图、速度图、GIF、MAT和JSON：

| 指标 | 2026-10-08工作区验证结果 |
| --- | --- |
| 车2起步/到达 | t=0、v=0、前进挡；11.0045s到达 `[10,8,pi]`，终速0 |
| 车3起步/到达 | t=0、v=0、倒挡；15.5447s到达 `[4,17,0]`，终速0 |
| 车3实际倒车距离 | 20.3393m；路径全程倒挡，运行中换挡0次 |
| 车3搜索扩展/生成 | 776 / 13154；动态拒绝2362 |
| 两车动力学 | 原模型逐点重放通过；没有直接覆盖终点位置或航向 |
| 动态/独立资源块交集 | 两种检查均为0块 |
| 采样最小真实车身净距 | 0.9522m；计入采样运动不确定后的保守下界0.7774m |
| 计算耗时 | 15.154s，高车0.148s、地图0.044s、低车14.592s、独立验收0.300s |
| 图/GIF/总运行 | 1.923s / 9.499s / 26.997s；机器和暖机状态会影响时间 |

部署后直接在用户指定F盘目标工程再次运行 `validate_day6_sequential(true)`：
226/226通过、主Demo通过；计算18.074s（低车搜索17.394s），绘图1.350s、
GIF动画9.358s、文件输出0.420s，总运行29.204s。两车轨迹及净距与上表一致。
该次准确计时保存在目标 `outputs/day6_sequential_report.json` 和
`outputs/day6_validation.json`；MAT顶层 `timings` 同步记录最终计时。
部署前被覆盖文件备份于 `.codex_backups/vehicle003_reverse_fix_20261008_103627/`。
哈希核验的72个未列入修改范围的文件全部保持原样。

仅移到y=14.3m可消除起点重叠，但下一时间层高车转弯的扫掠资源仍挡住车3
第一段扩展；因此不能把“当前矩形已分开”等同于完整场景可行。y=13.5m
是本次已完整验证的对照位置，不宣称是所需间距的数学最小值。

## 运行与图片解释

在工程根目录执行：

```matlab
addpath(fullfile(pwd,'matlab','single_vehicle'));
main = run_day6_sequential_planning_demo(true);
comparison = run_vehicle003_reverse_demo(true);
results = runtests('tests');
```

用户已确认将主场景y改为13.5m，第一行Demo现应完成真实规划。第二个Demo
使用独立集中场景配置，并将图和数据单独保存。若手工改回y=15m，则仍应
明确返回错误码9；新增测试保留这一原始不可行情形，不能通过放宽碰撞判据
或调速度来宣称原始布置成功。

独立输出在 `outputs/vehicle003_reverse_feasible/`，其JSON的
`scenario_override_used=true`、`input_states.low_start=[24,13.5,0,0,0]`明确记录
实际测试条件。主报告 `scenario_override_used=false`，也是y=13.5m；两个
成功报告均不能解释成修复前y=15m的布置成功。

图中蓝色为车2前进轨迹，橙色为车3倒车轨迹。XY曲线相交不代表相同时刻
发生碰撞，必须看绝对时间和车身占用。速度图画 `gear*v`，橙线负值表示倒车，
公共状态v仍非负；橙线中途回到零并短暂停留是避让高车的交通等待，路径
没有运行中换挡，不能把这段等待误当固定0.5s换挡。加减速均来自运动学控制。

## 当前问题与后续建议

主场景的唯一位置调整已由用户明确确认。安全裕度、静态地图、搜索120s预算及
原APA/Day3模型保持原值。
算法返回加权代价搜索中首先接受的可行轨迹；倒车惩罚仍为2，不保证全局
最短时间。后续可单独研究启发式与代价选择，不在本次实现联合或速度优化。
