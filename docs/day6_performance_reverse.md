# Day6运行效率、五档转角与倒车扩展

部署复验：已于2026-10-07将验证版本合并到用户指定F盘工程，确认实际调用的是该目录中的MATLAB函数。目标目录一键验收212项全部通过，失败/未完成均为0；两车Demo计算44.152s、PNG绘图1.647s、GIF动画11.784s、主文件输出0.533s、总运行58.124s。狭窄通道倒车验证也在目标目录通过：实际倒车10m、停稳换挡一次且等待0.5s。下文重复计时中位数仍以四组正式对照数据为准。覆盖前的原文件保存在目标目录`.codex_backups/day6_performance_reverse_20261007_214725/`。

更新日期：2026-10-07。本次在既有顺序多车搜索基础上调整运行效率、转角采样和倒车能力。仍采用高优先级车辆先搜索、低优先级车辆查询高车资源块的策略，没有进行连续轨迹优化、完整速度优化或优先级重新排序。

## 1. 性能检查与比较方法

本次检查使用用户当前的场景和配置。车辆2目标为`[10,5,pi]`，车辆3目标为`[24,15,pi/2]`；连接距离8m、参考速度2m/s、单车搜索计算预算60s均保留。两辆车依然从`t=0、v=0`同时起步。这里的结果不能与旧报告中车辆2目标`[15,9,pi]`的结果直接比较。

优化前暖机后的实际计算时间为60.7464s，其中低优先级车辆搜索58.8376s。该次搜索成功，耗时已接近60s搜索预算；单独打开MATLAB profiler后总计算约61.3328s，低车触发预算超时。Profiler本身会增加运行开销，因此不能把profile超时直接理解为数学无解，也不能用有profile与无profile的耗时作加速比。

| 优化前profile热点 | 累计时间，约s | 解释 |
| --- | ---: | --- |
| `vhybrid_expand_node` | 59 | 低车候选控制、运动积分和碰撞检测集中在此 |
| `day6_static_collision_check` | 23.8 | 约21.1万次静态采样检测，重复几何整理和通用多边形开销较高 |
| `check_trajectory_conflict` | 27.89 | 每条候选边执行时空资源查询 |
| `day6_resource_blocks` | 18.56 | 时间切层、矩形扫掠和资源索引合并 |
| 原矩形栅格化辅助函数 | 8.83 | 候选车身与大量整块逐一判定 |
| `query_occupancy` | 5.7 | 重复资源归属查询与报告构造 |

这些是**包含子调用的累计时间**。例如资源栅格化已经包含在资源查询、动态检查和节点扩展时间内，不能把表中的时间相加当作总耗时。瓶颈主要是低车搜索中的大量候选边，而非高车规划或一次性配置读取。

Profiler的`SelfTime`进一步定位函数自身开销；它不包含其子函数时间，应与上述累计时间分开解读：

| 优化前self热点 | 自身时间，s | 调用次数 |
| --- | ---: | ---: |
| `inflate_vehicle_occupancy` | 7.0427 | 221828 |
| `inpolygon`内部向量判定 | 6.1521 | 211333 |
| `query_occupancy` | 5.5773 | 6230 |
| `st_vehicle_cells` | 4.3530 | 292530 |
| `day6_resource_blocks` | 3.6678 | 9862 |
| `day6_static_collision_check` | 3.3698 | 211101 |
| `ndgrid` | 3.3565 | 见profile原始记录 |

这说明车辆尺寸/裕度重复处理、通用多边形检查，以及大量自由栅格的构造和查询，是优化中需要直接减少的工作。

性能基准使用同一场景、车辆、地图、搜索预算和安全采样，分四组比较：

| 组 | 代码 | 转角档数 | 倒车 | 比较目的 |
| --- | --- | ---: | --- | --- |
| `baseline_3_forward` | 优化前 | 3 | 关闭 | 原实现基线 |
| `optimized_3_forward` | 优化后 | 3 | 关闭 | 隔离实现效率改进 |
| `optimized_5_forward` | 优化后 | 5 | 关闭 | 隔离转角采样变化 |
| `optimized_5_reverse` | 优化后 | 5 | 开启 | 最终功能配置 |

每组先暖机一次，再至少交替重复3次；偶数轮反向执行四组，减小运行顺序和首次函数加载带来的影响。比较记录`wall_compute_s`，包含完整独立安全验收，但关闭PNG、GIF和本次Demo结果文件输出。失败、超时和异常均保留，同时报告成功率，不能将提前失败产生的短运行时间解释为成功加速。旧代码没有配置覆盖接口，基准把只读旧源码复制到临时目录并写入同一当前配置，原基线和用户工程均不覆盖。

### 重复实测结果

以下结果来自MATLAB R2023a实际运行。增量报告确认4次暖机、12次正式运行和4组汇总完整产生；每组3次正式重复均规划成功并通过完整安全验收。原始数据见`outputs/day6_performance_comparison.json`。

| 组 | 成功次数/重复次数 | 计算时间中位数，s | 低车搜索中位数，s | 低车扩展节点 | 低车生成节点 |
| --- | --- | ---: | ---: | ---: | ---: |
| 原3档前进 | 3/3 | 59.970386 | 58.337505 | 1293 | 6889 |
| 新3档前进 | 3/3 | 24.772079 | 23.216510 | 1293 | 6889 |
| 新5档前进 | 3/3 | 43.846479 | 42.200422 | 1575 | 14925 |
| 新5档可倒车 | 3/3 | 47.869331 | 46.211772 | 1600 | 15429 |

同为3档前进时，纯实现优化将计算时间降低58.7%，约为原来的2.42倍速度；两版低车扩展/生成节点数相同。对包含第二轮缓存复用优化的最终冻结版本再次真实调用，两车轨迹长度均与原版一致，逐点对照的`x/y/theta/v/t`最大差全部为0，确认同3档配置的轨迹及速度时间数据完全一致。这说明这一组加速来自等价实现的重复计算减少。新3档到新5档的实际计算增加77.0%，与控制候选及搜索节点增加相符。新5档开启倒车额外增加9.2%。最终默认5档可倒车的计算中位数比原3档前进仍减少20.2%。这些比例比较的是完整算法计算，不包含图像与动画输出。

全部正式运行的搜索资源交集和独立Day5资源写入交集均为0块。原/新3档前进的采样最小车身净距为1.122715m，新5档前进/可倒车为1.015491m。净距属于采样测量，不替代各次报告中的连续保守下界。本次普通两车场景最终仍选择前进路线，开启倒车会搜索倒车候选，但不强制输出倒车；必须倒车的特殊场景另作实际验证。

3档到5档后，每个挡位的转角×加速度候选从`3×5=15`增加为`5×5=25`。单次节点展开工作增加，但更细的转角可能更早找到可行路径、减少总扩展节点，所以总耗时必须实测。倒车同样扩大搜索空间，罚项会引导优先使用前进，但不能保证开启倒车之后一定更快。

## 2. 效率改进及保留的安全检查

`prepare_static_context.m`在每次搜索开始缓存车辆矩形、道路边界、静态多边形和包围盒。缓存同时保存车辆尺寸、安全裕度及原始边界/障碍；任一数据变化时静态缓存失效并重建，不能继续使用旧地图或旧车辆几何。轴对齐矩形道路使用精确车角范围判定，凸边界保留角点包含检测，凹边界仍检查车身边穿界；障碍包围盒仅用于排除明显分离，最终多边形判据保留。

既有`st_vehicle_cells.m`本来就采用旋转矩形与整块的四轴SAT，本轮没有修改该文件。新增的加速在`day6_resource_blocks`搜索模式：按时间层只对`dynamic_layer_cells`中**已经被高车占用的块**批量执行四轴SAT，避免为每个候选姿态构造周围全部自由块、生成完整索引集后再取交集。世界X/Y轴和车身纵/横轴的不等式与原栅格化一致，端点、层内扫掠姿态、采样密度及间隙padding保留；接触仍判冲突。最终`collect=true`验收继续收集完整车身资源集，不用稀疏首命中结果代替完整统计。

动态上下文缓存每层包络、稀疏占用块中心和归属索引。`st_write_occupancy`每次写入后删除旧动态mask、包络、块中心和owner缓存，下次检查从新占用归属重建；不能在加入新车辆后漏掉其资源。快速包络筛选只排除必然分离的候选，包络可能相交时仍执行完整旋转矩形资源判据。

Open集合改为稳定二叉最小堆，减少每次取最小节点的线性扫描。相同代价保持插入顺序，Open/Closed状态索引包含挡位，不能把相同位姿、速度和时间但不同挡位的节点合并。

`vhybrid_expand_node`把本条控制边已经通过原运动学积分得到的`static_states`传给`day6_motion_trajectory`。动态采样所需时刻如果已存在于同一控制边，则复用该模型样本；不存在的时刻仍调用原模型积分，不用位姿插值替代运动学。挡位适配器的参数检查也减少通用排序/集合函数开销，保留非法挡位与速度约束检测。

加速、匀速、减速和换挡驻留仍在加入Open前检查静态及动态约束；终点连接和到达后驻留同样检查。整条结果路径最后进行独立运动学验收、矩形净距采样及Day5双车资源写入复核。没有通过增大碰撞采样步长、取消矩形判定、延迟出发或改变目标来获得加速。

绘图复用同一次独立安全验收的`distance_samples`，避免再次计算全部车间矩形距离。动画图元只创建一次，后续原地更新；GIF首帧生成统一256色色表，后续帧只映射颜色，避免每帧重新量化。另存RGB全色PNG预览。`day6.save_animation=false`时仍输出轨迹PNG和速度PNG，但不生成GIF。

## 3. 五档转角集中配置

`planner_config.json`中的`vhybrid.control_steering_samples_rad`从

```text
[-delta_max, 0, delta_max]
```

改为

```text
[-delta_max, -delta_max/2, 0, delta_max/2, delta_max]
```

当前`delta_max=0.4680017179rad`，对应数组为`[-0.4680017179,-0.23400085895,0,0.23400085895,0.4680017179]`。采样值仍接受原车辆最大前轮转角约束；此处是前轮转角，不是车身航向角。加速度候选保留`[-2,-1,0,1,2]m/s²`。

## 4. 倒车运动学、换挡和罚项

公共状态继续使用`[x,y,theta,v,t]`。`x/y`是后轴中心，`theta`是实际车身航向，`v>=0`是速率，`t`是绝对任务时间。独立`gear=+1/-1`表示前进/倒车；带方向速度为`gear*v`。倒车时不能将实际车身航向直接旋转180°来伪装运动状态。

新增`summon_vehicle_dynamic_gear.m`作为原Day3模型的挡位适配器，原`summon_vehicle_dynamic.m`不覆盖。运动方向通过挡位传入原模型，转向遵守相同轴距与曲率关系，倒车的带方向弧长为负。倒车时正加速度增加速率，负加速度降低速率；停车和速度上限仍按非负速率验证。

换挡是一条独立的静止时间推进边。只有车辆先减速到0，才允许使用零加速度、零位移、固定航向的换挡边；该边等待`dynamics.direction_change_time_s=0.5s`后进入另一挡位，随后再按受限加速度起步。换挡过程中车身继续占用资源块，不能因为停止运动而跳过动态碰撞检查。回溯保留`path.gear`和`path.is_gear_change`，换挡边不细分成容易丢失事件标记的普通运动点。

| 配置字段 | 当前值 | 含义 |
| --- | ---: | --- |
| `vhybrid.reverse_enabled` | `true` | 是否允许搜索倒车 |
| `vhybrid.initial_gear` | `1` | 初始挡位，默认前进 |
| `vhybrid.reverse_penalty` | `2.0` | 倒车距离代价倍率，前进距离倍率为1 |
| `vhybrid.gear_switch_penalty` | `0.5` | 每次换挡增加的独立搜索代价，**不是时间单位** |
| `dynamics.direction_change_time_s` | `0.5` | 实际静止换挡等待时间，秒 |
| `day6.save_animation` | `true` | 保存结果时是否生成GIF |

设前进的距离代价为`weight_distance*abs(ds)`，倒车时变为`weight_distance*reverse_penalty*abs(ds)`；换挡边另外累计时间代价和`gear_switch_penalty`。倒车倍率2和换挡代价0.5属于可调整原型参数，没有声称是论文或实车标定值。

同挡位终点连接支持倒车：只将运动切向转换为车身航向加pi来计算连接几何，再通过挡位适配模型积分真实车身状态；连接中不隐式换挡，不直接赋值末端航向。当前支持控制采样与同挡位连接组合，不宣称已经实现完整Reeds–Shepp解析连接或全局最优混合挡位搜索。

`vhybrid.max_reverse_speed_mps`为可选独立倒车速率上限；没有配置该字段时沿用现有速度上限。最终测试记录应以部署后的实际JSON为准。

搜索统计新增倒车节点、换挡节点/拒绝数，以及`path_reverse_distance_m`、`path_gear_switches`和`path_gear_wait_time_s`。速度图用`gear*v`显示前进为正、倒车为负；速度大小`path.v`本身依然非负。零速附近的窄色带表示真实换挡等待区间，动画箭头保持实际车身航向。

## 5. 运行入口和计时定义

在MATLAB中切换到用户目标工程：

```matlab
project_root = 'F:\study\postgraduate\联培\路径规划论文\代码\辅助\day 4\MATLAB-9.11-45973bf338437f8ae7b9fe154fc587f714a4c0df';
addpath(fullfile(project_root,'matlab','single_vehicle'));
addpath(fullfile(project_root,'tests'));

% 正常运行，输出图像、GIF及成功/失败报告，并打印计算时间：
result = run_day6_sequential_planning_demo(true);

% 只计算及完整安全验收，关闭所有本次Demo文件与图像输出：
result = run_day6_sequential_planning_demo(false);
disp(result.timings);

% 保留PNG与JSON/MAT，仅关闭GIF：
cfg = jsondecode(fileread(fullfile(project_root,'config','planner_config.json')));
cfg.day6.save_animation = false;
result = run_day6_sequential_planning_demo(true,struct('planner_config',cfg));

% 本机保存的优化前只读工程；至少3轮重复：
baseline_dir = 'C:/Users/lenovo/Documents/ChatGPT/长安汽车路径规划/day6_oct7_baseline';
comparison = benchmark_day6_performance(baseline_dir,project_root,3);

% 狭窄通道中使用实际车型和真实搜索验证倒车及换挡：
reverse_demo = generate_reverse_validation_plot(true);

% Day3/4/5/6与倒车测试、真实两车Demo一键验收：
summary = validate_day6_sequential(true);
```

`options.planner_config`必须是完整配置结构体；它只覆盖本次运行的内存副本，场景、地图和车辆仍从当前工程读取，不回写用户JSON。

成功或明确规划失败结束后均打印计时，单位为秒：

| `result.timings`字段 | 计时范围 |
| --- | --- |
| `configuration_s` | 配置读取与场景/几何准备 |
| `high_search_s` | 高车搜索及该搜索器内的回溯/验收 |
| `resource_map_s` | 静态资源表、高车扫掠/驻留标记、动态上下文 |
| `low_search_s` | 低车搜索及该搜索器内的回溯/验收 |
| `independent_validation_s` | Demo独立矩形/资源安全验收与起终状态检查 |
| `low_resource_statistics_s` | 成功低车资源写入统计 |
| `compute_s` | 以上算法计算总时间，排除PNG/GIF及结果文件IO |
| `plot_s` | PNG绘图与导出 |
| `animation_s` | GIF创建、帧渲染、颜色映射和写入 |
| `file_output_s` | 主结果JSON/MAT首次保存 |
| `total_runtime_s` | 从入口开始至主结果首次保存完成 |

为避免重复压缩大轨迹，MAT中的主`result`只保存一次，再追加小型顶层`timings`变量。JSON、函数返回的`result.timings`和MAT顶层`timings`包含已测主文件IO的计时；MAT嵌套`result.timings`是IO前值。追加计时变量、最终JSON写回及打印本身的少量开销不计入上述`total_runtime_s`，这一约定保存在`measurement_note`中。

```matlab
data = load(fullfile(project_root,'outputs','day6_sequential_trajectories.mat'));
disp(data.timings); % 读取MAT顶层精确计时，而非IO前的data.result.timings
```

## 6. 验证状态、交付文件与限制

新增`tests/test_reverse_motion.m`及`tests/test_reverse_search.m`，验证前进兼容、倒车加速/转向/制动、非负速率、非法换挡、0.5s驻留、驻留动态冲突、挡位查重、倒车罚项、同挡位终点连接及实际搜索。历史Day4/Day6测试使用显式前进fixture保留原断言，新倒车行为单独验证。

既有测试覆盖追尾、交叉、时间错开、等待、旋转矩形、扫掠、边界/时间越界、解析连接航向、资源块占用一致性和同时起步。最终真实两车验收还必须确认两条路径有效、末速度0、目标位姿达标、运动学有效，含停车驻留的搜索资源交集与独立Day5写入交集都为0。

本次MATLAB R2023a单元测试和回归实际结果为**212/212通过、0失败、0未完成**。除历史Day3/4/5/6与倒车测试外，覆盖稀疏查询与完整查询等价、Open顺序/搜索兼容、缓存失效及模型样本复用。四组真实两车Demo共12次正式重复也全部通过，两车同时从静止起步、末速度0、目标位姿达标，运动学与独立资源验收均有效。每次Demo具体到达时间、阶段计时、连续净距下界和误差见本次生成的JSON，不混用此前场景的133项报告或旧图片。

在独立验证工程中另一次完整`validate_day6_sequential(true)`实际通过并生成PNG/GIF，计时如下。它是一次带输出运行，不能替代上述重复基准中位数，F盘最终部署及目标目录复验由最终交付记录确认。

| 完整输出运行 | 实际值 |
| --- | ---: |
| 算法计算 | 43.107998s |
| 高车搜索 / 资源地图 / 低车搜索 | 0.195212 / 0.077002 / 42.385245s |
| 独立验收 | 0.378635s |
| PNG绘图 | 2.009427s |
| GIF动画生成 | 12.116372s |
| 主JSON/MAT文件输出 | 0.567666s |
| 总运行 | 57.819777s |
| 高车最终状态 | `[10,5,-pi,0,9.52150928327]` |
| 低车最终状态 | `[24,15,pi/2,0,17.79440449396]` |
| 采样最小车身净距 / 保守连续下界 | 1.015491 / 0.753303m |
| 最大运动学重放残差：x / 航向 / 速度 | 9.566e-15m / 1.776e-15rad / 4.885e-15m/s |
| 两种资源交集 / 两车实际换挡次数 | 均0块 / 均0次 |

高车末航向`-pi`与目标`pi`表示相同车身方向，是角度归一化，并非终点姿态改变。从本次分阶段计时也可看到GIF还有约12s输出开销；只关注算法效率时使用`save_outputs=false`，需要静态图且不需要动画时使用`day6.save_animation=false`。

`generate_reverse_validation_plot`在2.6m宽直通道中使用同一实际车型与搜索器，从`[15,1.3,0,0,0]`搜索到车身后方`[5,1.3,0,0,0]`。实际搜索验证通过：倒车距离10m，1次换挡，全程位姿不变的零速换挡等待0.5s，终点零速且保持目标车身航向。前进模式在200节点预算内以错误码5退出；这是有限预算内未成功，不作为数学无解证明。此特殊验证仅在内存副本配置中使用其明确的场景参数，与四组基准不同，不参与性能比例计算。

### 本次图片如何解读

`day6_search_trajectories.png`的蓝线是车辆2，橙线是车辆3，车身矩形和箭头表示实际航向。蓝车约9.52s到达并停车，橙车约17.79s到达并停车。两条线在二维空间接近或交叉，不代表同一时刻占用相同资源，最终两种独立资源验收都是0交集。

主图速度曲线约5–7s的橙色零速区间是低车为避让高车而产生的交通等待，不是换挡；本次两条主路径的`is_gear_change`均没有标记，两车保持前进挡。交通等待时长由搜索决定，不受0.5s换挡常数限定。只有`is_gear_change=true`对应的独立零速边才是换挡，图中的换挡标记应以真实metadata为准，不能把所有零速区间都叫作换挡。

`day6_reverse_corridor.png`独立证明特殊场景的倒车能力：车身一直朝右，实际后轴运动朝左；下方`gear*v`为负表示倒车，而`path.v`始终非负。黄色区间是搜索回溯得到的实际0.5s零速换挡等待，随后受限加速倒车，再按原模型减速至目标停车。主两车场景无需倒车，倒车验证图也没有替换主场景真实规划结果。

| 输出文件 | 内容 |
| --- | --- |
| `outputs/day6_performance_comparison.json` | 四组暖机、重复原始数据、成功率、计算中位数及失败原因 |
| `outputs/day6_sequential_report.json` | 当前两车状态、独立安全验收和完整阶段计时 |
| `outputs/day6_sequential_trajectories.mat` | 搜索路径/控制/节点、挡位事件、顶层计时 |
| `outputs/day6_search_trajectories.png` | 真实搜索轨迹、带方向速度与车身净距 |
| `outputs/day6_speed_time.png` | 带方向v-t曲线及换挡等待区间 |
| `outputs/day6_sequential_planning.gif` | 启用动画时输出的同步两车运动 |
| `outputs/day6_animation_preview.png` | RGB全色动画首帧预览 |
| `outputs/day6_reverse_corridor.png` | 狭窄通道真实倒车搜索、零速换挡与带方向速度 |
| `outputs/day6_validation.json` | 本次测试与真实Demo验收记录 |

修改范围为独立召集规划模块、配置中的五档转角/倒车/动画字段、对应测试和文档。当前车辆场景、原连接距离/参考速度/搜索上限/安全步长保留；原APA模型及Day3原运动学不覆盖。部署后的完整变更与源文件保护结果由本次最终交付清单记录。

本次源代码、配置、测试及文档变更清单如下；部署辅助脚本、只读基线和生成结果不属于算法源文件：

```text
config/planner_config.json                         修改
matlab/single_vehicle/check_trajectory_conflict.m 修改
matlab/single_vehicle/day6_motion_trajectory.m     修改
matlab/single_vehicle/day6_resource_blocks.m       修改
matlab/single_vehicle/day6_static_collision_check.m 修改
matlab/single_vehicle/day6_trajectory_states.m     修改
matlab/single_vehicle/plot_day6_trajectories.m      修改
matlab/single_vehicle/prepare_dynamic_context.m    修改
matlab/single_vehicle/prepare_static_context.m     新增
matlab/single_vehicle/run_day6_sequential_planning_demo.m 修改
matlab/single_vehicle/st_write_occupancy.m         修改
matlab/single_vehicle/summon_vehicle_dynamic_gear.m 新增
matlab/single_vehicle/summon_vhybrid_astar.m        修改
matlab/single_vehicle/vhybrid_closed_set.m          修改
matlab/single_vehicle/vhybrid_cost.m                修改
matlab/single_vehicle/vhybrid_curve_boundary_check.m 修改
matlab/single_vehicle/vhybrid_expand_node.m         修改
matlab/single_vehicle/vhybrid_goal_connection.m     修改
matlab/single_vehicle/vhybrid_heuristic.m           修改
matlab/single_vehicle/vhybrid_node.m                修改
matlab/single_vehicle/vhybrid_open_set.m            修改
matlab/single_vehicle/vhybrid_validate_trajectory.m 修改
tests/benchmark_day6_performance.m                 新增
tests/generate_reverse_validation_plot.m           新增
tests/test_day6_performance_equivalence.m           新增
tests/test_dynamic_collision.m                     修改
tests/test_motion_sample_cache.m                   新增
tests/test_resource_query_equivalence.m             新增
tests/test_reverse_motion.m                        新增
tests/test_reverse_search.m                        新增
tests/test_vhybrid_core.m                          修改
tests/validate_day6_sequential.m                   修改
docs/day6_performance_reverse.md                   新增
docs/day6_dynamic_collision.md                     修改旧场景/版本说明
```

当前限制：加权启发、离散控制和减速中间碰撞剪枝不保证搜索最优性或完备性；动态资源层及扫掠膨胀可能保守拒绝实际可错开的近距离动作；允许倒车可能增加节点数量和运行时间。搜索超时仅表示计算预算用尽。没有实现重新排序、联合优化、连续轨迹优化或整车控制器。

下一步建议：保留这次四组原始数据和当前用户场景，再增加狭窄通道、停车位退出及必须倒车的场景，分别检查成功率、挡位切换次数、净距与计算预算。后续优化仍需经过相同安全验收，不能仅比较图像是否更平滑。
