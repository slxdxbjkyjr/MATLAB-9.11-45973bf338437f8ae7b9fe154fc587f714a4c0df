function result = run_day6_sequential_planning_demo(save_outputs,options)
%RUN_DAY6_SEQUENTIAL_PLANNING_DEMO Day1车辆2/3从t=0静止同步运动的顺序规划。
% 输入：save_outputs是否保存结果，默认true；options.planner_config可传完整配置副本。
% 输出：result含两车真实搜索路径、资源块统计、运动学/动态验收和失败诊断。
% result.timings分开记录算法计算、PNG绘图、GIF动画及文件输出，单位均为秒。
% 先计算高优先级路径，再预留其完整车身扫掠及终点停车资源，最后计算低优先级
% 路径。计算顺序不等于车辆出发时间；两车输入均为v=0、t=0，不能手工延迟。
total_timer = tic;
if nargin < 1, save_outputs = true; end
if nargin < 2, options = struct(); end
if ~islogical(save_outputs) || ~isscalar(save_outputs)
    error('Day6:InvalidArgument','save_outputs必须是逻辑标量。');
end
if ~isstruct(options) || ~isscalar(options)
    error('Day6:InvalidArgument','options必须是标量结构体。');
end
this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(fileparts(this_dir));
config_dir = fullfile(project_dir,'config');
vehicle = jsondecode(fileread(fullfile(config_dir,'vehicle_config.json')));
map_config = jsondecode(fileread(fullfile(config_dir,'map_config.json')));
planner = jsondecode(fileread(fullfile(config_dir,'planner_config.json')));
% 对比实验只覆盖内存中的规划配置，Day1场景和用户JSON始终保持原样。
if isfield(options,'planner_config'), planner = options.planner_config; end
% Day6仅在内存配置副本中加权启发式以减少无效扩展；不改Day4配置/场景。
% 两车使用相同权重和搜索器；权重大于1时不保证全局最优路径。
planner.vhybrid.heuristic_weight = planner.day6.search_heuristic_weight;
planner.vhybrid.heuristic_method = planner.day6.heuristic_method;
planner.vhybrid.minimum_turning_radius_m = vehicle.dimensions_m.wheelbase / ...
    tan(min(vehicle.steering.maximum_steer_rad,planner.dynamics.delta_max_rad));
scenario = jsondecode(fileread(fullfile(config_dir,'scenario_001.json')));
geometry = summon_map(map_config);
validate_st_occupancy_config(planner);
high_config = scenarioVehicle(scenario,'vehicle_002');
low_config = scenarioVehicle(scenario,'vehicle_003');
[high_start,high_goal] = scenarioStates(high_config);
[low_start,low_goal] = scenarioStates(low_config);
result = struct('valid',false,'stage','day6_sequential_planning', ...
    'high_priority_id',high_config.id,'low_priority_id',low_config.id, ...
    'high_priority_path',struct(),'low_priority_path',struct(), ...
    'validation',struct(),'resource_statistics',struct(),'output_files',struct());
result.timings = struct('configuration_s',toc(total_timer),'high_search_s',0, ...
    'resource_map_s',0,'low_search_s',0,'independent_validation_s',0, ...
    'low_resource_statistics_s',0,'compute_s',0,'plot_s',0,'animation_s',0, ...
    'file_output_s',0,'total_runtime_s',0, ...
    'measurement_note',['总耗时统计至主结果首次保存完成；随后小计时变量追加及JSON写回不计入。' ...
    'MAT内result.timings是IO前值，准确计时取MAT顶层timings、JSON或函数返回result.timings。']);
validation = struct('valid',false,'reason','not_completed','collision',false, ...
    'conflict_reason','not_checked','minimum_distance_m',NaN, ...
    'minimum_distance_lower_bound_m',NaN,'conflict_time_s',NaN, ...
    'conflict_position_xy',[NaN,NaN],'resource_overlap_blocks',NaN, ...
    'high_starts_at_rest_t0',false,'low_starts_at_rest_t0',false, ...
    'both_move_immediately',false,'both_stop_at_goal',false, ...
    'both_kinematics_valid',false,'dynamic_pruned_nodes',0, ...
    'resource_pruned_nodes',0,'rejection_counts',struct(), ...
    'adjustable_speed_range_mps',[planner.dynamics.v_min_mps,planner.dynamics.v_max_mps], ...
    'adjustable_enter_time_range_s',[planner.st_occupancy.t_min,planner.st_occupancy.t_max], ...
    'adjustments_applied',false,'failure_diagnostics',struct());

% 两次都使用同一通用搜索器。高优先级只受静态地图和运动学约束。
section_timer = tic;
high = summon_vhybrid_astar(high_start,high_goal,geometry,vehicle,planner,[],[]);
result.timings.high_search_s = toc(section_timer);
result.high_priority_path = high;
if ~high.valid
    validation.reason = 'high_priority_search_failed';
    validation.failure_diagnostics = high.failure_diagnostics;
    result.validation = validation;
    result.timings.compute_s = toc(total_timer);
    result = finishResult(result,save_outputs,project_dir,total_timer);
    return;
end

% 终点停车一直预留至半开时间范围上界；不允许到达后凭空消失。
section_timer = tic;
resource_planner = planner;
% 预留与最终矩形判据使用同一额外车间裕度，避免只修改Day6裕度后资源图偏小。
resource_planner.st_occupancy.safety_distance_m = max( ...
    resource_planner.st_occupancy.safety_distance_m,planner.day6.minimum_safety_distance_m/2);
if planner.day6.goal_hold_enabled
    resource_planner.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
end
st_map = st_occupancy_map(resource_planner,geometry);
[st_map,static_statistics] = mark_static_obstacles(st_map,geometry,vehicle);
obstacles = dynamic_vehicle_obstacle(high,vehicle,high_config.id,planner);
[st_map,high_resources] = mark_trajectory_occupancy(st_map,high,vehicle,high_config.id);
st_map = prepare_dynamic_context(st_map,obstacles);
result.resource_statistics = struct('static',static_statistics,'high_priority',high_resources);
result.timings.resource_map_s = toc(section_timer);
section_timer = tic;
low = summon_vhybrid_astar(low_start,low_goal,geometry,vehicle,planner,st_map,obstacles);
result.timings.low_search_s = toc(section_timer);
result.low_priority_path = low;
validation.dynamic_pruned_nodes = low.search_statistics.dynamic_pruned;
validation.resource_pruned_nodes = low.search_statistics.resource_pruned;
validation.rejection_counts = low.search_statistics.reason_counts;
if ~low.valid
    validation.reason = 'low_priority_search_failed';
    validation.failure_diagnostics = low.failure_diagnostics;
    result.validation = validation;
    result.timings.compute_s = toc(total_timer);
    result = finishResult(result,save_outputs,project_dir,total_timer);
    return;
end

% 验收包含低优先级停车驻留；原搜索路径仍保留原控制和真正到达时间。
section_timer = tic;
evaluation_states = day6_trajectory_states(low);
if planner.day6.goal_hold_enabled
    horizon_end = st_map.t_max-max(1e-9,eps(st_map.t_max));
    if evaluation_states(end,5) < horizon_end
        stop = evaluation_states(end,:);
        stop(4) = 0; stop(5) = horizon_end;
        evaluation_states(end+1,:) = stop;
    end
end
[collision,conflict] = check_trajectory_conflict(evaluation_states,obstacles,vehicle,planner,st_map);
validation.collision = collision;
validation.conflict_reason = conflict.reason;
validation.minimum_distance_m = conflict.minimum_distance_m;
validation.minimum_distance_lower_bound_m = conflict.minimum_distance_lower_bound_m;
validation.conflict_time_s = conflict.conflict_time_s;
validation.conflict_position_xy = conflict.conflict_position_xy;
validation.resource_overlap_blocks = numel(conflict.resource_indices);
% 用Day5写入函数独立复核两车资源交集；不借助搜索中的资源查询实现。
dynamic_only_map = st_occupancy_map(resource_planner,geometry);
dynamic_only_map = mark_trajectory_occupancy(dynamic_only_map,high,vehicle,high_config.id);
[~,independent_resources] = mark_trajectory_occupancy(dynamic_only_map,low,vehicle,low_config.id);
validation.independent_resource_overlap_blocks = independent_resources.conflict_blocks;
validation.high_starts_at_rest_t0 = startsAtRest(high);
validation.low_starts_at_rest_t0 = startsAtRest(low);
validation.both_move_immediately = movesImmediately(high) && movesImmediately(low);
validation.both_stop_at_goal = abs(high.v(end)) <= planner.vhybrid.goal_speed_tolerance_mps && ...
    abs(low.v(end)) <= planner.vhybrid.goal_speed_tolerance_mps;
validation.both_kinematics_valid = high.kinematic_validation.valid && low.kinematic_validation.valid;
validation.valid = high.valid && low.valid && ~collision && ...
    validation.resource_overlap_blocks == 0 && validation.independent_resource_overlap_blocks == 0 && ...
    validation.high_starts_at_rest_t0 && ...
    validation.low_starts_at_rest_t0 && validation.both_move_immediately && ...
    validation.both_stop_at_goal && validation.both_kinematics_valid;
if validation.valid
    validation.reason = 'passed';
elseif collision
    validation.reason = 'final_dynamic_validation_failed';
elseif ~validation.both_move_immediately
    validation.reason = 'initial_wait_violates_simultaneous_motion';
else
    validation.reason = 'initial_terminal_or_kinematic_validation_failed';
end
result.validation = validation;
result.valid = validation.valid;
result.final_conflict_report = conflict;
result.timings.independent_validation_s = toc(section_timer);

% 低优先级资源仅用于输出统计，不参与本日的重新排序或联合优化。
if result.valid
    section_timer = tic;
    [~,low_resources] = mark_trajectory_occupancy(st_map,low,vehicle,low_config.id);
    result.resource_statistics.low_priority = low_resources;
    result.timings.low_resource_statistics_s = toc(section_timer);
    result.timings.compute_s = toc(total_timer);
    if save_outputs
        output_dir = fullfile(project_dir,'outputs');
        if ~exist(output_dir,'dir'), mkdir(output_dir); end
        [result.output_files,graphics_timings] = ...
            plot_day6_trajectories(high,low,geometry,vehicle,planner,output_dir,conflict.distance_samples);
        result.timings.plot_s = graphics_timings.plot_s;
        result.timings.animation_s = graphics_timings.animation_s;
    end
else
    result.timings.compute_s = toc(total_timer);
end
result = finishResult(result,save_outputs,project_dir,total_timer);
end

function result = finishResult(result,enabled,project_dir,total_timer)
%FINISHRESULT 成功或明确规划失败均输出计时；计算时间不包含PNG/GIF与文件写入。
output_timer = tic;
result = saveResult(result,enabled,project_dir);
result.timings.file_output_s = toc(output_timer);
result.timings.total_runtime_s = toc(total_timer);
% 大轨迹result只保存一次；只追加小计时变量及更新JSON，避免重复压缩巨大MAT。
if enabled
    timings = result.timings; 
    save(result.output_files.trajectory_mat,'timings','-append');
    result = saveResult(result,true,project_dir,false);
end
t = result.timings;
fprintf(['Day6计算时间 %.3f s（高车搜索 %.3f，资源地图 %.3f，低车搜索 %.3f，' ...
    '独立验收 %.3f）\nPNG绘图 %.3f s，GIF动画 %.3f s，文件输出 %.3f s，' ...
    '总运行 %.3f s；结果：%s。\n'],t.compute_s,t.high_search_s,t.resource_map_s, ...
    t.low_search_s,t.independent_validation_s,t.plot_s,t.animation_s, ...
    t.file_output_s,t.total_runtime_s,result.validation.reason);
end

function vehicle = scenarioVehicle(scenario,id)
%SCENARIOVEHICLE 通过Day1唯一ID取车，不依赖数组顺序或改写场景。
index = find(strcmp({scenario.vehicles.id},id));
if numel(index) ~= 1
    error('Day6:InvalidScenario','场景必须恰好含一辆%s。',id);
end
vehicle = scenario.vehicles(index);
end

function [start,goal] = scenarioStates(vehicle)
%SCENARIOSTATES 位置和航向完全使用Day1配置，出发速度/时间固定为零。
start = [vehicle.start_pose.x_m,vehicle.start_pose.y_m,vehicle.start_pose.yaw_rad,0,0];
goal = [vehicle.summon_goal.x_m,vehicle.summon_goal.y_m,vehicle.summon_goal.yaw_rad,0,0];
end

function flag = startsAtRest(path)
%STARTSATREST 出发绝对时间和速度必须同时为零。
flag = ~isempty(path.t) && abs(path.t(1)) < 1e-9 && abs(path.v(1)) < 1e-9;
end

function flag = movesImmediately(path)
%MOVESIMMEDIATELY 第一条回溯运动边即加速，不接受起步前插入等待边。
flag = numel(path.v) >= 2 && path.v(2) > 1e-9 && path.acceleration(2) > 0 && ...
    hypot(path.x(2)-path.x(1),path.y(2)-path.y(1)) > 1e-10;
end

function result = saveResult(result,enabled,project_dir,write_mat)
%SAVERESULT 成功/失败均保存独立Day6数据；失败时不生成成功轨迹图或动画。
if nargin < 4, write_mat = true; end
if ~enabled, return; end
output_dir = fullfile(project_dir,'outputs');
if ~exist(output_dir,'dir'), mkdir(output_dir); end
result.output_files.report_json = fullfile(output_dir,'day6_sequential_report.json');
result.output_files.trajectory_mat = fullfile(output_dir,'day6_sequential_trajectories.mat');
report = result;
report.high_priority_path = compactPath(result.high_priority_path);
report.low_priority_path = compactPath(result.low_priority_path);
if isfield(report,'final_conflict_report')
    report.final_conflict_report = rmfield(report.final_conflict_report,'distance_samples');
end
text = jsonencode(report,'PrettyPrint',true);
fid = fopen(result.output_files.report_json,'w','n','UTF-8');
if fid < 0, error('Day6:OutputFailure','无法写入Day6 JSON报告。'); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',text);
clear cleanup;
if write_mat, save(result.output_files.trajectory_mat,'result','-v7'); end
end

function summary = compactPath(path)
%COMPACTPATH JSON保留结果与统计，完整轨迹和搜索节点保存在MAT中。
if isempty(fieldnames(path)), summary = struct('valid',false,'reason','not_planned'); return; end
summary = struct('valid',path.valid,'error_code',path.error_code, ...
    'search_statistics',path.search_statistics,'failure_diagnostics',path.failure_diagnostics);
if ~isempty(path.t)
    summary.start_state = [path.x(1),path.y(1),path.theta(1),path.v(1),path.t(1)];
    summary.final_state = [path.x(end),path.y(end),path.theta(end),path.v(end),path.t(end)];
    summary.final_position_error_m = path.final_position_error_m;
    summary.final_heading_error_rad = path.final_heading_error_rad;
    summary.kinematic_validation = path.kinematic_validation;
    if isfield(path,'gear')
        summary.reverse_samples = nnz(path.gear < 0 & path.v > 1e-9);
        summary.gear_switches = nnz(diff(path.gear) ~= 0);
    end
end
end
