function path = summon_vhybrid_astar(start_state,goal_state,map,vehicle_config,planner_config,st_map,higher_priority_trajectories)
%SUMMON_VHYBRID_ASTAR 单车/顺序多车共用的五维 V-Hybrid A* 搜索器。
% 输入：起终点[x,y,theta,v,t]、静态地图、集中配置；可选时空地图及高优先级车辆。
% 输出：路径、搜索节点、运动学/动态验收和明确的失败原因。
% 状态按x/y/theta/v/t查重。控制由既有Day3模型积分；所有运动段在加入
% Open之前接受静态与动态碰撞验收。无动态参数时保持Day4单车接口行为。
if nargin < 6, st_map = []; end
if nargin < 7, higher_priority_trajectories = []; end
start_state = double(start_state(:).'); goal_state = double(goal_state(:).');
has_context = ~isempty(st_map) || ~isempty(higher_priority_trajectories);
if has_context, validate_st_occupancy_config(planner_config); end
path = emptyPath();
if numel(start_state) ~= 5 || numel(goal_state) ~= 5 || ...
        any(~isfinite([start_state,goal_state])) || start_state(4) < planner_config.dynamics.v_min_mps || ...
        start_state(4) > planner_config.dynamics.v_max_mps
    path.error_code = uint8(1);
    path.failure_diagnostics = failureDiagnostics(path,planner_config);
    return;
end
path.goal_pose = goal_state(1:3);
timer_id = tic;
stats = initStats();
[start_collision, ~] = day6_static_collision_check(start_state(1:3), map, vehicle_config);
[goal_collision, ~] = day6_static_collision_check(goal_state(1:3), map, vehicle_config);
if start_collision
    path.error_code = uint8(2);
    path.search_statistics = finishStats(stats, timer_id);
    path.failure_diagnostics = failureDiagnostics(path,planner_config);
    return;
end
if goal_collision
    path.error_code = uint8(3);
    path.search_statistics = finishStats(stats, timer_id);
    path.failure_diagnostics = failureDiagnostics(path,planner_config);
    return;
end

if has_context
    [blocked,report] = check_trajectory_conflict(start_state,higher_priority_trajectories, ...
        vehicle_config,planner_config,st_map);
    if blocked
        path.error_code = uint8(9); path.dynamic_validation = report;
        path.search_statistics = finishStats(stats,timer_id);
        path.failure_diagnostics = failureDiagnostics(path,planner_config);
        return;
    end
end
cfg = planner_config.vhybrid;
start_indices = discretizeState(start_state, map, cfg);
start_node = vhybrid_node(start_state, 0, 0, 0, 0, ...
    goalHeuristic(start_state, goal_state, cfg), start_indices, 1);
open_set = vhybrid_open_set();
closed_set = vhybrid_closed_set();
open_set.push(start_node);
nodes = repmat(vhybrid_node(), double(cfg.max_search_nodes), 1);
nodes(1) = start_node;
node_count = 1;
stats.open_peak = 1;

while ~open_set.is_empty()
    if toc(timer_id) > cfg.max_search_time_s
        path.error_code = uint8(6);
        path.search_statistics = finishStats(stats, timer_id);
        path.search_nodes = nodes(1:node_count);
        path.failure_diagnostics = failureDiagnostics(path,planner_config);
        return;
    end
    [current, valid] = open_set.pop_min();
    if ~valid
        break;
    end
    if closed_set.contains(current)
        stats.closed_pruned_nodes = stats.closed_pruned_nodes + 1;
        continue;
    end
    closed_set.add(current);
    stats.expanded_nodes = stats.expanded_nodes + 1;
    if has_context && ~isempty(st_map) && current.t >= st_map.t_max
        stats.horizon_pruned = stats.horizon_pruned+1;
        continue;
    end
    if isGoal(current, goal_state, cfg)
        path = backtrackPath(nodes, current.node_id, stats, timer_id, vehicle_config, planner_config);
        path.search_nodes = nodes(1:node_count);
        path.goal_pose = goal_state(1:3);
        path = validateFinalState(path, goal_state, cfg);
        path = validateKinematics(path, map, vehicle_config, planner_config);
        path = validateDynamic(path,vehicle_config,planner_config,st_map,higher_priority_trajectories);
        if path.valid, return; end
        continue;
    end

    if isfield(cfg, 'goal_connection_enabled') && cfg.goal_connection_enabled
        [connected, connection_nodes, connection_stats] = vhybrid_goal_connection( ...
            current, goal_state, map, vehicle_config, planner_config,st_map,higher_priority_trajectories);
        stats.goal_connection_candidates = stats.goal_connection_candidates + connection_stats.candidates;
        stats.goal_connection_collision_pruned = stats.goal_connection_collision_pruned + connection_stats.collision_pruned;
        stats.goal_connection_geometric_candidates = stats.goal_connection_geometric_candidates + ...
            connection_stats.geometric_candidates;
        stats.goal_connection_boundary_pruned = stats.goal_connection_boundary_pruned + ...
            connection_stats.boundary_geometry_pruned;
        stats = mergeDynamicStats(stats,connection_stats,planner_config);
        if connected && node_count + numel(connection_nodes) <= cfg.max_search_nodes
            previous_id = current.node_id;
            for connection_id = 1:numel(connection_nodes)
                node_count = node_count + 1;
                connection_nodes(connection_id).node_id = node_count;
                connection_nodes(connection_id).parent_id = previous_id;
                nodes(node_count) = connection_nodes(connection_id);
                previous_id = node_count;
            end
            path = backtrackPath(nodes, previous_id, stats, timer_id, vehicle_config, planner_config);
            path.search_nodes = nodes(1:node_count);
            path.goal_pose = goal_state(1:3);
            path = validateFinalState(path, goal_state, cfg);
            path = validateKinematics(path, map, vehicle_config, planner_config);
            path = validateDynamic(path,vehicle_config,planner_config,st_map,higher_priority_trajectories);
            if path.valid, return; end
            continue;
        end
    end

    [children, expand_stats] = vhybrid_expand_node(current, map, ...
        vehicle_config, planner_config, goal_state,st_map,higher_priority_trajectories);
    stats.sampled_controls = stats.sampled_controls + expand_stats.sampled;
    stats.generated_nodes = stats.generated_nodes + expand_stats.valid;
    stats.collision_pruned_nodes = stats.collision_pruned_nodes + expand_stats.collision_pruned;
    stats.constraint_pruned_nodes = stats.constraint_pruned_nodes + expand_stats.constraint_pruned;
    stats = mergeDynamicStats(stats,expand_stats,planner_config);
    for k = 1:numel(children)
        if node_count >= cfg.max_search_nodes
            path.error_code = uint8(5);
            path.search_statistics = finishStats(stats, timer_id);
            path.search_nodes = nodes(1:node_count);
            path.failure_diagnostics = failureDiagnostics(path,planner_config);
            return;
        end
        child = children(k);
        if closed_set.contains(child)
            stats.closed_pruned_nodes = stats.closed_pruned_nodes + 1;
            continue;
        end
        child.node_id = node_count + 1;
        if open_set.push_or_update(child)
            node_count = node_count + 1;
            nodes(node_count) = child;
        end
        stats.open_peak = max(stats.open_peak, open_set.count());
    end
end

path.error_code = uint8(4);
if stats.horizon_pruned > 0, path.error_code = uint8(11); end
path.search_statistics = finishStats(stats, timer_id);
path.search_nodes = nodes(1:node_count);
path.failure_diagnostics = failureDiagnostics(path,planner_config);
end

function path = backtrackPath(nodes, goal_id, stats, timer_id, vehicle, planner)
%BACKTRACKPATH 回溯控制序列，再用运动学细分节点；禁止 xy/theta 插值。
if goal_id <= 0 || goal_id > numel(nodes)
    path = emptyPath();
    path.valid = false;
    path.error_code = uint8(4);
    path.search_statistics = finishStats(stats, timer_id);
    path.failure_diagnostics = failureDiagnostics(path,planner);
    return;
end
indices = zeros(numel(nodes),1);
count = 0;
id = goal_id;
while id > 0
    count = count + 1;
    indices(count) = id;
    id = nodes(id).parent_id;
end
indices = indices(count:-1:1);
path = emptyPath();
first = nodes(indices(1));
states = [first.x,first.y,first.theta,first.v,first.t];
controls = [0,0,0]; % 每行 [加速度, 前轮转角, 从上一个点到本点的弧长]
dyn = planner.dynamics; cfg = planner.vhybrid;
for k = 2:numel(indices)
    parent = nodes(indices(k-1)); child = nodes(indices(k));
    start = [parent.x,parent.y,parent.theta,parent.v,parent.t];
    duration = child.t-parent.t;
    n = max([1,ceil(duration/cfg.path_sample_step_s), ...
        ceil(abs(child.travelled_distance)/cfg.collision_check_step_m)]);
    for j = 1:n
        dt = duration*j/n;
        [state,~,~,valid] = summon_vehicle_dynamic(start,child.steering_angle, ...
            child.acceleration,dt,vehicle.dimensions_m.wheelbase, ...
            dyn.v_max_mps,dyn.v_min_mps,dyn.a_min_mps2,dyn.a_max_mps2, ...
            min(vehicle.steering.maximum_steer_rad,dyn.delta_max_rad));
        if ~valid
            path.error_code = uint8(8); return;
        end
        previous = states(end,:);
        ds = 0.5*(previous(4)+state(4))*(state(5)-previous(5));
        states(end+1,:) = state; %#ok<AGROW>
        controls(end+1,:) = [child.acceleration,child.steering_angle,ds]; %#ok<AGROW>
    end
end
path.x = states(:,1); path.y = states(:,2); path.theta = states(:,3);
path.v = states(:,4); path.t = states(:,5);
path.acceleration = controls(:,1); path.steering_angle = controls(:,2);
path.travelled_distance = controls(:,3); path.direction = sign(controls(:,3));
path.valid = true;
path.error_code = uint8(0);
stats.path_nodes = size(states,1);
path.search_statistics = finishStats(stats, timer_id);
end

function flag = isGoal(node, goal_state, cfg)
%ISGOAL 位置、航向和停车速度必须同时满足容差。
node = node(1);
if isempty(node.node_id) || node.node_id(1) <= 0
    flag = false;
    return;
end
position_ok = logical(hypot(node.x-goal_state(1), node.y-goal_state(2)) ...
    <= cfg.goal_position_tolerance_m);
heading_ok = logical(abs(wrapToPiLocal(node.theta-goal_state(3))) ...
    <= cfg.goal_heading_tolerance_rad);
speed_ok = logical(abs(node.v - cfg.terminal_speed_mps) <= cfg.goal_speed_tolerance_mps);
flag = all([position_ok(:); heading_ok(:); speed_ok(:)]);
end

function h = goalHeuristic(state, goal_state, cfg)
%GOALHEURISTIC 计算起点节点的目标距离启发值。
h = vhybrid_heuristic(state,goal_state,cfg);
end

function indices = discretizeState(state, map, cfg)
%DISCRETIZESTATE 生成起点使用的五维索引。
indices = [floor((state(1)-map.bounds(1))/cfg.xy_grid_resolution_m), ...
    floor((state(2)-map.bounds(2))/cfg.xy_grid_resolution_m), ...
    round((wrapToPiLocal(state(3))+pi)/cfg.yaw_grid_resolution_rad), ...
    round(state(4)/cfg.velocity_grid_resolution_mps), round(state(5)/cfg.time_grid_resolution_s)];
end

function path = emptyPath()
%EMPTYPATH 初始化 V-Hybrid A* 统一路径输出。
path = struct('x',zeros(0,1),'y',zeros(0,1),'theta',zeros(0,1), ...
    'v',zeros(0,1),'t',zeros(0,1),'direction',zeros(0,1), ...
    'valid',false,'error_code',uint8(0),'search_nodes',repmat(vhybrid_node(),0,1), ...
    'search_statistics',initStats(),'goal_pose',zeros(1,3), ...
    'final_position_error_m',NaN,'final_heading_error_rad',NaN, ...
    'final_speed_error_mps',NaN,'acceleration',zeros(0,1), ...
    'steering_angle',zeros(0,1),'travelled_distance',zeros(0,1), ...
    'kinematic_validation',struct(),'dynamic_validation',struct(),'failure_diagnostics',struct());
end

function path = validateFinalState(path, goal_state, cfg)
%VALIDATEFINALSTATE 对最终路径点执行位置、航向和速度硬性验收。
if isempty(path.x)
    path.valid = false;
    path.error_code = uint8(4);
    return;
end
path.final_position_error_m = hypot(path.x(end)-goal_state(1), ...
    path.y(end)-goal_state(2));
path.final_heading_error_rad = abs(wrapToPiLocal(path.theta(end)-goal_state(3)));
path.final_speed_error_mps = abs(path.v(end)-cfg.terminal_speed_mps);
path.valid = path.final_position_error_m <= cfg.goal_position_tolerance_m && ...
    path.final_heading_error_rad <= cfg.goal_heading_tolerance_rad && ...
    path.final_speed_error_mps <= cfg.goal_speed_tolerance_mps;
if ~path.valid
    path.error_code = uint8(7);
end
end

function path = validateKinematics(path, map, vehicle, planner)
%VALIDATEKINEMATICS 对成功路径逐点重放，航向误差小不代表运动学可行。
path.kinematic_validation = vhybrid_validate_trajectory(path,map,vehicle,planner);
if ~path.kinematic_validation.valid
    path.valid = false; path.error_code = uint8(8);
end
end

function stats = initStats()
%INITSTATS 初始化 V-Hybrid A* 搜索统计。
stats = struct('expanded_nodes',0,'generated_nodes',0,'sampled_controls',0, ...
    'collision_pruned_nodes',0,'constraint_pruned_nodes',0,'closed_pruned_nodes',0, ...
    'goal_connection_candidates',0,'goal_connection_collision_pruned',0, ...
    'goal_connection_geometric_candidates',0,'goal_connection_boundary_pruned',0, ...
    'open_peak',0,'path_nodes',0,'elapsed_sec',0,'dynamic_pruned',0, ...
    'resource_pruned',0,'braking_midpoint_pruned',0,'faster_controls_skipped',0, ...
    'waiting_nodes',0,'horizon_pruned',0,'reason_counts',struct(),'conflict_examples',{{}});
end

function stats = finishStats(stats, timer_id)
%FINISHSTATS 写入搜索耗时。
stats.elapsed_sec = toc(timer_id);
end

function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将角度归一化到 [-pi,pi)。
angle = mod(angle+pi,2*pi)-pi;
end


function stats = mergeDynamicStats(stats,part,planner)
%MERGEDYNAMICSTATS 汇总扩展段和终点连接的分类裁剪、冲突实例与等待节点。
names = {'dynamic_pruned','resource_pruned','braking_midpoint_pruned', ...
    'faster_controls_skipped','waiting_nodes','horizon_pruned'};
for k = 1:numel(names)
    if isfield(part,names{k}), stats.(names{k}) = stats.(names{k})+part.(names{k}); end
end
if isfield(part,'reason_counts')
    reasons = fieldnames(part.reason_counts);
    for k = 1:numel(reasons)
        key = reasons{k};
        if ~isfield(stats.reason_counts,key), stats.reason_counts.(key) = 0; end
        stats.reason_counts.(key) = stats.reason_counts.(key)+part.reason_counts.(key);
    end
end
if isfield(part,'conflict_examples') && isfield(planner,'day6')
    count = max(0,planner.day6.max_conflict_examples-numel(stats.conflict_examples));
    stats.conflict_examples = [stats.conflict_examples,part.conflict_examples(1:min(count,numel(part.conflict_examples)))];
end
end

function path = validateDynamic(path,vehicle,planner,st_map,higher)
%VALIDATEDYNAMIC 对完整规划结果及终点停车驻留执行独立的动态安全验收。
if isempty(st_map) && isempty(higher), return; end
[collision,report] = check_trajectory_conflict(path,higher,vehicle,planner,st_map);
if ~collision && ~isempty(higher) && planner.day6.goal_hold_enabled
    if isempty(st_map), horizon = planner.st_occupancy.t_max;
    else, horizon = st_map.t_max; end
    hold = struct('x',[path.x(end);path.x(end)],'y',[path.y(end);path.y(end)], ...
        'theta',[path.theta(end);path.theta(end)],'v',[0;0], ...
        't',[path.t(end);horizon-max(1e-9,16*eps(horizon))], ...
        'acceleration',[0;0],'steering_angle',[0;0]);
    if hold.t(2)>hold.t(1)
        [held_collision,held_report] = check_trajectory_conflict(hold,higher,vehicle,planner,st_map);
        if held_collision, collision = true; report = held_report;
        else
            report.minimum_distance_m = min(report.minimum_distance_m,held_report.minimum_distance_m);
            report.minimum_distance_lower_bound_m = min(report.minimum_distance_lower_bound_m,held_report.minimum_distance_lower_bound_m);
            report.checked_samples = report.checked_samples+held_report.checked_samples;
        end
    end
end
path.dynamic_validation = report;
if collision, path.valid = false; path.error_code = uint8(10); end
end

function diagnostics = failureDiagnostics(path,planner)
%FAILUREDIAGNOSTICS 记录失败及可调整速度/进入时间，不自动改变优先级或起步时刻。
labels = {'success','invalid_state','static_start_collision','static_goal_collision', ...
    'open_exhausted','node_limit','search_timeout','terminal_state_invalid', ...
    'kinematic_invalid','dynamic_start_collision','dynamic_final_collision','time_horizon_exhausted'};
index = double(path.error_code)+1;
diagnostics = struct('reason',labels{min(index,numel(labels))}, ...
    'error_code',path.error_code,'suggested_speed_range_mps', ...
    [planner.dynamics.v_min_mps,planner.dynamics.v_max_mps], ...
    'adjustable_entry_times_s',[],'automatic_entry_time_shift_applied',false, ...
    'note','可由调用方调整控制速度采样或进入冲突区时间；本次搜索不改变t=0起步、不重新排序。');
if isfield(planner,'st_occupancy')
    diagnostics.adjustable_entry_times_s = planner.st_occupancy.t_min:planner.st_occupancy.dt:planner.st_occupancy.t_max;
end
diagnostics.rejection_counts = path.search_statistics.reason_counts;
end
