function path = run_vhybrid_astar_demo(vehicle_index, save_outputs)
%RUN_VHYBRID_ASTAR_DEMO 运行 Day 4 单车 V-Hybrid A* 原型。
%
% 输入：vehicle_index 为场景车辆序号（默认 2）；save_outputs 是否保存图（默认 true）。
% 输出：path.x/y/theta/v/t/direction、valid、error_code 和搜索统计。
%
% 算法逻辑：
%   选取 Day 1 场景指定车辆；Open 使用最小 f_cost 节点选择，Closed
%   使用 x/y/yaw/v/time 五维索引查重。搜索仅考虑静态道路边界，不使用多车动态障碍物。
%   完成后输出搜索节点、最终路径以及速度-时间曲线图。

if nargin < 1, vehicle_index = 2; end
if nargin < 2, save_outputs = true; end
this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(fileparts(this_dir));
config_dir = fullfile(project_dir, 'config');
vehicle_config = jsondecode(fileread(fullfile(config_dir, 'vehicle_config.json')));
map_config = jsondecode(fileread(fullfile(config_dir, 'map_config.json')));
planner_config = jsondecode(fileread(fullfile(config_dir, 'planner_config.json')));
scenario = jsondecode(fileread(fullfile(config_dir, 'scenario_001.json')));
map = summon_map(map_config);
vehicle = scenario.vehicles(vehicle_index);
start_state = [vehicle.start_pose.x_m, vehicle.start_pose.y_m, ...
    vehicle.start_pose.yaw_rad, 0.5, 0];
goal_state = [vehicle.summon_goal.x_m, vehicle.summon_goal.y_m, ...
    vehicle.summon_goal.yaw_rad, 0, 0];

path = emptyPath();
path.goal_pose = goal_state(1:3);
timer_id = tic;
stats = initStats();
[start_collision, ~] = summon_collision_check(start_state(1:3), map, vehicle_config);
[goal_collision, ~] = summon_collision_check(goal_state(1:3), map, vehicle_config);
if start_collision
    path.error_code = uint8(2);
    path.search_statistics = finishStats(stats, timer_id);
    return;
end
if goal_collision
    path.error_code = uint8(3);
    path.search_statistics = finishStats(stats, timer_id);
    return;
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
    if isGoal(current, goal_state, cfg)
        path = backtrackPath(nodes, current.node_id, stats, timer_id, vehicle_config, planner_config);
        path.search_nodes = nodes(1:node_count);
        path.goal_pose = goal_state(1:3);
        path = validateFinalState(path, goal_state, cfg);
        path = validateKinematics(path, map, vehicle_config, planner_config);
        if save_outputs, saveDemoPlots(path, map, vehicle_config, project_dir); end
        return;
    end

    if isfield(cfg, 'goal_connection_enabled') && cfg.goal_connection_enabled
        [connected, connection_nodes, connection_stats] = vhybrid_goal_connection( ...
            current, goal_state, map, vehicle_config, planner_config);
        stats.goal_connection_candidates = stats.goal_connection_candidates + connection_stats.candidates;
        stats.goal_connection_collision_pruned = stats.goal_connection_collision_pruned + connection_stats.collision_pruned;
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
            if save_outputs, saveDemoPlots(path, map, vehicle_config, project_dir); end
            return;
        end
    end

    [children, expand_stats] = vhybrid_expand_node(current, map, ...
        vehicle_config, planner_config, goal_state);
    stats.sampled_controls = stats.sampled_controls + expand_stats.sampled;
    stats.generated_nodes = stats.generated_nodes + expand_stats.valid;
    stats.collision_pruned_nodes = stats.collision_pruned_nodes + expand_stats.collision_pruned;
    stats.constraint_pruned_nodes = stats.constraint_pruned_nodes + expand_stats.constraint_pruned;
    for k = 1:numel(children)
        if node_count >= cfg.max_search_nodes
            path.error_code = uint8(5);
            path.search_statistics = finishStats(stats, timer_id);
            path.search_nodes = nodes(1:node_count);
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
path.search_statistics = finishStats(stats, timer_id);
path.search_nodes = nodes(1:node_count);
end

function path = backtrackPath(nodes, goal_id, stats, timer_id, vehicle, planner)
%BACKTRACKPATH 回溯控制序列，再用运动学细分节点；禁止 xy/theta 插值。
if goal_id <= 0 || goal_id > numel(nodes)
    path = emptyPath();
    path.valid = false;
    path.error_code = uint8(4);
    path.search_statistics = finishStats(stats, timer_id);
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
h = cfg.weight_goal_distance * (hypot(state(1)-goal_state(1), state(2)-goal_state(2)) ...
    + 0.5*abs(wrapToPiLocal(state(3)-goal_state(3))));
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
    'kinematic_validation',struct());
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
    'open_peak',0,'path_nodes',0,'elapsed_sec',0);
end

function stats = finishStats(stats, timer_id)
%FINISHSTATS 写入搜索耗时。
stats.elapsed_sec = toc(timer_id);
end

function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将角度归一化到 [-pi,pi)。
angle = mod(angle+pi,2*pi)-pi;
end

function saveDemoPlots(path, map, vehicle_config, project_dir)
%SAVEDEMOPLOTS 保存搜索节点/轨迹图和速度-时间图。
if isempty(path.x)
    return;
end
output_dir = fullfile(project_dir, 'outputs');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
summon_plot_path(path, map, vehicle_config, ...
    fullfile(output_dir, 'day4_vhybrid_astar.png'));
fig = figure('Visible', 'off', 'Color', 'w', 'Name', 'V-Hybrid speed-time');
plot(path.t, path.v, 'r-', 'LineWidth', 1.6);
grid on; xlabel('t / s'); ylabel('v / m/s');
title(sprintf('V-Hybrid A* speed profile, terminal v = %.3f m/s', path.v(end)));
exportgraphics(fig, fullfile(output_dir, 'day4_vhybrid_speed_time.png'));
close(fig);
end
