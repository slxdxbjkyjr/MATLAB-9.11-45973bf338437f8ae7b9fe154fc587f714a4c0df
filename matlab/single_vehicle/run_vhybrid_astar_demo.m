function path = run_vhybrid_astar_demo(vehicle_id)
%RUN_VHYBRID_ASTAR_DEMO 运行 Day 4 单车 V-Hybrid A* 原型。
%
% 输入：无。函数从当前工程 config 目录读取所有配置。
% 输出：path.x/y/theta/v/t/direction、valid、error_code 和搜索统计。
%
% 算法逻辑：
%   选取 Day 1 场景的 vehicle_001；Open 使用最小 f_cost 节点选择，Closed
%   使用 x/y/yaw/v/time 五维索引查重。搜索仅考虑静态道路边界，不使用多车动态障碍物。
%   完成后输出搜索节点、最终路径以及速度-时间曲线图。

this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(fileparts(this_dir));
config_dir = fullfile(project_dir, 'config');
vehicle_config = jsondecode(fileread(fullfile(config_dir, 'vehicle_config.json')));
map_config = jsondecode(fileread(fullfile(config_dir, 'map_config.json')));
planner_config = jsondecode(fileread(fullfile(config_dir, 'planner_config.json')));
scenario = jsondecode(fileread(fullfile(config_dir, 'scenario_001.json')));
map = summon_map(map_config);
if nargin < 1, vehicle_id = 'vehicle_002'; end
vehicle = scenario.vehicles(strcmp({scenario.vehicles.id}, vehicle_id));
if isempty(vehicle), error('Summon:Demo:VehicleNotFound', '未找到车辆 %s。', vehicle_id); end
start_state = [vehicle.start_pose.x_m, vehicle.start_pose.y_m, ...
    vehicle.start_pose.yaw_rad, 0.5, 0];
goal_state = [vehicle.summon_goal.x_m, vehicle.summon_goal.y_m, ...
    vehicle.summon_goal.yaw_rad, 0, 0];

path = emptyPath();
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
        if isempty(current) || ~isscalar(current.x)
            continue;
        end
        path = backtrackPath(nodes, current.node_id, stats, timer_id);
        path.search_nodes = nodes(1:node_count);
        return;
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
        node_count = node_count + 1;
        child.node_id = node_count;
        nodes(node_count) = child;
        open_set.push(child);
        stats.open_peak = max(stats.open_peak, open_set.count());
    end
end

path.error_code = uint8(4);
path.search_statistics = finishStats(stats, timer_id);
path.search_nodes = nodes(1:node_count);
end

function path = backtrackPath(nodes, goal_id, stats, timer_id)
%BACKTRACKPATH 沿 parent_id 回溯并输出带速度/时间的路径。
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
path.x = zeros(numel(indices),1); path.y = path.x; path.theta = path.x;
path.v = path.x; path.t = path.x; path.direction = path.x;
for k = 1:numel(indices)
    node = nodes(indices(k));
    path.x(k) = node.x; path.y(k) = node.y; path.theta(k) = node.theta;
    path.v(k) = node.v; path.t(k) = node.t; path.direction(k) = node.direction;
end
path.direction(1) = 0;
path.valid = true;
path.error_code = uint8(0);
stats.path_nodes = numel(indices);
path.search_statistics = finishStats(stats, timer_id);
end

function flag = isGoal(node, goal_state, cfg)
%ISGOAL 判断位置和航向是否达到目标容差。
if isempty(node) || ~isscalar(node.x)
    flag = false;
    return;
end
position_ok = hypot(node.x-goal_state(1), node.y-goal_state(2)) ...
    <= cfg.goal_position_tolerance_m;
heading_ok = abs(wrapToPiLocal(node.theta-goal_state(3))) ...
    <= cfg.goal_heading_tolerance_rad;
flag = position_ok && heading_ok;
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
    'search_statistics',initStats());
end

function stats = initStats()
%INITSTATS 初始化 V-Hybrid A* 搜索统计。
stats = struct('expanded_nodes',0,'generated_nodes',0,'sampled_controls',0, ...
    'collision_pruned_nodes',0,'constraint_pruned_nodes',0,'closed_pruned_nodes',0, ...
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
