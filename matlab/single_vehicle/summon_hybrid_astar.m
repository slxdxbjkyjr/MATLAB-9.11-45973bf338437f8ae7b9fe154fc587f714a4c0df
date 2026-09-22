function path = summon_hybrid_astar(start_state, goal_state, map, vehicle_config, planner_config)
%SUMMON_HYBRID_ASTAR 独立最小版单车 Hybrid A* 搜索器。
%
% 输入：
%   start_state   - [x,y,theta] 起始位姿。
%   goal_state    - [x,y,theta] 目标位姿。
%   map           - summon_map 生成的地图结构。
%   vehicle_config- vehicle_config.json 解码后的车辆配置。
%   planner_config- planner_config.json 解码后的规划配置。
%
% 输出：
%   path.x/path.y/path.theta - 路径离散点。
%   path.direction           - 每个点的行驶方向，-1/0/+1。
%   path.valid               - 是否成功。
%   path.error_code          - 0 成功，1 输入错误，2 起点碰撞，3 终点碰撞，
%                              4 Open 为空，5 达到扩展上限。
%   path.search_statistics   - 扩展数、生成数、碰撞裁剪数和耗时。
%
% 算法逻辑：
%   该实现提取原 Local_Planner 的核心思想：连续车辆状态、离散栅格查重、
%   双向行驶扩展、代价加启发函数、父节点回溯。停车专用车位逻辑、APA 状态机、
%   Geo_Planning1、Planner_Core、超声波和 Moco 均已移除。

path = emptyPath();
tic_id = tic;
stats = initStatistics();

if numel(start_state) ~= 3 || numel(goal_state) ~= 3
    path.error_code = uint8(1);
    path.search_statistics = finishStatistics(stats, tic_id);
    return;
end

search = planner_config.search;
if ~isfield(search,'goal_connection_distance_m'), search.goal_connection_distance_m = 8; end 
start_state = double(start_state(:).');
goal_state = double(goal_state(:).');
[start_collision, ~] = summon_collision_check(start_state, map, vehicle_config);
[goal_collision, ~] = summon_collision_check(goal_state, map, vehicle_config);
if start_collision
    path.error_code = uint8(2);
    path.search_statistics = finishStatistics(stats, tic_id);
    return;
end
if goal_collision
    path.error_code = uint8(3);
    path.search_statistics = finishStatistics(stats, tic_id);
    return;
end

max_nodes = double(search.max_expansions) + 1;
nodes = repmat(emptyNode(), max_nodes, 1); 
open_mask = false(max_nodes, 1); 
closed_keys = containers.Map('KeyType', 'char', 'ValueType', 'uint32');

nodes(1) = makeNode(start_state, 0, heuristicCost(start_state, goal_state, search), ...
    0, 0, 0, 0);
open_mask(1) = true;
node_count = 1;
stats.open_peak = 1;

while any(open_mask)
    open_indices = find(open_mask);
    [~, local_idx] = min([nodes(open_indices).f_cost]);
    current_index = open_indices(local_idx);
    current = nodes(current_index);
    open_mask(current_index) = false;
    current_key = stateKey(current.pose, map, search);
    if isKey(closed_keys,current_key)
        stats.closed_pruned_nodes = stats.closed_pruned_nodes + 1;
        continue;
    end
    closed_keys(current_key) = uint32(current_index);
    stats.expanded_nodes = stats.expanded_nodes + 1;

    if isGoalReached(current.pose, goal_state, search)
        path = getFinalPath(nodes, current_index, stats, tic_id, vehicle_config, search);
        return;
    end
    [connection, curvature] = summon_goal_connection(current.pose, goal_state, map, vehicle_config, search);
    if ~isempty(connection)
        path = getFinalPath(nodes, current_index, stats, tic_id, vehicle_config, search);
        path.x = [path.x; connection(2:end,1)];
        path.y = [path.y; connection(2:end,2)];
        path.theta = [path.theta; connection(2:end,3)];
        path.direction = [path.direction; ones(size(connection,1)-1,1)];
        path.connection_curvature = curvature;
        path.search_statistics.elapsed_sec = toc(tic_id);
        return;
    end
    if stats.expanded_nodes >= double(search.max_expansions)
        path.error_code = uint8(5);
        path.search_statistics = finishStatistics(stats, tic_id);
        return;
    end

    [nodes, open_mask, node_count, stats] = HybridAStar_Update( ...
        nodes, open_mask, node_count, current, current_index, closed_keys, ...
        goal_state, map, vehicle_config, search, stats);
end

path.error_code = uint8(4);
path.search_statistics = finishStatistics(stats, tic_id);
end

function [nodes, open_mask, node_count, stats] = HybridAStar_Update( ...
    nodes, open_mask, node_count, current, current_index, closed_keys, ...
    goal_state, map, vehicle_config, search, stats)
%HYBRIDASTAR_UPDATE 产生前进/倒车和离散转角后继节点。

steer_values = linspace(-vehicle_config.steering.maximum_steer_rad, ...
    vehicle_config.steering.maximum_steer_rad, double(search.steer_sample_count));
for direction = search.direction_set(:).'
    for steer = steer_values
        [is_valid, child_pose] = CalcNextNode(current.pose, direction, steer, ...
            map, vehicle_config, search);
        stats.generated_nodes = stats.generated_nodes + 1;
        if ~is_valid
            stats.collision_pruned_nodes = stats.collision_pruned_nodes + 1;
            continue;
        end

        child_key = stateKey(child_pose, map, search);
        if isKey(closed_keys, child_key)
            stats.closed_pruned_nodes = stats.closed_pruned_nodes + 1;
            continue;
        end

        transition_cost = abs(direction) * search.motion_step_m;
        if direction < 0
            transition_cost = transition_cost * search.reverse_penalty;
        end
        if current.direction ~= 0 && sign(current.direction) ~= sign(direction)
            transition_cost = transition_cost + search.reverse_penalty;
        end
        transition_cost = transition_cost + search.steer_change_penalty * abs(steer - current.steer_rad);
        g_cost = current.g_cost + transition_cost;
        h_cost = heuristicCost(child_pose, goal_state, search);
        node_count = node_count + 1;
        if node_count > numel(nodes)
            nodes(end+1024,1) = emptyNode();
            open_mask(end+1024,1) = false;
        end
        nodes(node_count) = makeNode(child_pose, g_cost, h_cost, current_index, direction, steer, node_count);
        open_mask(node_count) = true;
        stats.open_peak = max(stats.open_peak, nnz(open_mask));
    end
end
end

function [is_valid, next_pose] = CalcNextNode(pose, direction, steer, map, vehicle_config, search)
%CALCNEXTNODE 沿车辆运动学轨迹逐小步推进并检查碰撞。

next_pose = pose;
is_valid = true;
sub_steps = max(1, ceil(search.motion_step_m / search.collision_check_step_m));
step_distance = direction * search.motion_step_m / sub_steps;
for k = 1:sub_steps
    state = summon_vehicle_state(next_pose, vehicle_config, step_distance, steer);
    next_pose = state.pose;
    [is_collision, ~] = summon_collision_check(next_pose, map, vehicle_config);
    if is_collision
        is_valid = false;
        return;
    end
end
end

function path = getFinalPath(nodes, goal_index, stats, tic_id, vehicle_config, search)
%GETFINALPATH 沿 parent_index 回溯路径，并输出统一方向字段。

indices = zeros(numel(nodes), 1);
count = 0;
index = goal_index;
while index > 0
    count = count + 1;
    indices(count) = index;
    index = nodes(index).parent_index;
end
indices = indices(count:-1:1);
path = emptyPath();
path.x = zeros(numel(indices), 1);
path.y = zeros(numel(indices), 1);
path.theta = zeros(numel(indices), 1);
path.direction = zeros(numel(indices), 1);
for k = 1:numel(indices)
    node = nodes(indices(k));
    path.x(k) = node.pose(1);
    path.y(k) = node.pose(2);
    path.theta(k) = node.pose(3);
    path.direction(k) = node.direction;
end
path.direction(1) = 0;
% 输出与搜索完全相同的圆弧，而不是用 1 m 端点间的直线代替。
dense = [path.x(1),path.y(1),path.theta(1)];
directions = 0;
n = max(1,ceil(search.motion_step_m/0.05));
for k = 2:numel(indices)
    node = nodes(indices(k));
    parent = nodes(node.parent_index);
    for j = 1:n
        state = summon_vehicle_state(parent.pose,vehicle_config, ...
            node.direction*search.motion_step_m*j/n,node.steer_rad);
        dense(end+1,:) = state.pose; %#ok<AGROW>
        directions(end+1,1) = node.direction; %#ok<AGROW>
    end
end
path.x = dense(:,1); path.y = dense(:,2); path.theta = dense(:,3);
path.direction = directions;
path.valid = true;
path.error_code = uint8(0);
stats.closed_nodes = stats.expanded_nodes;
path.search_statistics = finishStatistics(stats, tic_id);
end

function cost = heuristicCost(pose, goal_pose, search)
%GETHEURISTICCOST 使用位置欧氏距离与航向差估计剩余代价。
position_cost = hypot(pose(1) - goal_pose(1), pose(2) - goal_pose(2));
heading_cost = abs(wrapToPiLocal(pose(3) - goal_pose(3)));
cost = search.heuristic_weight * (position_cost + 0.5 * heading_cost);
end

function total = getTotalCost(g_cost, h_cost)
%GETTOTALCOST 返回 Hybrid A* 的 f=g+h 评价值。
total = g_cost + h_cost;
end

function node = makeNode(pose, g_cost, h_cost, parent_index, direction, steer, index)
%MAKENODE 构造包含连续状态、父索引和代价的搜索节点。
node = emptyNode();
node.pose = pose;
node.g_cost = g_cost;
node.h_cost = h_cost;
node.f_cost = getTotalCost(g_cost, h_cost);
node.parent_index = parent_index;
node.direction = direction;
node.steer_rad = steer;
node.index = index;
end

function tf = isGoalReached(pose, goal_pose, search)
%ISGOALREACHED 判断位置和航向是否同时进入终点容差。
tf = hypot(pose(1) - goal_pose(1), pose(2) - goal_pose(2)) <= ...
    search.goal_position_tolerance_m && ...
    abs(wrapToPiLocal(pose(3) - goal_pose(3))) <= search.goal_heading_tolerance_rad;
end

function key = stateKey(pose, map, search)
%STATEKEY 将连续状态映射到离散栅格，用于 Open/Close 查重。
x_idx = floor((pose(1) - map.bounds(1)) / search.xy_grid_resolution_m);
y_idx = floor((pose(2) - map.bounds(2)) / search.xy_grid_resolution_m);
yaw_bins = round(2*pi/search.yaw_grid_resolution_rad);
yaw_idx = mod(round((wrapToPiLocal(pose(3)) + pi) / (2*pi/yaw_bins)),yaw_bins);
key = sprintf('%d_%d_%d', x_idx, y_idx, yaw_idx);
end

function node = emptyNode()
%EMPTYNODE 返回固定字段的空节点，避免搜索过程中字段不一致。
node = struct('pose', [0,0,0], 'g_cost', 0, 'h_cost', 0, 'f_cost', 0, ...
    'parent_index', 0, 'direction', 0, 'steer_rad', 0, 'index', 0);
end

function path = emptyPath()
%EMPTYPATH 初始化统一路径输出结构。
path = struct('x', zeros(0,1), 'y', zeros(0,1), 'theta', zeros(0,1), ...
    'direction', zeros(0,1), 'valid', false, 'error_code', uint8(0), ...
    'search_statistics', initStatistics());
end

function stats = initStatistics()
%INITSTATISTICS 初始化搜索统计量。
stats = struct('expanded_nodes', 0, 'generated_nodes', 0, ...
    'collision_pruned_nodes', 0, 'closed_pruned_nodes', 0, ...
    'closed_nodes', 0, 'open_peak', 0, 'elapsed_sec', 0);
end

function stats = finishStatistics(stats, tic_id)
%FINISHSTATISTICS 记录搜索耗时并返回统计结构。
stats.elapsed_sec = toc(tic_id);
end

function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将角度归一化到 [-pi,pi)。
angle = mod(angle + pi, 2*pi) - pi;
end

