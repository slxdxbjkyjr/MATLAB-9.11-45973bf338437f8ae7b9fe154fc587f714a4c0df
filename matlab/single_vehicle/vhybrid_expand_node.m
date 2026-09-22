function [children, statistics] = vhybrid_expand_node(parent_node, map, ...
    vehicle_config, planner_config, goal_state)
%VHYBRID_EXPAND_NODE 对节点采样加速度和转角并生成子节点。
%
% 输入：
%   parent_node    - vhybrid_node 结构体，状态为 [x,y,theta,v,t]。
%   map            - summon_map 输出的静态地图。
%   vehicle_config - 车辆尺寸和转角配置。
%   planner_config - dynamics/vhybrid 配置。
%   goal_state     - 真实目标状态 [x,y,theta,v,t]，用于启发代价。
%
% 输出：
%   children       - 通过运动学和静态碰撞检查的子节点数组。
%   statistics     - 采样总数、有效数、碰撞裁剪数。
%
% 算法逻辑：
%   对每个节点组合 [-a_max,0,+a_max] 和 [-delta_max,0,+delta_max]。
%   每个组合调用第 3 天 summon_vehicle_dynamic，使用时间步长推进状态，
%   再调用 summon_collision_check。离散索引包含 x/y/yaw/v/time 五维。

cfg = planner_config.vhybrid;
dynamics = planner_config.dynamics;
acceleration_values = [-dynamics.a_max_mps2, 0, dynamics.a_max_mps2];
steering_values = [-vehicle_config.steering.maximum_steer_rad, 0, ...
    vehicle_config.steering.maximum_steer_rad];
children = repmat(vhybrid_node(), 0, 1);
statistics = struct('sampled', 0, 'valid', 0, 'collision_pruned', 0, ...
    'constraint_pruned', 0);

for acceleration = acceleration_values
    for steering_angle = steering_values
        statistics.sampled = statistics.sampled + 1;
        current_state = [parent_node.x, parent_node.y, parent_node.theta, ...
            parent_node.v, parent_node.t];
        [next_state, distance, ~, valid] = summon_vehicle_dynamic( ...
            current_state, steering_angle, acceleration, cfg.time_step_s, ...
            vehicle_config.dimensions_m.wheelbase, dynamics.v_max_mps, ...
            dynamics.v_min_mps, dynamics.a_min_mps2, dynamics.a_max_mps2, ...
            vehicle_config.steering.maximum_steer_rad);
        if ~valid
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            continue;
        end
        [is_collision, ~] = summon_collision_check(next_state(1:3), map, vehicle_config);
        if is_collision
            statistics.collision_pruned = statistics.collision_pruned + 1;
            continue;
        end

        indices = discretizeState(next_state, map, cfg);
        [g_cost, h_cost, ~, ~] = vhybrid_cost(parent_node, next_state, ...
            goal_state, ...
            planner_config, distance, steering_angle);
        child = vhybrid_node(next_state, acceleration, steering_angle, ...
            parent_node.node_id, g_cost, h_cost, indices, 0);
        child.direction = sign(distance);
        child.travelled_distance = distance;
        children(end+1,1) = child; %#ok<AGROW>
        statistics.valid = statistics.valid + 1;
    end
end
end

function indices = discretizeState(state, map, cfg)
%DISCRETIZESTATE 将连续状态离散为五维查重索引。
indices = [floor((state(1) - map.bounds(1)) / cfg.xy_grid_resolution_m), ...
    floor((state(2) - map.bounds(2)) / cfg.xy_grid_resolution_m), ...
    round((wrapToPiLocal(state(3)) + pi) / cfg.yaw_grid_resolution_rad), ...
    round(state(4) / cfg.velocity_grid_resolution_mps), ...
    round(state(5) / cfg.time_grid_resolution_s)];
end

function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将航向角归一化到 [-pi,pi)。
angle = mod(angle + pi, 2*pi) - pi;
end
