function [g_cost, h_cost, f_cost, terms] = vhybrid_cost(parent_node, ...
    child_state, goal_state, planner_config, travelled_distance, steering_angle)
%VHYBRID_COST 计算 V-Hybrid A* 节点代价。
%
% 输入：
%   parent_node    - 父节点结构体。
%   child_state    - 子节点 [x,y,theta,v,t]。
%   goal_state     - 目标状态 [x,y,theta,v,t]；目标速度/时间可为参考值。
%   planner_config - 含 vhybrid 权重的规划配置。
%   travelled_distance - 本步带方向距离，单位 m。
%   steering_angle - 本步前轮转角，单位 rad。
%
% 输出：
%   g_cost/h_cost/f_cost - 累积、启发和总评价代价。
%   terms              - 各代价分项，便于调试和后续调参。
%
% 算法逻辑：
%   g 代价包含行驶距离、速度变化、航向变化和时间；h 代价包含到目标位置、
%   航向和时间的估计。所有权重从 planner_config.vhybrid 读取，不散落在代码中。

cfg = planner_config.vhybrid;
child_state = double(child_state(:).');
distance_term = abs(double(travelled_distance));
speed_term = abs(child_state(4) - parent_node.v);
heading_term = abs(wrapToPiLocal(child_state(3) - parent_node.theta));
time_term = max(0, child_state(5) - parent_node.t);
goal_distance_term = hypot(child_state(1) - goal_state(1), child_state(2) - goal_state(2));
goal_heading_term = abs(wrapToPiLocal(child_state(3) - goal_state(3)));
goal_time_term = abs(child_state(5) - goal_state(5));

terms = struct(...
    'distance', distance_term, ...
    'speed_change', speed_term, ...
    'heading_change', heading_term, ...
    'time', time_term, ...
    'goal_distance', goal_distance_term, ...
    'goal_heading', goal_heading_term, ...
    'goal_time', goal_time_term, ...
    'steering', abs(double(steering_angle)));

g_cost = parent_node.g_cost + cfg.weight_distance * distance_term ...
    + cfg.weight_speed_change * speed_term ...
    + cfg.weight_heading_change * heading_term ...
    + cfg.weight_time * time_term;
h_cost = cfg.weight_goal_distance * (goal_distance_term + 0.5 * goal_heading_term);
f_cost = g_cost + h_cost;
end

function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将角度归一化到 [-pi,pi)。
angle = mod(angle + pi, 2*pi) - pi;
end
