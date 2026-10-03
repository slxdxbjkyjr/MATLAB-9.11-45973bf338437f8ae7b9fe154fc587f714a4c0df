function result = generate_day3_summon_plot(vehicle_id)
%GENERATE_DAY3_SUMMON_PLOT 为 Day 2 召集路径增加速度-时间参数。
%
% 输入：vehicle_id - Day 2 场景中的车辆 ID，默认 vehicle_002。
% 输出：result.path、result.time_s、result.speed_mps、result.valid。
% 算法逻辑：复用 Day 2 的空间路径，并调用 Day 3 运动学函数更新速度。

if nargin < 1, vehicle_id = 'vehicle_003'; end
this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(this_dir);
source_dir = fullfile(project_dir, 'matlab', 'single_vehicle');
addpath(source_dir);
config_dir = fullfile(project_dir, 'config');
vehicle_config = jsondecode(fileread(fullfile(config_dir, 'vehicle_config.json')));
map_config = jsondecode(fileread(fullfile(config_dir, 'map_config.json')));
planner_config = jsondecode(fileread(fullfile(config_dir, 'planner_config.json')));
scenario = jsondecode(fileread(fullfile(config_dir, 'scenario_001.json')));
map = summon_map(map_config);
vehicle = scenario.vehicles(strcmp({scenario.vehicles.id}, vehicle_id));
if isempty(vehicle), error('Summon:Day3:VehicleNotFound', '未找到车辆 %s。', vehicle_id); end

start_state = [vehicle.start_pose.x_m, vehicle.start_pose.y_m, vehicle.start_pose.yaw_rad];
goal_state = [vehicle.summon_goal.x_m, vehicle.summon_goal.y_m, vehicle.summon_goal.yaw_rad];
path = summon_hybrid_astar(start_state, goal_state, map, vehicle_config, planner_config);
path.goal_pose = goal_state;
if ~path.valid
    error('Summon:Day3:PlanningFailed', 'Day 2 路径规划失败，error_code=%d。', path.error_code);
end

dynamics = planner_config.dynamics;
n = numel(path.x);
ds = [0; hypot(diff(path.x), diff(path.y))];
if any(~isfinite(ds)) || any(ds(2:end) <= 0)
    error('Summon:Day3:InvalidPathSpacing', 'Day 2 路径包含无效或重复采样点。');
end

direction = sign(path.direction);
direction(1) = 0;
% 基于弧长做前向/后向速度优化，避免速度突变。
cruise_speed = min(1.5, dynamics.v_max_mps);
speed_abs = cruise_speed * ones(n, 1);
speed_abs(1) = 0;
speed_abs(end) = 0;
% 倒车/前进切换必须先停车，不能在同一时刻改变速度符号。
direction_for_stop = direction;
direction_for_stop(1) = direction_for_stop(find(direction_for_stop ~= 0, 1, 'first'));
stop_indices = [1; find(diff(direction_for_stop) ~= 0) + 1; n];
stop_indices = unique(stop_indices(stop_indices >= 1 & stop_indices <= n));
speed_abs(stop_indices) = 0;
for pass = 1:3
    for k = 2:n
        reachable = sqrt(max(0, speed_abs(k-1)^2 + ...
            2 * dynamics.a_max_mps2 * ds(k)));
        speed_abs(k) = min(speed_abs(k), reachable);
        if any(stop_indices == k), speed_abs(k) = 0; end
    end
    for k = n-1:-1:1
        reachable = sqrt(max(0, speed_abs(k+1)^2 + ...
            2 * abs(dynamics.a_min_mps2) * ds(k+1)));
        speed_abs(k) = min(speed_abs(k), reachable);
        if any(stop_indices == k), speed_abs(k) = 0; end
    end
end

% 按优化后的速度计算行驶时间，并用 Day 3 动力学函数逐段校核加速度。
arrival_time_s = zeros(n, 1);
travel_dt_s = zeros(n, 1);
for k = 2:n
    mean_speed = 0.5 * (speed_abs(k-1) + speed_abs(k));
    if ds(k) > 0 && mean_speed <= 0
        error('Summon:Day3:ZeroSpeedSegment', ...
            '第 %d 段路径在非零距离上出现零平均速度。', k);
    end
    segment_dt = ds(k) / max(mean_speed, eps);
    acceleration = (speed_abs(k) - speed_abs(k-1)) / segment_dt;
    acceleration = min(max(acceleration, dynamics.a_min_mps2), dynamics.a_max_mps2);
    test_state = [0, 0, 0, speed_abs(k-1), arrival_time_s(k-1)];
    [next_state, travelled, ~, valid] = summon_vehicle_dynamic(test_state, 0, ...
        acceleration, segment_dt, vehicle_config.dimensions_m.wheelbase, ...
        dynamics.v_max_mps, dynamics.v_min_mps, dynamics.a_min_mps2, ...
        dynamics.a_max_mps2, dynamics.delta_max_rad);
    if ~valid || abs(next_state(4) - speed_abs(k)) > 1e-9 || ...
            abs(travelled - ds(k)) > 1e-8
        error('Summon:Day3:InvalidSpeedProfile', ...
            '第 %d 段速度轨迹未通过动力学校核：a=%.9f, ds=%.9f, dt=%.9f, travelled=%.9f。', ...
            k, acceleration, ds(k), segment_dt, travelled);
    end
    travel_dt_s(k) = segment_dt;
    arrival_time_s(k) = arrival_time_s(k-1) + segment_dt;
end
speed_mps = speed_abs .* direction;
speed_mps(1) = 0;
% 换挡等待从到达换挡点之后开始，不改变车辆行驶段的加速度。
wait_time_s = zeros(n, 1);
if isfield(dynamics, 'direction_change_time_s')
    direction_change_time_s = double(dynamics.direction_change_time_s);
else
    direction_change_time_s = 0.5;
end
direction_change_indices = stop_indices(stop_indices > 1 & stop_indices < n);
wait_time_s(direction_change_indices) = direction_change_time_s;
time_s = arrival_time_s;
accumulated_wait = 0;
for k = 2:n
    accumulated_wait = accumulated_wait + wait_time_s(k-1);
    time_s(k) = arrival_time_s(k) + accumulated_wait;
end
max_acceleration = max(diff(speed_abs(1:end)) ./ max(travel_dt_s(2:end), eps));
min_acceleration = min(diff(speed_abs(1:end)) ./ max(travel_dt_s(2:end), eps));
if max_acceleration > dynamics.a_max_mps2 + 1e-9 || ...
        min_acceleration < dynamics.a_min_mps2 - 1e-9
    error('Summon:Day3:AccelerationLimit', '优化速度曲线超过加速度约束。');
end
result = struct('path', path, 'time_s', time_s, 'speed_mps', speed_mps, ...
    'speed_abs_mps', speed_abs, 'valid', true);

output_dir = fullfile(project_dir, 'outputs');
if ~exist(output_dir, 'dir'), mkdir(output_dir); end
path_output = fullfile(output_dir, ['day3_summon_path_' char(vehicle_id) '.png']);
vt_output = fullfile(output_dir, ['day3_summon_vt_' char(vehicle_id) '.png']);

fig = figure('Visible', 'off', 'Color', 'w');
plot(map.boundary_xy(:,1), map.boundary_xy(:,2), 'k-', 'LineWidth', 1.5); hold on;
for k = 1:numel(map.obstacles)
    obstacle = map.obstacles{k}; patch(obstacle(:,1), obstacle(:,2), [0.45,0.45,0.45]);
end
plot(path.x, path.y, 'b-', 'LineWidth', 1.8);
plot(path.x(1), path.y(1), 'go', 'MarkerFaceColor', 'g');
plot(path.x(end), path.y(end), 'rx', 'LineWidth', 2, 'MarkerSize', 10);
plot(goal_state(1), goal_state(2), 'kp', 'MarkerSize', 12);
axis equal; grid on; xlabel('x / m'); ylabel('y / m');
title(sprintf('Day 3 summon path with speed, %s', char(vehicle_id)));
legend('road boundary', 'Hybrid A* path', 'start', 'actual endpoint', 'requested goal', 'Location', 'best');
exportgraphics(fig, path_output, 'Resolution', 160); close(fig);

fig = figure('Visible', 'off', 'Color', 'w');
plot_time_s = time_s;
plot_speed_mps = speed_mps;
plot_speed_abs = speed_abs;
for k = fliplr(direction_change_indices.')
    plot_time_s = [plot_time_s(1:k); time_s(k) + direction_change_time_s; plot_time_s(k+1:end)]; %#ok<AGROW>
    plot_speed_mps = [plot_speed_mps(1:k); 0; plot_speed_mps(k+1:end)]; %#ok<AGROW>
    plot_speed_abs = [plot_speed_abs(1:k); 0; plot_speed_abs(k+1:end)]; %#ok<AGROW>
end
plot(plot_time_s, plot_speed_mps, 'r-', 'LineWidth', 1.8); hold on;
plot(plot_time_s, plot_speed_abs, 'b--', 'LineWidth', 1.0);
grid on; xlabel('t / s'); ylabel('v / (m/s)');
title(sprintf('Day 3 velocity-time profile, %s', char(vehicle_id)));
legend('signed speed', 'speed magnitude', 'Location', 'best');
exportgraphics(fig, vt_output, 'Resolution', 160); close(fig);
save(fullfile(output_dir, ['day3_summon_' char(vehicle_id) '.mat']), 'result');
fprintf('vehicle=%s, samples=%d, path_length=%.4f m, duration=%.4f s\n', ...
    char(vehicle_id), n, sum(ds), time_s(end));
fprintf('terminal_speed=%.6f m/s (target stop)\n', speed_mps(end));
fprintf('acceleration_range=[%.6f, %.6f] m/s^2\n', min_acceleration, max_acceleration);
fprintf('direction_change_wait=%.3f s, wait_events=%d\n', ...
    direction_change_time_s, numel(direction_change_indices));
fprintf('path_output=%s\nvt_output=%s\n', path_output, vt_output);
end
