function result = generate_day3_summon_plot(vehicle_id)
%GENERATE_DAY3_SUMMON_PLOT 为 Day 2 召集路径增加速度-时间参数。
%
% 输入：vehicle_id - Day 2 场景中的车辆 ID，默认 vehicle_002。
% 输出：result.path、result.time_s、result.speed_mps、result.valid。
% 算法逻辑：复用 Day 2 的空间路径，并调用 Day 3 运动学函数更新速度。

if nargin < 1, vehicle_id = 'vehicle_001'; end
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

% 速度取绝对值，行驶方向由 path.direction 单独表示。
dt = dynamics.default_dt_s;
cruise_speed = min(1.5, dynamics.v_max_mps);
speed_abs = zeros(n, 1);
time_s = zeros(n, 1);
dynamic_state = [0, 0, 0, min(0.5, dynamics.v_max_mps), 0];
speed_abs(1) = dynamic_state(4);
for k = 2:n
    acceleration = (cruise_speed - dynamic_state(4)) / dt;
    acceleration = min(max(acceleration, dynamics.a_min_mps2), dynamics.a_max_mps2);
    [dynamic_state, ~, ~, valid] = summon_vehicle_dynamic(dynamic_state, 0, ...
        acceleration, dt, vehicle_config.dimensions_m.wheelbase, ...
        dynamics.v_max_mps, dynamics.v_min_mps, dynamics.a_min_mps2, ...
        dynamics.a_max_mps2, dynamics.delta_max_rad);
    if ~valid || dynamic_state(4) <= 0
        error('Summon:Day3:InvalidSpeedUpdate', '第 %d 个路径点速度更新失败。', k);
    end
    speed_abs(k) = dynamic_state(4);
    time_s(k) = time_s(k-1) + ds(k) / max(0.5 * (speed_abs(k-1) + speed_abs(k)), eps);
end
direction = sign(path.direction);
direction(1) = 0;
speed_mps = speed_abs .* direction;
speed_mps(1) = 0;
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
plot(time_s, speed_mps, 'r-', 'LineWidth', 1.8); hold on;
plot(time_s, speed_abs, 'b--', 'LineWidth', 1.0);
grid on; xlabel('t / s'); ylabel('v / (m/s)');
title(sprintf('Day 3 velocity-time profile, %s', char(vehicle_id)));
legend('signed speed', 'speed magnitude', 'Location', 'best');
exportgraphics(fig, vt_output, 'Resolution', 160); close(fig);
save(fullfile(output_dir, ['day3_summon_' char(vehicle_id) '.mat']), 'result');
fprintf('vehicle=%s, samples=%d, path_length=%.4f m, duration=%.4f s\n', ...
    char(vehicle_id), n, sum(ds), time_s(end));
fprintf('path_output=%s\nvt_output=%s\n', path_output, vt_output);
end
