function output_file = generate_vehicle_dynamic_plot()
%GENERATE_VEHICLE_DYNAMIC_PLOT 生成 Day 3 车辆运动学测试轨迹图。
%
% 输出：
%   output_file - 生成的 PNG 文件路径。
%
% 算法逻辑：
%   在同一张图中依次展示直行加速、匀速左转、减速右转三段运动，
%   每一步均调用 summon_vehicle_dynamic，轨迹状态包含 x/y/theta/v/t。

this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(this_dir);
source_dir = fullfile(project_dir, 'matlab', 'single_vehicle');
addpath(source_dir);
config_dir = fullfile(project_dir, 'config');
planner_config = jsondecode(fileread(fullfile(config_dir, 'planner_config.json')));
vehicle_config = jsondecode(fileread(fullfile(config_dir, 'vehicle_config.json')));
dynamics = planner_config.dynamics;
L = vehicle_config.dimensions_m.wheelbase;

state = [3, 3, 0, 0.5, 0];
trajectory = state;
for k = 1:20
    [state, ~, ~, valid] = summon_vehicle_dynamic(state, 0, 0.5, ...
        dynamics.default_dt_s, L, dynamics.v_max_mps, dynamics.v_min_mps, ...
        dynamics.a_min_mps2, dynamics.a_max_mps2, dynamics.delta_max_rad);
    if ~valid
        error('Summon:DynamicPlot:InvalidAccelerationSegment', ...
            '加速测试段违反了集中配置的速度/加速度约束。');
    end
    trajectory(end+1,:) = state; %#ok<AGROW>
end
for k = 1:25
    [state, ~, ~, valid] = summon_vehicle_dynamic(state, 0.20, 0, ...
        dynamics.default_dt_s, L, dynamics.v_max_mps, dynamics.v_min_mps, ...
        dynamics.a_min_mps2, dynamics.a_max_mps2, dynamics.delta_max_rad);
    if ~valid
        error('Summon:DynamicPlot:InvalidTurnSegment', ...
            '左转测试段违反了集中配置的速度/转角约束。');
    end
    trajectory(end+1,:) = state; %#ok<AGROW>
end
for k = 1:10
    [state, ~, ~, valid] = summon_vehicle_dynamic(state, -0.20, -1, ...
        dynamics.default_dt_s, L, dynamics.v_max_mps, dynamics.v_min_mps, ...
        dynamics.a_min_mps2, dynamics.a_max_mps2, dynamics.delta_max_rad);
    if ~valid
        error('Summon:DynamicPlot:InvalidDecelerationSegment', ...
            '减速测试段违反了集中配置的速度/加速度约束。');
    end
    trajectory(end+1,:) = state; %#ok<AGROW>
end

output_file = fullfile(project_dir, 'outputs', 'day3_vehicle_dynamic.png');
fig = figure('Visible', 'off', 'Color', 'w');
tiledlayout(2,1);
nexttile;
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 1.5);
hold on; plot(trajectory(1,1), trajectory(1,2), 'go', 'MarkerFaceColor', 'g');
plot(trajectory(end,1), trajectory(end,2), 'rx', 'LineWidth', 2);
grid on; axis equal; xlabel('x / m'); ylabel('y / m');
title('Day 3 vehicle kinematic trajectory');
nexttile;
plot(trajectory(:,5), trajectory(:,4), 'r-', 'LineWidth', 1.5);
grid on; xlabel('t / s'); ylabel('v / m/s');
title('Velocity profile');
exportgraphics(fig, output_file);
close(fig);
fprintf('trajectory_samples=%d, final_state=[%.4f %.4f %.4f %.4f %.4f]\n', ...
    size(trajectory,1), trajectory(end,1), trajectory(end,2), trajectory(end,3), ...
    trajectory(end,4), trajectory(end,5));
end
