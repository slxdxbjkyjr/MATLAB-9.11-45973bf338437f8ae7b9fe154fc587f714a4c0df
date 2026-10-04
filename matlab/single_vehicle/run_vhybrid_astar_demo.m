function path = run_vhybrid_astar_demo(vehicle_index, save_outputs)
%RUN_VHYBRID_ASTAR_DEMO 运行 Day 4 单车 V-Hybrid A* 原型。
%
% 输入：vehicle_index 为场景车辆序号（默认 3）；save_outputs 是否保存图（默认 true）。
% 输出：path.x/y/theta/v/t/direction、valid、error_code 和搜索统计。
%
% 算法逻辑：
%   选取 Day 1 场景指定车辆；Open 使用最小 f_cost 节点选择，Closed
%   使用 x/y/yaw/v/time 五维索引查重。Day4入口使用统一搜索器的无动态参数模式。
%   完成后输出搜索节点、最终路径以及速度-时间曲线图。

if nargin < 1, vehicle_index = 3; end
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
    vehicle.start_pose.yaw_rad, planner_config.vhybrid.initial_speed_mps, 0];
% 起步速度集中配置为0；后续速度和位移均通过运动学模型受限加速生成。
goal_state = [vehicle.summon_goal.x_m, vehicle.summon_goal.y_m, ...
    vehicle.summon_goal.yaw_rad, 0, 0];

path = summon_vhybrid_astar(start_state,goal_state,map,vehicle_config,planner_config);
if save_outputs, saveDemoPlots(path,map,vehicle_config,project_dir); end
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
