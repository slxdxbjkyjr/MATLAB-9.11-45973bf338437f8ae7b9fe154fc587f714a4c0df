function path = run_single_vehicle_demo(vehicle_id)
%RUN_SINGLE_VEHICLE_DEMO 使用 Day 1 指定车辆执行单车规划。
%
% 输入：可选 vehicle_id，默认 vehicle_002。读取本工程 JSON 配置。
% 输出：path，包含统一的路径字段和搜索统计信息。
%
% 算法逻辑：
%   读取 vehicle_config、map_config、planner_config、scenario_001，
%   按车辆 ID 选取起点和对应召集点。
%   本脚本只调用独立 summon_hybrid_astar，不加载原 APA 模型。

this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(fileparts(this_dir));
config_dir = fullfile(project_dir, 'config');
if nargin < 1, vehicle_id = 'vehicle_001'; end
output_file = fullfile(project_dir, 'outputs', ['day2_single_' char(vehicle_id) '.png']);

vehicle_config = jsondecode(fileread(fullfile(config_dir, 'vehicle_config.json')));
map_config = jsondecode(fileread(fullfile(config_dir, 'map_config.json')));
planner_config = jsondecode(fileread(fullfile(config_dir, 'planner_config.json')));
scenario = jsondecode(fileread(fullfile(config_dir, 'scenario_001.json')));

map = summon_map(map_config);
vehicle = scenario.vehicles(strcmp({scenario.vehicles.id}, vehicle_id));
if isempty(vehicle), error('Summon:Demo:VehicleNotFound', '未找到车辆 %s。', vehicle_id); end
start_state = [vehicle.start_pose.x_m, vehicle.start_pose.y_m, vehicle.start_pose.yaw_rad];
goal_state = [vehicle.summon_goal.x_m, vehicle.summon_goal.y_m, vehicle.summon_goal.yaw_rad];

path = summon_hybrid_astar(start_state, goal_state, map, vehicle_config, planner_config);
path.goal_pose = goal_state;
if path.valid
    path.position_error_m = hypot(path.x(end)-goal_state(1),path.y(end)-goal_state(2));
    path.heading_error_deg = abs(mod(path.theta(end)-goal_state(3)+pi,2*pi)-pi)*180/pi;
    fprintf('goal=[%.6f %.6f %.6f], actual=[%.6f %.6f %.6f]\n', ...
        goal_state,path.x(end),path.y(end),path.theta(end));
    fprintf('position_error_m=%.9f, heading_error_deg=%.9f\n', ...
        path.position_error_m,path.heading_error_deg);
end
summon_plot_path(path, map, vehicle_config, output_file);
save(strrep(output_file,'.png','.mat'),'path');

fprintf('valid=%d, error_code=%d\n', path.valid, path.error_code);
fprintf('expanded_nodes=%d, generated_nodes=%d, collision_pruned=%d\n', ...
    path.search_statistics.expanded_nodes, path.search_statistics.generated_nodes, ...
    path.search_statistics.collision_pruned_nodes);
fprintf('elapsed_sec=%.6f, output=%s\n', path.search_statistics.elapsed_sec, output_file);
end
