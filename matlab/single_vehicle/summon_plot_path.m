function statistics = summon_plot_path(path, map, vehicle_config, output_file)
%SUMMON_PLOT_PATH 绘制并保存独立单车召集路径。
%
% 输入：
%   path          - summon_hybrid_astar 输出的统一路径结构。
%   map           - summon_map 输出的地图结构。
%   vehicle_config- 车辆尺寸配置。
%   output_file   - 可选 SVG 输出路径。
%
% 输出：
%   statistics    - 原样返回 path.search_statistics，便于脚本打印。
%
% 算法逻辑：
%   绘制道路边界、静态障碍物、路径、起点/终点和车辆尺寸示意。
%   本函数不参与规划，不修改 path。

statistics = path.search_statistics;
if isempty(path.x)
    return;
end

fig = figure('Visible', 'off', 'Color', 'w', 'Name', 'Summon single vehicle');
hold on; grid on; axis equal;
plot(map.boundary_xy(:,1), map.boundary_xy(:,2), 'k-', 'LineWidth', 1.5, ...
    'DisplayName', 'road boundary');
for k = 1:numel(map.obstacles)
    obstacle = map.obstacles{k};
    patch(obstacle(:,1), obstacle(:,2), [0.45,0.45,0.45], ...
        'DisplayName', 'static obstacle');
end
plot(path.x, path.y, 'b-', 'LineWidth', 1.6, 'DisplayName', 'Hybrid A* path');
plot(path.x(1), path.y(1), 'go', 'MarkerFaceColor', 'g', ...
    'DisplayName', 'start');
plot(path.x(end), path.y(end), 'rx', 'LineWidth', 2, 'MarkerSize', 10, ...
    'DisplayName', 'actual endpoint');
if isfield(path,'goal_pose')
    g = path.goal_pose;
    plot(g(1),g(2),'kp','MarkerSize',13,'DisplayName','requested goal');
    quiver(g(1),g(2),2*cos(g(3)),2*sin(g(3)),0,'k--', ...
        'LineWidth',2,'DisplayName','requested heading');
end
quiver(path.x(end),path.y(end),2*cos(path.theta(end)),2*sin(path.theta(end)), ...
    0,'r-','LineWidth',1.5,'DisplayName','actual heading');

drawVehicle(path.x(1), path.y(1), path.theta(1), vehicle_config, [0.2,0.65,0.95]);
drawVehicle(path.x(end), path.y(end), path.theta(end), vehicle_config, [1.0,0.65,0.2]);
xlabel('x / m'); ylabel('y / m');
title(sprintf('Single-vehicle summon path, expanded = %d', ...
    statistics.expanded_nodes));
legend('Location', 'best');

if nargin >= 4 && ~isempty(output_file)
    output_dir = fileparts(output_file);
    if ~isempty(output_dir) && ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    exportgraphics(fig, output_file, 'Resolution', 160);
end
close(fig);
end

function drawVehicle(x, y, theta, vehicle_config, color)
%DRAWVEHICLE 按车辆尺寸绘制旋转矩形示意。
front = double(vehicle_config.dimensions_m.front_axle_to_front) + ...
    double(vehicle_config.dimensions_m.wheelbase);
rear = double(vehicle_config.dimensions_m.rear_axle_to_rear);
half_width = double(vehicle_config.dimensions_m.width) / 2;
local = [front,half_width; front,-half_width; -rear,-half_width; -rear,half_width; front,half_width];
rotation = [cos(theta),-sin(theta); sin(theta),cos(theta)];
corners = (rotation * local.').';
corners = corners + [x,y];
patch(corners(:,1), corners(:,2), color, 'FaceAlpha', 0.25, 'EdgeColor', color, ...
    'HandleVisibility','off');
end
