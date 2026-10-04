function output_file = generate_vhybrid_plot()
%GENERATE_VHYBRID_PLOT 运行 V-Hybrid A* Demo 并生成搜索/轨迹图。
%
% 输出：
%   output_file - 搜索节点、最终轨迹、速度和航向四格图路径。
% 算法逻辑：使用车辆 2 的静态场景，保存独立搜索图，不覆盖主轨迹图。

this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(this_dir);
source_dir = fullfile(project_dir, 'matlab', 'single_vehicle');
addpath(source_dir);
path = run_vhybrid_astar_demo(2, false);
output_file = fullfile(project_dir, 'outputs', 'day4_vhybrid_search.png');
output_dir = fileparts(output_file);
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
fig = figure('Visible','off','Color','w');
tiledlayout(2,2);
nexttile;
if isfield(path, 'search_nodes') && ~isempty(path.search_nodes)
    plot([path.search_nodes.x], [path.search_nodes.y], '.', 'Color', [0.65,0.65,0.65]);
end
hold on; plot(path.x, path.y, 'b-', 'LineWidth', 1.5);
if ~isempty(path.x)
    plot(path.x(1), path.y(1), 'go', 'MarkerFaceColor','g');
end
if ~isempty(path.x), plot(path.x(end), path.y(end), 'rx', 'LineWidth',2); end
grid on; axis equal; xlabel('x / m'); ylabel('y / m'); title('Search nodes and final path');
nexttile;
plot(path.x, path.y, 'b-', 'LineWidth', 1.5); grid on; axis equal;
xlabel('x / m'); ylabel('y / m'); title(sprintf('valid=%d, error=%d', path.valid, path.error_code));
nexttile;
if ~isempty(path.t)
    plot(path.t, path.v, 'r-', 'LineWidth',1.5); ylabel('v / m/s');
else
    text(0.2,0.5,'搜索失败，未生成最终路径'); axis off;
end
grid on; xlabel('t / s'); title('Velocity versus time');
nexttile;
if ~isempty(path.t)
    % unwrap 消除角度跨越正负 pi 时的表示跳变，保留真实运动学航向变化。
    plot(path.t, unwrap(path.theta)*180/pi, 'Color', [0.1,0.4,0.7], 'LineWidth',1.5);
    ylabel('heading / deg');
else
    text(0.2,0.5,'搜索失败，未生成最终路径'); axis off;
end
grid on; xlabel('t / s'); title('Continuous vehicle heading');
exportgraphics(fig, output_file);
close(fig);
stats = path.search_statistics;
fprintf('valid=%d error_code=%d expanded=%d generated=%d elapsed=%.6f\n', ...
    path.valid, path.error_code, stats.expanded_nodes, stats.generated_nodes, stats.elapsed_sec);
end
