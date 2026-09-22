function output_file = generate_vhybrid_plot()
%GENERATE_VHYBRID_PLOT 运行 V-Hybrid A* Demo 并生成搜索/轨迹图。
%
% 输出：
%   output_file - Day 4 PNG 图路径。

this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(this_dir);
source_dir = fullfile(project_dir, 'matlab', 'single_vehicle');
addpath(source_dir);
path = run_vhybrid_astar_demo();
output_file = fullfile(project_dir, 'outputs', 'day4_vhybrid_astar.png');
fig = figure('Visible','off','Color','w');
tiledlayout(2,2);
nexttile;
if ~isempty(path.search_nodes)
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
nexttile([1 2]);
if ~isempty(path.t)
    yyaxis left; plot(path.t, path.v, 'r-', 'LineWidth',1.5); ylabel('v / m/s');
    yyaxis right; plot(path.t, path.t, 'k--', 'LineWidth',1.2); ylabel('t / s');
else
    text(0.2,0.5,'搜索失败，未生成最终路径'); axis off;
end
grid on; xlabel('path sample time / s'); title('Path velocity and time');
exportgraphics(fig, output_file);
close(fig);
stats = path.search_statistics;
fprintf('valid=%d error_code=%d expanded=%d generated=%d elapsed=%.6f\n', ...
    path.valid, path.error_code, stats.expanded_nodes, stats.generated_nodes, stats.elapsed_sec);
end
