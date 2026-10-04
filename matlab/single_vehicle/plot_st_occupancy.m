function [output_file, fig] = plot_st_occupancy(map, output_file, options)
%PLOT_ST_OCCUPANCY 绘制真实 XY/XT/YT 资源块切片和三维时空占用示意。
%
% 输入：map 为 st_occupancy_map 的结果，包含 owner_type/owner_id 和资源块中心。
%       output_file 可选导出图路径；options.time_s/x_m/y_m 指定切片坐标。
%       options.max_3d_points 控制三维散点数量；options.visible 控制显示窗口。
% 输出：output_file 为实际导出路径；fig 为图窗句柄。未请求 fig 时关闭图窗。
% 状态：资源块使用半开区间 [edge(i),edge(i+1))；坐标数组表示块中心。
% 逻辑：XY 固定一个时间层，XT 固定一个 y 块，YT 固定一个 x 块。
%       颜色区分边界、静态障碍、每辆动态车和多所有者占用；不使用投影冒充切片。
%       三维图仅作示意，确定性抽样不影响占用数据和任何查询。

if nargin < 2, output_file = ''; end
if nargin < 3, options = struct(); end
required = {'occupancy','owner_type','owner_id','owners', ...
    'x_coords','y_coords','t_coords','x_edges','y_edges','t_edges'};
for k = 1:numel(required)
    if ~isfield(map, required{k})
        error('Summon:STPlot:InvalidMap', '时空地图缺少字段 %s。', required{k});
    end
end
if ~isstruct(options)
    error('Summon:STPlot:InvalidOptions', 'options 必须是结构体。');
end
[requested_t, requested_x, requested_y] = defaultSlices(map);
requested_t = optionValue(options, 'time_s', requested_t);
requested_x = optionValue(options, 'x_m', requested_x);
requested_y = optionValue(options, 'y_m', requested_y);
[slice_indices,inside] = st_resource_index(map,[requested_x,requested_y,requested_t]);
if ~inside
    error('Summon:STPlot:OutOfBounds','切片坐标越过资源块半开范围。');
end
ix = slice_indices(1); iy = slice_indices(2); it = slice_indices(3);
[colors, labels, owner_categories] = categoryPalette(map);
category_count = size(colors, 1);

max_points = 30000;
if isfield(map, 'config') && isfield(map.config, 'plotting') && ...
        isfield(map.config.plotting, 'max_3d_points')
    max_points = double(map.config.plotting.max_3d_points);
end
max_points = optionValue(options, 'max_3d_points', max_points);
if ~isscalar(max_points) || ~isfinite(max_points) || max_points < category_count ...
        || max_points ~= floor(max_points)
    error('Summon:STPlot:InvalidLimit', 'max_3d_points 必须是足以表示各类别的正整数。');
end
visible = 'off';
if isfield(options, 'visible') && logical(options.visible)
    visible = 'on';
end
fig = figure('Color','w', 'Visible',visible, 'Position',[60,60,1450,960], ...
    'Name','Day 5: X-Y-T occupancy resource blocks');
layout = tiledlayout(fig, 2, 2, 'TileSpacing','compact', 'Padding','compact');
title(layout, {'Day 5: X-Y-T resource blocks (independent trajectories)', ...
    'Static: conservative all-heading reference mask; dynamic: swept vehicle footprints'}, ...
    'FontSize',13, 'Interpreter','none');

% XY 切片：第三维固定为 it，转置后横轴为 x、纵轴为 y。
ax_xy = nexttile(layout, 1);
xy = categorize(map.owner_type(:,:,it), map.owner_id(:,:,it), owner_categories, category_count);
drawSlice(ax_xy, map.x_coords, map.y_coords, xy.', colors);
hold(ax_xy, 'on');
xline(ax_xy, map.x_coords(ix), 'k:', 'LineWidth',1.2, 'HandleVisibility','off');
yline(ax_xy, map.y_coords(iy), 'k:', 'LineWidth',1.2, 'HandleVisibility','off');
if isfield(map, 'trajectory_records')
    for k = 1:numel(map.trajectory_records)
        states = map.trajectory_records(k).states;
        if isempty(states), continue; end
        color = recordColor(map, map.trajectory_records(k).owner_id, colors, owner_categories);
        plot(ax_xy, states(:,1), states(:,2), '--', 'Color',0.65*color, ...
            'LineWidth',0.8, 'HandleVisibility','off');
        if map.t_coords(it) >= states(1,5) && map.t_coords(it) <= states(end,5)
            pose = interpolateXY(states, map.t_coords(it));
            plot(ax_xy, pose(1), pose(2), 'o', 'MarkerSize',5, ...
                'MarkerFaceColor',color, 'MarkerEdgeColor','k', 'HandleVisibility','off');
        end
    end
end
hold(ax_xy, 'off');
axis(ax_xy, 'equal');
xlim(ax_xy, [map.x_edges(1), map.x_edges(end)]);
ylim(ax_xy, [map.y_edges(1), map.y_edges(end)]);
xlabel(ax_xy, 'x / m'); ylabel(ax_xy, 'y / m');
title(ax_xy, sprintf('XY: t block [%.2f, %.2f) s; center %.2f s', ...
    map.t_edges(it), map.t_edges(it+1), map.t_coords(it)), 'Interpreter','none');

% XT 切片：第二维固定为 iy，绝不沿 y 方向求 any/max 投影。
ax_xt = nexttile(layout, 2);
xt_type = reshape(map.owner_type(:,iy,:), numel(map.x_coords), numel(map.t_coords));
xt_id = reshape(map.owner_id(:,iy,:), numel(map.x_coords), numel(map.t_coords));
xt = categorize(xt_type, xt_id, owner_categories, category_count);
drawSlice(ax_xt, map.x_coords, map.t_coords, xt.', colors);
hold(ax_xt, 'on');
yline(ax_xt, map.t_coords(it), 'k:', 'LineWidth',1.2, 'HandleVisibility','off');
hold(ax_xt, 'off');
xlim(ax_xt, [map.x_edges(1), map.x_edges(end)]);
ylim(ax_xt, [map.t_edges(1), map.t_edges(end)]);
xlabel(ax_xt, 'x / m'); ylabel(ax_xt, 't / s');
title(ax_xt, sprintf('XT: y block [%.2f, %.2f) m; center %.2f m', ...
    map.y_edges(iy), map.y_edges(iy+1), map.y_coords(iy)), 'Interpreter','none');

% YT 切片：第一维固定为 ix；数组重排保持 y、t 两维对应。
ax_yt = nexttile(layout, 3);
yt_type = reshape(map.owner_type(ix,:,:), numel(map.y_coords), numel(map.t_coords));
yt_id = reshape(map.owner_id(ix,:,:), numel(map.y_coords), numel(map.t_coords));
yt = categorize(yt_type, yt_id, owner_categories, category_count);
drawSlice(ax_yt, map.y_coords, map.t_coords, yt.', colors);
hold(ax_yt, 'on');
yline(ax_yt, map.t_coords(it), 'k:', 'LineWidth',1.2, 'HandleVisibility','off');
hold(ax_yt, 'off');
xlim(ax_yt, [map.y_edges(1), map.y_edges(end)]);
ylim(ax_yt, [map.t_edges(1), map.t_edges(end)]);
xlabel(ax_yt, 'y / m'); ylabel(ax_yt, 't / s');
title(ax_yt, sprintf('YT: x block [%.2f, %.2f) m; center %.2f m', ...
    map.x_edges(ix), map.x_edges(ix+1), map.x_coords(ix)), 'Interpreter','none');

% 三维散点先按类别分组再限量采样，避免永久静态占用淹没动态占用。
ax_3d = nexttile(layout, 4);
sample_count = drawVolume(ax_3d, map, colors, owner_categories, max_points);
xlabel(ax_3d, 'x / m'); ylabel(ax_3d, 'y / m'); zlabel(ax_3d, 't / s');
title(ax_3d, sprintf('X-Y-T schematic: %d sampled occupied blocks', sample_count), ...
    'Interpreter','none');
xlim(ax_3d, [map.x_edges(1), map.x_edges(end)]);
ylim(ax_3d, [map.y_edges(1), map.y_edges(end)]);
zlim(ax_3d, [map.t_edges(1), map.t_edges(end)]);
view(ax_3d, 42, 25); grid(ax_3d, 'on'); box(ax_3d, 'on');
colormap(ax_3d, colors); ax_3d.CLim = [0.5, category_count+0.5];

% 共享离散色标明确资源块所有者；重叠块的所有车辆 ID 保存在 map.overlaps 中。
cb = colorbar(ax_3d);
cb.Layout.Tile = 'east';
cb.Ticks = 1:category_count;
cb.TickLabels = labels;
cb.TickLabelInterpreter = 'none';
cb.Label.String = 'Resource-block category / owner';
cb.Label.Interpreter = 'none';
axes_list = [ax_xy, ax_xt, ax_yt, ax_3d];
set(axes_list, 'FontSize',10, 'Layer','top');
fig.UserData = struct('slice_indices',[ix,iy,it], ...
    'slice_centers',[map.x_coords(ix),map.y_coords(iy),map.t_coords(it)], ...
    'three_dimensional_points',sample_count);

if ~isempty(output_file)
    output_file = char(output_file);
    output_dir = fileparts(output_file);
    if ~isempty(output_dir) && ~exist(output_dir, 'dir'), mkdir(output_dir); end
    exportgraphics(fig, output_file, 'Resolution',160);
end
if nargout < 2, close(fig); end
end

function [time_s, x_m, y_m] = defaultSlices(map)
%DEFAULTSLICES 在已有轨迹共同时间范围选择车辆相距最近处，便于展示时空关系。
time_s = map.t_coords(max(1, ceil(numel(map.t_coords)/2)));
x_m = map.x_coords(max(1, ceil(numel(map.x_coords)/2)));
y_m = map.y_coords(max(1, ceil(numel(map.y_coords)/2)));
if ~isfield(map,'trajectory_records') || isempty(map.trajectory_records), return; end
records = map.trajectory_records;
records = records(arrayfun(@(r) ~isempty(r.states), records));
if isempty(records), return; end
begin = max(arrayfun(@(r) r.states(1,5), records));
finish = min(arrayfun(@(r) r.states(end,5), records));
begin = max(begin, map.t_edges(1));
finish = min(finish, map.t_edges(end) - eps(map.t_edges(end)));
if finish < begin, return; end
samples = linspace(begin,finish,120);
spread = zeros(size(samples));
centers = zeros(numel(samples),2);
for k = 1:numel(samples)
    positions = zeros(numel(records),2);
    for j = 1:numel(records)
        positions(j,:) = interpolateXY(records(j).states, samples(k));
    end
    centers(k,:) = mean(positions,1);
    spread(k) = sum(sum((positions-centers(k,:)).^2));
end
if numel(records) == 1
    chosen = ceil(numel(samples)/2);
else
    [~,chosen] = min(spread);
end
time_s = samples(chosen);
% 轨迹参考点可能不在显示范围内；默认切片夹至最邻近合法块中心。
x_m = min(max(centers(chosen,1),map.x_coords(1)),map.x_coords(end));
y_m = min(max(centers(chosen,2),map.y_coords(1)),map.y_coords(end));
end

function value = optionValue(options, name, fallback)
%OPTIONVALUE 读取显式绘图选项，默认值仅影响显示，不改变地图或规划配置。
value = fallback;
if isfield(options,name), value = double(options.(name)); end
end

function [colors, labels, owner_categories] = categoryPalette(map)
%CATEGORYPALETTE 为每个动态所有者指定独立颜色，静态/边界/多所有者用固定类别。
dynamic = find([map.owners.type] == uint8(3));
colors = [1,1,1; 0.18,0.18,0.18; 0.57,0.61,0.66; ...
    lines(numel(dynamic)); 0.88,0.12,0.30];
labels = [{'free','boundary (conservative inflation)','static obstacle (inflated)'}, ...
    {map.owners(dynamic).id}, {'multiple owners'}];
owner_categories = zeros(numel(map.owners),1);
for k = 1:numel(dynamic), owner_categories(dynamic(k)) = k+3; end
end

function categories = categorize(types, owner_ids, owner_categories, category_count)
%CATEGORIZE owner_type 只表达大类；动态类进一步按注册表 owner_id 细分颜色。
categories = ones(size(types));
categories(types == uint8(1)) = 2;
categories(types == uint8(2)) = 3;
dynamic = types == uint8(3);
if any(dynamic(:))
    categories(dynamic) = owner_categories(double(owner_ids(dynamic)));
end
categories(types == uint8(4)) = category_count;
end

function drawSlice(ax, horizontal, vertical, categories, colors)
%DRAWSLICE 绘制一个真正的二维切片：数据已按纵轴行、横轴列重排。
imagesc(ax, horizontal, vertical, categories);
set(ax, 'YDir','normal');
colormap(ax, colors); ax.CLim = [0.5,size(colors,1)+0.5];
box(ax, 'on');
end

function count = drawVolume(ax, map, colors, owner_categories, max_points)
%DRAWVOLUME 从已占用线性索引抽样，不创建 nx×ny×nt 的三维坐标网格。
occupied = find(map.occupancy);
categories = categorize(map.owner_type(occupied), map.owner_id(occupied), ...
    owner_categories, size(colors,1));
active = unique(categories);
per_category = floor(max_points/max(1,numel(active)));
count = 0;
hold(ax,'on');
for k = 1:numel(active)
    category = active(k);
    candidates = occupied(categories == category);
    chosen = unique(round(linspace(1,numel(candidates),min(per_category,numel(candidates)))));
    indices = candidates(chosen);
    [ix,iy,it] = ind2sub(size(map.occupancy),indices);
    alpha = 0.09;
    point_size = 5;
    if category > 3, alpha = 0.7; point_size = 13; end
    scatter3(ax, map.x_coords(ix), map.y_coords(iy), map.t_coords(it), ...
        point_size, colors(category,:), 'filled', 'MarkerFaceAlpha',alpha, ...
        'MarkerEdgeAlpha',alpha, 'HandleVisibility','off');
    count = count+numel(indices);
end
if isfield(map,'trajectory_records')
    for k = 1:numel(map.trajectory_records)
        record = map.trajectory_records(k);
        if isempty(record.states), continue; end
        color = recordColor(map,record.owner_id,colors,owner_categories);
        plot3(ax,record.states(:,1),record.states(:,2),record.states(:,5), ...
            '-', 'Color',0.6*color, 'LineWidth',1.7, 'HandleVisibility','off');
    end
end
hold(ax,'off');
end

function color = recordColor(map, external_id, colors, owner_categories)
%RECORDCOLOR 将可读车辆 ID 匹配到注册表，从而保证三维路径与占用块颜色一致。
index = find(strcmp({map.owners.id},char(external_id)),1,'first');
if isempty(index) || owner_categories(index) == 0
    color = [0.2,0.3,0.7];
else
    color = colors(owner_categories(index),:);
end
end

function position = interpolateXY(states, time_s)
%INTERPOLATEXY 图中参考点按时间插值；只有图示使用此函数，资源块标记另有扫掠逻辑。
if size(states,1) == 1
    position = states(1,1:2);
else
    position = interp1(states(:,5),states(:,1:2),time_s,'linear');
end
end
