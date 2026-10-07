function [indices, outside] = day6_resource_blocks(states, map, footprint, stop_on_hit)
%DAY6_RESOURCE_BLOCKS 栅格化候选车身的层内扫掠，查询动态资源前使用。
% 输入：密集Nx5候选轨迹、Day5图、膨胀车身。输出：三维线性索引与越界标志。
% 采用与Day5相同的半开时间切段、整块矩形SAT和采样间隙补偿。
% 搜索可在首个动态块命中后退出；最终验收仍收集全部块，不改变拒绝判据。
if nargin < 4, stop_on_hit = false; end
[~,inside] = st_resource_index(map,states(:,[1,2,5]));
outside = any(~inside);
indices = zeros(0,1);
if outside, return; end
bins = cell(map.nt,1);
for k = 1:size(states,1)
    [index,~] = st_resource_index(map,states(k,[1,2,5]));
    cells = st_vehicle_cells(map,states(k,1:3),footprint,0);
    if stop_on_hit
        global_indices = cells+(index(3)-1)*map.nx*map.ny;
        indices = global_indices(map.dynamic_occupancy(global_indices));
        if ~isempty(indices), return; end
    end
    bins{index(3)} = [bins{index(3)};cells];
end
for k = 1:size(states,1)-1
    a = states(k,:); b = states(k+1,:);
    cuts = [a(5),map.t_edges(map.t_edges > a(5) & map.t_edges < b(5)),b(5)];
    for j = 1:numel(cuts)-1
        first = a+(b-a)*(cuts(j)-a(5))/(b(5)-a(5));
        last = a+(b-a)*(cuts(j+1)-a(5))/(b(5)-a(5));
        duration = last(5)-first(5);
        % 对带运动学控制的弧段，弧长不小于首末点直线距离。
        distance = max(hypot(last(1)-first(1),last(2)-first(2)), ...
            0.5*(first(4)+last(4))*duration);
        bound = distance+footprint.radius*abs(last(3)-first(3));
        count = max([1,ceil(duration/map.config.sweep_max_dt_s), ...
            ceil(bound/map.config.sweep_max_distance_m)]);
        padding = bound/(2*count);
        [index,~] = st_resource_index(map,[first(1:2),(first(5)+last(5))/2]);
        hits = cell(count+1,1);
        for n = 0:count
            state = first+(last-first)*n/count;
            hits{n+1} = st_vehicle_cells(map,state(1:3),footprint,padding);
            if stop_on_hit
                global_indices = hits{n+1}+(index(3)-1)*map.nx*map.ny;
                indices = global_indices(map.dynamic_occupancy(global_indices));
                if ~isempty(indices), return; end
            end
        end
        bins{index(3)} = [bins{index(3)};vertcat(hits{:})];
    end
end
for k = 1:map.nt, bins{k} = unique(bins{k})+(k-1)*map.nx*map.ny; end
indices = vertcat(bins{:});
end
