function map = prepare_dynamic_context(map, obstacles)
%PREPARE_DYNAMIC_CONTEXT 缓存地图中的动态占用掩码，避免重复静态膨胀。
% 输入：Day5资源图、高优先级障碍列表。输出：具有dynamic_occupancy的资源图。
% 主归属为静态但重叠列表含车辆的块，也加入动态掩码。
if isempty(map), return; end
map.dynamic_occupancy = map.owner_type == 3;
for k = 1:numel(map.overlaps)
    if any([map.owners(map.overlaps(k).owner_ids).type] == 3)
        map.dynamic_occupancy(map.overlaps(k).linear_index) = true;
    end
end
for k = 1:numel(obstacles)
    if ~any(strcmp({map.owners.id},obstacles(k).id))
        error('DynamicCollision:InvalidContext','高优先级轨迹%s尚未写入资源块。',obstacles(k).id);
    end
end
% 各时间层缓存动态块的包络，只能用于安全分离预筛选；命中后仍查矩形块。
map.dynamic_layer_bounds = nan(map.nt,4);
for k = 1:map.nt
    [ix,iy] = find(map.dynamic_occupancy(:,:,k));
    if ~isempty(ix)
        map.dynamic_layer_bounds(k,:) = [min(map.x_coords(ix))-map.dx/2, ...
            max(map.x_coords(ix))+map.dx/2,min(map.y_coords(iy))-map.dy/2, ...
            max(map.y_coords(iy))+map.dy/2];
    end
end
end
