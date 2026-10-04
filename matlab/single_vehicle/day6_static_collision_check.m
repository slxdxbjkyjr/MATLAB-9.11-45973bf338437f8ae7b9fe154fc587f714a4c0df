function [is_collision, detail] = day6_static_collision_check(pose,map,vehicle_config)
%DAY6_STATIC_COLLISION_CHECK 完整车辆矩形与原始静态多边形的适配检测。
% 输入：以后轴中心为参考的[x,y,theta]，summon_map几何和车辆尺寸/安全裕度。
% 输出：碰撞标志；detail.type/obstacle_index/corners_xy描述原因与车辆外包络。
% 逻辑：只对车辆矩形加一次配置裕度，不重复使用Day5全航向膨胀地图。
% 障碍物检测包含角点落入、障碍物被车身包住和边相交。凹道路边界除
% 检查角点外，还将车身边按所有边界交点分段，检查每个开段是否在道路内。
if ~isnumeric(pose) || numel(pose) ~= 3 || ~isreal(pose) || any(~isfinite(pose))
    error('Day6:StaticCollision:InvalidPose','位姿必须为有限实数[x,y,theta]。');
end
pose = double(pose(:).');
footprint = inflate_vehicle_occupancy(vehicle_config,0);
c = cos(pose(3)); s = sin(pose(3));
corners = footprint.corners_local*[c,s;-s,c]+pose(1:2);
boundary = polygonVertices(map.boundary_xy);
detail = struct('type','none','obstacle_index',0,'corners_xy',corners);
is_collision = ~all(inpolygon(corners(:,1),corners(:,2),boundary(:,1),boundary(:,2)));
if ~is_collision && ~isConvex(boundary)
    % 凸边界中四角在内即保证矩形在内；凹边界必须检查车身边穿界。
    is_collision = boundaryCrossed(corners,boundary);
end
if is_collision
    detail.type = 'road_boundary';
    return;
end
for k = 1:numel(map.obstacles)
    obstacle = polygonVertices(map.obstacles{k});
    % 包围盒仅用于快速排除明显分离，多边形判定仍是最后判据。
    if max(corners(:,1)) < min(obstacle(:,1)) || max(obstacle(:,1)) < min(corners(:,1)) || ...
            max(corners(:,2)) < min(obstacle(:,2)) || max(obstacle(:,2)) < min(corners(:,2))
        continue;
    end
    overlap = any(inpolygon(corners(:,1),corners(:,2),obstacle(:,1),obstacle(:,2))) || ...
        any(inpolygon(obstacle(:,1),obstacle(:,2),corners(:,1),corners(:,2)));
    if ~overlap
        for i = 1:4
            a = corners(i,:); b = corners(mod(i,4)+1,:);
            for j = 1:size(obstacle,1)
                p = obstacle(j,:); q = obstacle(mod(j,size(obstacle,1))+1,:);
                if ~isempty(segmentParameters(a,b,p,q))
                    overlap = true; break;
                end
            end
            if overlap, break; end
        end
    end
    if overlap
        is_collision = true; detail.type = 'static_obstacle'; detail.obstacle_index = k;
        return;
    end
end
end

function vertices = polygonVertices(vertices)
%POLYGONVERTICES 统一开环顶点数组；边循环自行补上末顶点到首顶点的边。
vertices = double(vertices);
if size(vertices,2) ~= 2 || size(vertices,1) < 3 || any(~isfinite(vertices(:)))
    error('Day6:StaticCollision:InvalidPolygon','静态几何须为有限Nx2多边形。');
end
if isequal(vertices(1,:),vertices(end,:)), vertices(end,:) = []; end
end

function flag = isConvex(vertices)
%ISCONVEX 以相邻边有向叉积识别凸边界，避免常规矩形道路的逐边求交开销。
n = size(vertices,1); turns = zeros(n,1);
for k = 1:n
    a = vertices(mod(k,n)+1,:)-vertices(k,:);
    b = vertices(mod(k+1,n)+1,:)-vertices(mod(k,n)+1,:);
    turns(k) = cross2(a,b);
end
turns = turns(abs(turns)>1e-12);
flag = isempty(turns) || all(turns>0) || all(turns<0);
end

function crossed = boundaryCrossed(corners,boundary)
%BOUNDARYCROSSED 按交点切分车身边，任何边段位于凹边界外即为越界。
crossed = false;
for i = 1:4
    a = corners(i,:); b = corners(mod(i,4)+1,:); parameters = [0,1];
    for j = 1:size(boundary,1)
        parameters = [parameters,segmentParameters(a,b,boundary(j,:), ...
            boundary(mod(j,size(boundary,1))+1,:))]; %#ok<AGROW>
    end
    parameters = unique(min(1,max(0,parameters)));
    mids = (parameters(1:end-1)+parameters(2:end))/2;
    probes = a+mids(:)*(b-a);
    if any(~inpolygon(probes(:,1),probes(:,2),boundary(:,1),boundary(:,2)))
        crossed = true; return;
    end
end
end

function parameters = segmentParameters(a,b,p,q)
%SEGMENTPARAMETERS 求线段ab与pq交点在ab上的参数，保留接触和共线重叠。
r = b-a; u = q-p; offset = p-a; denominator = cross2(r,u);
tol = 1e-12*max(1,norm(r)*norm(u)); parameters = [];
if abs(denominator)>tol
    t = cross2(offset,u)/denominator; v = cross2(offset,r)/denominator;
    if t>=-1e-12 && t<=1+1e-12 && v>=-1e-12 && v<=1+1e-12
        parameters = min(1,max(0,t));
    end
elseif abs(cross2(offset,r))<=tol
    length2 = dot(r,r);
    if length2<=eps, return; end
    interval = sort([dot(p-a,r),dot(q-a,r)]/length2);
    low = max(0,interval(1)); high = min(1,interval(2));
    if low<=high+1e-12, parameters = [low,high]; end
end
end

function value = cross2(a,b)
%CROSS2 二维向量叉积标量，用于方向判断和线段交点求解。
value = a(1)*b(2)-a(2)*b(1);
end
