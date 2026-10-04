function [success, connection_nodes, statistics] = vhybrid_goal_connection( ...
    parent_node, goal_state, map, vehicle_config, planner_config)
%VHYBRID_GOAL_CONNECTION 用真实 Dubins 控制段和既有运动学连接终点。
% 输入：父节点 [x,y,theta,v,t]、目标状态、静态地图和车辆/规划配置。
% 输出：success、父节点之后的连接节点、候选/碰撞/约束裁剪统计。
% 逻辑：枚举六类圆弧/直线组合，按最小转弯半径限制曲率；在弧长上
% 构造加速/匀速/减速剖面。每个点都由 summon_vehicle_dynamic 积分，
% 在几何段边界与速度相位边界切步。禁止直接赋值目标位置或航向。

cfg = planner_config.vhybrid;
dyn = planner_config.dynamics;
success = false;
connection_nodes = repmat(vhybrid_node(), 0, 1);
statistics = struct('candidates', 0, 'collision_pruned', 0, ...
    'constraint_pruned', 0, 'terminal_speed', NaN, 'connection_length_m', NaN);
start = [parent_node.x,parent_node.y,parent_node.theta,parent_node.v,parent_node.t];
goal_state = double(goal_state(:).');
if hypot(start(1)-goal_state(1),start(2)-goal_state(2)) > cfg.goal_connection_distance_m
    return;
end
delta_max = min(vehicle_config.steering.maximum_steer_rad, dyn.delta_max_rad);
radius = vehicle_config.dimensions_m.wheelbase / tan(delta_max);
curves = dubinsCandidates(start(1:3), goal_state(1:3), radius);
% 采样的是连接段巡航速度，终端速度始终为目标停车约束。
speeds = unique([cfg.goal_connection_speed_samples_mps(:).', ...
    cfg.reference_speed_mps, start(4), cfg.terminal_speed_mps]);
speeds = sort(speeds, 'descend');
for curve_id = 1:numel(curves)
    curve = curves(curve_id);
    for cruise_speed = speeds
        statistics.candidates = statistics.candidates + 1;
        [phases, feasible] = speedPhases(sum(curve.lengths),start(4), ...
            cfg.terminal_speed_mps,cruise_speed,dyn);
        if ~feasible
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            continue;
        end
        [trial, reason] = integrateCurve(parent_node, goal_state, curve, phases, ...
            delta_max, map, vehicle_config, planner_config);
        if strcmp(reason,'collision')
            statistics.collision_pruned = statistics.collision_pruned + 1;
            % 静态碰撞仅依赖几何，对同一曲线换速度无法绕开障碍物。
            break;
        elseif ~strcmp(reason,'ok')
            statistics.constraint_pruned = statistics.constraint_pruned + 1;
            continue;
        end
        connection_nodes = trial;
        statistics.terminal_speed = trial(end).v;
        statistics.connection_length_m = sum(curve.lengths);
        success = true;
        return;
    end
end
end

function curves = dubinsCandidates(start, goal, radius)
%DUBINSCANDIDATES 在单位半径坐标下求 LSL/RSR/LSR/RSL/RLR/LRL。
% L/R 的长度是非负转角，S 是直线长度；乘 radius 后全部为弧长 m。
d = hypot(goal(1)-start(1),goal(2)-start(2))/radius;
bearing = atan2(goal(2)-start(2),goal(1)-start(1));
alpha = mod(start(3)-bearing,2*pi); beta = mod(goal(3)-bearing,2*pi);
sa = sin(alpha); sb = sin(beta); ca = cos(alpha); cb = cos(beta);
cab = cos(alpha-beta);
lengths = nan(6,3);
types = [1,0,1; -1,0,-1; 1,0,-1; -1,0,1; -1,1,-1; 1,-1,1];
p2 = 2+d*d-2*cab+2*d*(sa-sb);
if p2 >= -1e-12
    tmp = atan2(cb-ca,d+sa-sb);
    lengths(1,:) = [mod(-alpha+tmp,2*pi),sqrt(max(0,p2)),mod(beta-tmp,2*pi)];
end
p2 = 2+d*d-2*cab+2*d*(sb-sa);
if p2 >= -1e-12
    tmp = atan2(ca-cb,d-sa+sb);
    lengths(2,:) = [mod(alpha-tmp,2*pi),sqrt(max(0,p2)),mod(-beta+tmp,2*pi)];
end
p2 = -2+d*d+2*cab+2*d*(sa+sb);
if p2 >= -1e-12
    p = sqrt(max(0,p2));
    tmp = atan2(-ca-cb,d+sa+sb)-atan2(-2,p);
    lengths(3,:) = [mod(-alpha+tmp,2*pi),p,mod(-beta+tmp,2*pi)];
end
p2 = d*d-2+2*cab-2*d*(sa+sb);
if p2 >= -1e-12
    p = sqrt(max(0,p2));
    tmp = atan2(ca+cb,d-sa-sb)-atan2(2,p);
    lengths(4,:) = [mod(alpha-tmp,2*pi),p,mod(beta-tmp,2*pi)];
end
tmp = (6-d*d+2*cab+2*d*(sa-sb))/8;
if abs(tmp) <= 1+1e-12
    p = mod(2*pi-acos(max(-1,min(1,tmp))),2*pi);
    t = mod(alpha-atan2(ca-cb,d-sa+sb)+p/2,2*pi);
    lengths(5,:) = [t,p,mod(alpha-beta-t+p,2*pi)];
end
tmp = (6-d*d+2*cab+2*d*(-sa+sb))/8;
if abs(tmp) <= 1+1e-12
    p = mod(2*pi-acos(max(-1,min(1,tmp))),2*pi);
    t = mod(-alpha-atan2(ca-cb,d+sa-sb)+p/2,2*pi);
    lengths(6,:) = [t,p,mod(beta-alpha-t+p,2*pi)];
end
valid_ids = find(all(isfinite(lengths),2));
[~,order] = sort(sum(lengths(valid_ids,:),2));
curves = repmat(struct('lengths',zeros(1,3),'types',zeros(1,3)),numel(order),1);
for k = 1:numel(order)
    id = valid_ids(order(k));
    curves(k).lengths = radius*lengths(id,:);
    curves(k).types = types(id,:);
end
end

function [phases, feasible] = speedPhases(distance, v0, vf, cap, dyn)
%SPEEDPHASES 三角形/梯形速度剖面，用 v²-v0²=2*a*s 保证路程一致。
% 相位字段：[持续时间, 加速度, 相位末速度]，终端 vf 是硬约束。
phases = zeros(0,3); feasible = false;
ap = dyn.a_max_mps2; ab = -dyn.a_min_mps2;
if distance <= 1e-10 || ap <= 0 || ab <= 0 || cap <= 0 || ...
        cap < max(v0,vf) || cap > dyn.v_max_mps || min(v0,vf) < dyn.v_min_mps
    return;
end
peak = min(cap,sqrt((2*ap*ab*distance+ab*v0^2+ap*vf^2)/(ap+ab)));
if peak < max(v0,vf)-1e-10
    return; % 剩余距离不足以在约束内停车；不能人为截断速度。
end
peak = max(peak,max(v0,vf));
ta = (peak-v0)/ap; tb = (peak-vf)/ab;
cruise_distance = distance-(peak^2-v0^2)/(2*ap)-(peak^2-vf^2)/(2*ab);
tc = max(0,cruise_distance)/peak;
phases = [ta,ap,peak; tc,0,peak; tb,-ab,vf];
phases = phases(phases(:,1)>1e-12,:);
feasible = ~isempty(phases);
end

function [nodes, reason] = integrateCurve(parent, goal, curve, phases, ...
    delta_max, map, vehicle, planner)
%INTEGRATECURVE 以相位/圆弧边界为事件切步，积分同一个自行车模型。
cfg = planner.vhybrid; dyn = planner.dynamics;
nodes = repmat(vhybrid_node(),0,1); reason = 'constraint';
state = [parent.x,parent.y,parent.theta,parent.v,parent.t];
previous = parent; segment = 1; phase = 1;
segment_remaining = curve.lengths(1); time_remaining = phases(1,1);
while segment <= 3 && phase <= size(phases,1)
    if segment_remaining <= 1e-10 && segment < 3
        segment = segment+1;
        if segment <= 3, segment_remaining = curve.lengths(segment); end
        continue;
    end
    if time_remaining <= 0
        phase = phase+1;
        if phase <= size(phases,1), time_remaining = phases(phase,1); end
        continue;
    end
    % 根据剩余相位时间修正舍入误差；不修改状态，不超过加速度边界。
    acceleration = (phases(phase,3)-state(4))/time_remaining;
    acceleration = max(dyn.a_min_mps2,min(dyn.a_max_mps2,acceleration));
    steering = curve.types(segment)*delta_max;
    dt = min(cfg.goal_connection_step_s,time_remaining);
    ds_limit = min(segment_remaining,cfg.collision_check_step_m);
    if segment == 3 && segment_remaining <= 1e-10
        % 最末段不能因弧长舍入提前结束制动，仍积分完整相位直到 v=0。
        ds_limit = cfg.collision_check_step_m;
    end
    possible_distance = state(4)*dt+0.5*acceleration*dt^2;
    if possible_distance > ds_limit
        % 稳定解法，避免减速时求根的数值抵消。
        end_speed = sqrt(max(0,state(4)^2+2*acceleration*ds_limit));
        dt = min(dt,2*ds_limit/(state(4)+end_speed));
    end
    [next,ds,~,valid] = summon_vehicle_dynamic(state,steering,acceleration,dt, ...
        vehicle.dimensions_m.wheelbase,dyn.v_max_mps,dyn.v_min_mps, ...
        dyn.a_min_mps2,dyn.a_max_mps2,delta_max);
    if ~valid || ds <= 0
        return;
    end
    if summon_collision_check(next(1:3),map,vehicle)
        reason = 'collision'; return;
    end
    [g,h] = vhybrid_cost(previous,next,goal,planner,ds,steering);
    indices = [floor((next(1)-map.bounds(1))/cfg.xy_grid_resolution_m), ...
        floor((next(2)-map.bounds(2))/cfg.xy_grid_resolution_m), ...
        round((mod(next(3)+pi,2*pi))/cfg.yaw_grid_resolution_rad), ...
        round(next(4)/cfg.velocity_grid_resolution_mps),round(next(5)/cfg.time_grid_resolution_s)];
    node = vhybrid_node(next,acceleration,steering,previous.node_id,g,h,indices,0);
    node.direction = 1; node.travelled_distance = ds;
    nodes(end+1,1) = node; %#ok<AGROW>
    previous = node; state = next;
    segment_remaining = segment_remaining-ds; time_remaining = time_remaining-dt;
end
% 验收模型积分的真实末状态，绝不将节点替换成目标。
if ~isempty(nodes) && hypot(state(1)-goal(1),state(2)-goal(2)) <= cfg.goal_position_tolerance_m && ...
        abs(mod(state(3)-goal(3)+pi,2*pi)-pi) <= cfg.goal_heading_tolerance_rad && ...
        abs(state(4)-cfg.terminal_speed_mps) <= cfg.goal_speed_tolerance_mps
    reason = 'ok';
end
end
