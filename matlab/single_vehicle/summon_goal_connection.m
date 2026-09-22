function [poses, curvature] = summon_goal_connection(start, goal, map, vehicle, search)
%SUMMON_GOAL_CONNECTION 三次 Hermite 曲线满足两端位置和车身航向。
% 仅接受前进、无奇点、曲率受限且逐点车身碰撞检查通过的连接。
% 不把未到达的搜索节点强行替换成目标姿态。
poses = []; curvature = [];
d = norm(goal(1:2)-start(1:2));
if d < 1e-8 || d > search.goal_connection_distance_m, return; end
limit = tan(vehicle.steering.maximum_steer_rad)/vehicle.dimensions_m.wheelbase;
for scale = [1, 1.5, 2, 0.75]
    m0 = scale*d*[cos(start(3)), sin(start(3))];
    m1 = scale*d*[cos(goal(3)), sin(goal(3))];
    a = 2*start(1:2)-2*goal(1:2)+m0+m1;
    b = -3*start(1:2)+3*goal(1:2)-2*m0-m1;
    c = m0;
    % 导数范数的上界保证相邻采样点弧长不超过 0.05 m。
    n = max(100,ceil((3*norm(a)+2*norm(b)+norm(c))/0.05));
    u = linspace(0,1,n+1).';
    xy = u.^3*a+u.^2*b+u*c+start(1:2);
    v = 3*u.^2*a+2*u*b+c;
    acc = 6*u*a+2*b;
    speed = hypot(v(:,1),v(:,2));
    kappa = (v(:,1).*acc(:,2)-v(:,2).*acc(:,1))./speed.^3;
    if any(speed<1e-6) || any(abs(kappa)>0.995*limit), continue; end
    trial = [xy, atan2(v(:,2),v(:,1))];
    valid = true;
    for j = 1:size(trial,1)
        if summon_collision_check(trial(j,:),map,vehicle)
            valid = false; break;
        end
    end
    if valid, poses = trial; curvature = kappa; return; end
end
end
