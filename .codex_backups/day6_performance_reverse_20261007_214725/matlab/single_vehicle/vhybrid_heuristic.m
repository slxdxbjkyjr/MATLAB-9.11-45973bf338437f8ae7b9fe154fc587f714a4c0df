function [h,distance,method] = vhybrid_heuristic(state,goal,cfg)
%VHYBRID_HEURISTIC 起点和扩展节点共用的目标启发代价。
% 输入：连续状态[x,y,theta,v,t]、目标状态和vhybrid集中配置。
% 输出：加权启发h、未加权几何距离distance和实际采用的方法method。
% Day4默认保留欧氏距离加航向项。Day6选择dubins时使用满足最小转弯
% 半径的无障碍前向最短Dubins弧长；忽略加速度和障碍，是空间运动学
% 距离下界，而不是预设路径。权重大于1的加权A*不保证全局最优。
method = 'euclidean';
if isfield(cfg,'heuristic_method'), method = char(cfg.heuristic_method); end
if strcmp(method,'dubins')
    if ~isfield(cfg,'minimum_turning_radius_m')
        error('VHybrid:MissingTurningRadius','Dubins启发需要由车辆模型计算的最小转弯半径。');
    end
    curves = vhybrid_dubins_candidates(state(1:3),goal(1:3),cfg.minimum_turning_radius_m);
    if isempty(curves)
        distance = hypot(state(1)-goal(1),state(2)-goal(2));
    else
        distance = sum(curves(1).lengths);
    end
elseif strcmp(method,'euclidean')
    distance = hypot(state(1)-goal(1),state(2)-goal(2)) + ...
        0.5*abs(mod(state(3)-goal(3)+pi,2*pi)-pi);
else
    error('VHybrid:InvalidHeuristicMethod','未知启发式方法%s。',method);
end
weight = 1;
if isfield(cfg,'heuristic_weight'), weight = cfg.heuristic_weight; end
if ~isnumeric(weight) || ~isscalar(weight) || ~isfinite(weight) || weight<=0
    error('VHybrid:InvalidHeuristicWeight','启发式权重必须为正有限标量。');
end
h = weight*cfg.weight_goal_distance*distance;
end
