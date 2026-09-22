function validate_day2_heading()
%VALIDATE_DAY2_HEADING 对比同一起终点修复前后结果并校核全路径。
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab','single_vehicle'));
out = fullfile(root,'outputs','heading_validation');
before = load(fullfile(out,'before.mat')); after = load(fullfile(out,'after.mat'));
vehicle = jsondecode(fileread(fullfile(root,'config','vehicle_config.json')));
map = summon_map(jsondecode(fileread(fullfile(root,'config','map_config.json'))));
goal = [15,9,3.1415926536];
p = after.p;
assert(p.valid,'规划失败');
assert(hypot(p.x(end)-goal(1),p.y(end)-goal(2)) < 0.05);
assert(abs(atan2(sin(p.theta(end)-goal(3)),cos(p.theta(end)-goal(3)))) < pi/180);
assert(norm([p.x(1),p.y(1)]-[27,17]) < 1e-9);
ds = hypot(diff(p.x),diff(p.y));
dtheta = atan2(sin(diff(p.theta)),cos(diff(p.theta)));
limit = tan(vehicle.steering.maximum_steer_rad)/vehicle.dimensions_m.wheelbase;
assert(all(ds>0 & ds<0.051),'路径存在重复点或位置跳变');
assert(all(abs(dtheta)./ds < limit*1.002),'路径曲率超限');
% 检查切线与车身航向的一致性（倒车允许相差 pi）。
tangent = atan2(diff(p.y),diff(p.x));
midheading = p.theta(1:end-1)+dtheta/2;
expected = midheading + pi*(p.direction(2:end)<0);
assert(max(abs(atan2(sin(tangent-expected),cos(tangent-expected)))) < 0.005);
for k=1:numel(p.x)
    assert(~summon_collision_check([p.x(k),p.y(k),p.theta(k)],map,vehicle));
end
fig = figure('Visible','off','Color','w','Position',[100,100,1200,700]);
paths = {before.p,p}; names = {'Before','After'};
metrics = zeros(2,7);
for k=1:2
    q=paths{k};
    epos=hypot(q.x(end)-goal(1),q.y(end)-goal(2));
    eyaw=abs(atan2(sin(q.theta(end)-goal(3)),cos(q.theta(end)-goal(3))))*180/pi;
    metrics(k,:)=[epos,eyaw,q.search_statistics.expanded_nodes, ...
        q.search_statistics.generated_nodes,q.search_statistics.collision_pruned_nodes, ...
        q.search_statistics.elapsed_sec,sum(hypot(diff(q.x),diff(q.y)))];
    subplot(2,2,k);
    plot(map.boundary_xy(:,1),map.boundary_xy(:,2),'k-'); hold on;
    plot(q.x,q.y,'b-','LineWidth',2);
    plot(goal(1),goal(2),'kp','MarkerSize',12);
    quiver(goal(1),goal(2),2*cos(goal(3)),2*sin(goal(3)),0,'k','LineWidth',2);
    quiver(q.x(end),q.y(end),2*cos(q.theta(end)),2*sin(q.theta(end)),0,'r','LineWidth',2);
    axis equal; xlim([0,30]); ylim([0,20]); grid on;
    title(sprintf('%s: %.6f m, %.6f deg',names{k},epos,eyaw));
    xlabel('x / m'); ylabel('y / m');
    legend('boundary','path','requested goal','requested heading','actual heading','Location','southwest');
    subplot(2,2,k+2);
    s=[0;cumsum(hypot(diff(q.x),diff(q.y)))];
    plot(s,unwrap(q.theta)*180/pi,'b-','LineWidth',1.5); hold on;
    yline(180,'k--'); grid on;
    xlabel('path length / m'); ylabel('body yaw / deg'); title(names{k});
end
exportgraphics(fig,fullfile(out,'comparison.png'),'Resolution',160); close(fig);
T=array2table(metrics,'VariableNames',{'position_error_m','heading_error_deg', ...
    'expanded_nodes','generated_nodes','collision_pruned_nodes','elapsed_sec','path_length_m'}, ...
    'RowNames',names);
writetable(T,fullfile(out,'comparison.csv'),'WriteRowNames',true); disp(T);
fprintf('PASS: start/end, heading, continuity, curvature, direction and %d collision samples.\n',numel(p.x));
end
