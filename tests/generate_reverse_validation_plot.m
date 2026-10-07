function result = generate_reverse_validation_plot(write_output)
%GENERATE_REVERSE_VALIDATION_PLOT 用实际车型和真实搜索验证狭窄巷道倒车能力。
% 输入：write_output 可选逻辑值，默认true；false只搜索和验收、不输出图片。
% 输出：result含两次搜索结果、零速换挡等待、倒车距离、时间和图片绝对路径。
% 场景为狭窄直通道：初始前进挡，目标在车身后方且
% 目标车身航向不变。前进搜索与允许倒车搜索使用同一搜索器和车辆模型。
% 前进搜索在有限预算内未成功，不将节点上限退出解释为数学上的无解证明。
% 常规Day6双车可以都采用前进路线，不强制选倒车；倒车能力在此特殊场景验证。
% 配置仅在内存副本中调整，不修改JSON；图中每个轨迹点均由搜索器生成。
if nargin < 1, write_output = true; end
total_timer = tic;
root = fileparts(fileparts(mfilename('fullpath')));
original_path = path;
path_cleanup = onCleanup(@() path(original_path)); %#ok<NASGU>
addpath(fullfile(root,'matlab','single_vehicle'));
vehicle = jsondecode(fileread(fullfile(root,'config','vehicle_config.json')));
planner = jsondecode(fileread(fullfile(root,'config','planner_config.json')));
planner.vhybrid.reverse_enabled = true;
planner.vhybrid.initial_gear = 1;
planner.vhybrid.max_search_nodes = 200;
planner.vhybrid.max_search_time_s = 10;
planner.vhybrid.goal_connection_distance_m = 40;
planner.vhybrid.heuristic_method = 'euclidean';
planner.vhybrid.heuristic_weight = 1;
if ~isfield(planner.vhybrid,'reverse_penalty'), planner.vhybrid.reverse_penalty = 2; end
if ~isfield(planner.vhybrid,'gear_switch_penalty'), planner.vhybrid.gear_switch_penalty = 1; end
if ~isfield(planner.vhybrid,'max_reverse_speed_mps'), planner.vhybrid.max_reverse_speed_mps = 2; end
% 保留五档转角。通道空间限制决定可行性，不手工规定倒车轨迹或控制序列。
maximum_steer = min(vehicle.steering.maximum_steer_rad,planner.dynamics.delta_max_rad);
planner.vhybrid.control_steering_samples_rad = linspace(-maximum_steer,maximum_steer,5);
geometry = struct('boundary_xy',[0,0;20,0;20,2.6;0,2.6;0,0], ...
    'obstacles',{{}},'bounds',[0,0,20,2.6]);
start = [15,1.3,0,0,0]; goal = [5,1.3,0,0,0];
forward_planner = planner; forward_planner.vhybrid.reverse_enabled = false;
forward_timer = tic;
forward_path = summon_vhybrid_astar(start,goal,geometry,vehicle,forward_planner);
forward_seconds = toc(forward_timer);
reverse_timer = tic;
reverse_path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner);
reverse_seconds = toc(reverse_timer);
result = struct('forward_only_path',forward_path,'reverse_path',reverse_path, ...
    'forward_only_failed',~forward_path.valid,'reverse_planning_succeeded',reverse_path.valid, ...
    'start_state',start,'goal_state',goal,'geometry',geometry, ...
    'forward_search_seconds',forward_seconds,'reverse_search_seconds',reverse_seconds, ...
    'reverse_distance_m',NaN,'gear_switch_count',0,'gear_wait_time_s',0, ...
    'complete_zero_speed_gear_changes',false,'validation_passed',false, ...
    'plot_path',fullfile(root,'outputs','day6_reverse_corridor.png'),'elapsed_sec',0);
if ~reverse_path.valid
    result.elapsed_sec = toc(total_timer);
    fprintf('倒车验证失败：error_code=%d；前进 %.3fs，倒车 %.3fs，总计 %.3fs。\n', ...
        reverse_path.error_code,forward_seconds,reverse_seconds,result.elapsed_sec);
    return;
end
shift_indices = find(reverse_path.is_gear_change);
gear_flips = find(diff(reverse_path.gear)~=0)+1;
waits = reverse_path.t(shift_indices)-reverse_path.t(shift_indices-1);
shift_motion = hypot(reverse_path.x(shift_indices)-reverse_path.x(shift_indices-1), ...
    reverse_path.y(shift_indices)-reverse_path.y(shift_indices-1));
shift_angles = abs(mod(reverse_path.theta(shift_indices)-reverse_path.theta(shift_indices-1)+pi,2*pi)-pi);
result.gear_switch_count = numel(shift_indices);
result.gear_wait_time_s = sum(waits);
result.reverse_distance_m = sum(abs(reverse_path.travelled_distance(reverse_path.direction<0)));
result.complete_zero_speed_gear_changes = isequal(shift_indices,gear_flips) && ...
    all(waits>=planner.dynamics.direction_change_time_s-1e-10) && ...
    all(reverse_path.v(shift_indices)==0 & reverse_path.v(shift_indices-1)==0) && ...
    all(shift_motion<=1e-8 & shift_angles<=1e-8);
result.validation_passed = result.forward_only_failed && reverse_path.kinematic_validation.valid && ...
    result.complete_zero_speed_gear_changes && result.gear_switch_count>=1 && ...
    result.reverse_distance_m>0 && all(reverse_path.v>=0) && ...
    reverse_path.final_position_error_m<=planner.vhybrid.goal_position_tolerance_m && ...
    reverse_path.final_heading_error_rad<=planner.vhybrid.goal_heading_tolerance_rad && ...
    abs(reverse_path.v(end))<=planner.vhybrid.goal_speed_tolerance_mps;
if write_output
    output_dir = fullfile(root,'outputs');
    if ~isfolder(output_dir), mkdir(output_dir); end
    renderReverseResult(reverse_path,geometry,vehicle,waits,shift_indices,result);
end
result.elapsed_sec = toc(total_timer);
fprintf(['倒车能力验证：通过=%d；前进搜索未成功=%d（错误码%d，有限搜索预算）；' ...
    '前进搜索 %.3fs，倒车搜索 %.3fs，总计 %.3fs；' ...
    '倒车 %.3fm，换挡 %d次，零速等待 %.3fs。\n'],result.validation_passed, ...
    result.forward_only_failed,result.forward_only_path.error_code, ...
    forward_seconds,reverse_seconds,result.elapsed_sec,result.reverse_distance_m, ...
    result.gear_switch_count,result.gear_wait_time_s);
end

function renderReverseResult(trajectory,geometry,vehicle,waits,shift_indices,result)
%RENDERREVERSERESULT 绘制实际搜索位姿和有符号速度，黄色区域表示整段换挡驻留。
fig = figure('Visible','off','Color','w','Position',[100,100,1200,800]);
figure_cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
layout = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax_xy = nexttile(layout,1); hold(ax_xy,'on');
ax_xy.FontName = 'Microsoft YaHei';
plot(ax_xy,geometry.boundary_xy(:,1),geometry.boundary_xy(:,2), ...
    'Color',[.25,.25,.25],'LineWidth',1.4,'HandleVisibility','off');
trajectory_line = plot(ax_xy,trajectory.x,trajectory.y,'Color',[.8,.2,.15], ...
    'LineWidth',2.2);
footprint = inflate_vehicle_occupancy(vehicle,0);
start_body = drawVehicle(ax_xy,trajectory,1,footprint,[.2,.45,.8],.14);
drawVehicle(ax_xy,trajectory,max(2,round(numel(trajectory.x)/2)),footprint,[.55,.55,.55],.05);
goal_body = drawVehicle(ax_xy,trajectory,numel(trajectory.x),footprint,[.15,.6,.35],.14);
scatter(ax_xy,trajectory.x(1),trajectory.y(1),45,[.2,.45,.8],'filled','HandleVisibility','off');
scatter(ax_xy,trajectory.x(end),trajectory.y(end),45,[.15,.6,.35],'filled','HandleVisibility','off');
middle = max(2,round(numel(trajectory.x)/2));
text(ax_xy,trajectory.x(middle),3.02,'车身朝右，运动朝左：实际倒车', ...
    'HorizontalAlignment','center','FontSize',11,'Interpreter','none');
axis(ax_xy,'equal'); xlim(ax_xy,[0,20]); ylim(ax_xy,[-.4,3.35]); grid(ax_xy,'on');
xlabel(ax_xy,'X / m'); ylabel(ax_xy,'Y / m');
title(ax_xy,sprintf('狭窄通道真实搜索：前进搜索未成功（有限预算，错误码 %d），倒车规划成功', ...
    result.forward_only_path.error_code),'Interpreter','none');
legend(ax_xy,[trajectory_line,start_body,goal_body], ...
    {'搜索生成的倒车轨迹','起始车辆安全包络','目标车辆安全包络'},'Location','southoutside', ...
    'Orientation','horizontal','Interpreter','none');
ax_speed = nexttile(layout,2); hold(ax_speed,'on'); ax_speed.FontName = 'Microsoft YaHei';
signed_speed = trajectory.gear.*trajectory.v;
limits = [min([-.2;signed_speed(:)])-.1,max([.2;signed_speed(:)])+.1];
for k=1:numel(shift_indices)
    right = trajectory.t(shift_indices(k)); left = trajectory.t(shift_indices(k)-1);
    patch(ax_speed,[left,right,right,left],limits([1,1,2,2]),[.96,.77,.25], ...
        'FaceAlpha',.24,'EdgeColor','none','HandleVisibility','off');
    text(ax_speed,(left+right)/2,limits(2)-.06,sprintf('换挡等待 %.2fs',waits(k)), ...
        'HorizontalAlignment','center','VerticalAlignment','top','FontSize',10,'Interpreter','none');
end
plot(ax_speed,trajectory.t,signed_speed,'Color',[.8,.2,.15],'LineWidth',2.2);
yline(ax_speed,0,'--','Color',[.4,.4,.4],'HandleVisibility','off');
xlim(ax_speed,[trajectory.t(1),trajectory.t(end)]); ylim(ax_speed,limits); grid(ax_speed,'on');
xlabel(ax_speed,'时间 t / s'); ylabel(ax_speed,'有符号速度 gear × v / (m/s)');
title(ax_speed,'负值表示倒车；速度大小 v 非负，停稳换挡后再加速，终点零速停车', ...
    'Interpreter','none');
ax_speed.Layer = 'top';
exportgraphics(fig,result.plot_path,'Resolution',180);
end

function body = drawVehicle(ax,trajectory,index,footprint,color,alpha)
%DRAWVEHICLE 以实际后轴位姿旋转车辆安全矩形；箭头指示真实车身航向。
theta = trajectory.theta(index); c = cos(theta); s = sin(theta);
corners = footprint.corners_local*[c,s;-s,c]+[trajectory.x(index),trajectory.y(index)];
body = patch(ax,corners(:,1),corners(:,2),color,'FaceAlpha',alpha,'EdgeColor',color,'LineWidth',1.2);
quiver(ax,trajectory.x(index),trajectory.y(index),1.6*c,1.6*s,0, ...
    'Color',color,'LineWidth',1.4,'MaxHeadSize',.5,'HandleVisibility','off');
end
