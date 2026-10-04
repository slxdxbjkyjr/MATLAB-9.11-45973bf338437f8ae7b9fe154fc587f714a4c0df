function files = plot_day6_trajectories(high,low,geometry,vehicle,planner,output_dir)
%PLOT_DAY6_TRAJECTORIES 显示真实搜索轨迹、速度时间、车间距离和同步运动GIF。
% 输入：两车有效搜索输出、Day1地图/车辆、集中配置、独立Day6输出目录。
% 输出：PNG/GIF的绝对路径。只可视化已有搜索数据，不生成或修正路径。
% 动画图元仅创建一次，后续更新矩形/位置文字，按同一绝对时间插值两车。
if ~high.valid || ~low.valid
    error('Day6:InvalidPlotPath','只允许绘制已验收的有效搜索轨迹。');
end
if ~exist(output_dir,'dir'), mkdir(output_dir); end
files = struct('trajectory_png',fullfile(output_dir,'day6_search_trajectories.png'), ...
    'speed_png',fullfile(output_dir,'day6_speed_time.png'), ...
    'animation_gif',fullfile(output_dir,'day6_sequential_planning.gif'));
colors = [0.10,0.42,0.82;0.90,0.34,0.12];
high_obstacle = dynamic_vehicle_obstacle(high,vehicle,'vehicle_002',planner);
low_obstacle = dynamic_vehicle_obstacle(low,vehicle,'vehicle_003',planner);
high_obstacle.hold_end = true; low_obstacle.hold_end = true;
end_time = max(high.t(end),low.t(end));
sample_times = linspace(0,end_time,max(1,ceil(end_time/planner.day6.collision_sample_step_s))+1);
[high_states,~] = predict_vehicle_pose(high_obstacle,sample_times);
[low_states,~] = predict_vehicle_pose(low_obstacle,sample_times);
distances = zeros(size(sample_times));
for k = 1:numel(sample_times)
    [~,detail] = check_vehicle_vehicle_collision(high_states(k,:),low_states(k,:), ...
        vehicle,vehicle,planner.day6);
    distances(k) = detail.minimum_distance_m;
end

fig = figure('Visible','off','Color','w','Position',[80,80,1200,850]);
cleanup = onCleanup(@() close(fig));
layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
xy = nexttile(layout,[1,2]);
drawMap(xy,geometry);
drawPath(xy,high,vehicle,colors(1,:),'vehicle\_002 (high)');
drawPath(xy,low,vehicle,colors(2,:),'vehicle\_003 (low)');
title(xy,sprintf('Sequential V-Hybrid A*: both start at t = 0, v = 0; arrivals %.2f / %.2f s', ...
    high.t(end),low.t(end)));
legend(xy,'show','Location','eastoutside');
speed = nexttile(layout);
drawSpeed(speed,high,low,end_time,colors);
distance = nexttile(layout);
plot(distance,sample_times,distances,'Color',[0.22,0.55,0.30],'LineWidth',1.5);
grid(distance,'on'); xlabel(distance,'t / s'); ylabel(distance,'Body clearance / m');
title(distance,sprintf('Sampled rectangle clearance, minimum %.3f m',min(distances)));
xlim(distance,[0,max(end_time,eps)]);
exportgraphics(fig,files.trajectory_png,'Resolution',160);
clear cleanup;

speed_fig = figure('Visible','off','Color','w','Position',[80,80,950,420]);
cleanup = onCleanup(@() close(speed_fig));
speed_ax = axes(speed_fig);
drawSpeed(speed_ax,high,low,end_time,colors);
exportgraphics(speed_fig,files.speed_png,'Resolution',160);
clear cleanup;

% 通过位置/航向的时间预测播放真实路径；车辆先到终点后保持停车。
animation = figure('Visible','off','Color','w','Position',[80,80,900,650]);
cleanup = onCleanup(@() close(animation));
ax = axes(animation);
drawMap(ax,geometry);
plot(ax,high.x,high.y,'--','Color',colors(1,:),'LineWidth',1.2,'DisplayName','vehicle\_002 path');
plot(ax,low.x,low.y,'--','Color',colors(2,:),'LineWidth',1.2,'DisplayName','vehicle\_003 path');
body1 = drawBody(ax,high_states(1,:),vehicle,colors(1,:),0.45);
body2 = drawBody(ax,low_states(1,:),vehicle,colors(2,:),0.45);
rear1 = plot(ax,high.x(1),high.y(1),'o','Color',colors(1,:),'MarkerFaceColor',colors(1,:), ...
    'HandleVisibility','off');
rear2 = plot(ax,low.x(1),low.y(1),'o','Color',colors(2,:),'MarkerFaceColor',colors(2,:), ...
    'HandleVisibility','off');
heading1 = quiver(ax,high.x(1),high.y(1),cos(high.theta(1)),sin(high.theta(1)),0, ...
    'Color',colors(1,:),'LineWidth',1.5,'HandleVisibility','off');
heading2 = quiver(ax,low.x(1),low.y(1),cos(low.theta(1)),sin(low.theta(1)),0, ...
    'Color',colors(2,:),'LineWidth',1.5,'HandleVisibility','off');
clock_text = title(ax,'');
legend(ax,'show','Location','eastoutside');
if isprop(ax,'Toolbar'), ax.Toolbar = []; end
step = planner.day6.animation_step_s;
frame_times = unique([0:step:end_time,end_time]);
[high_frames,~] = predict_vehicle_pose(high_obstacle,frame_times);
[low_frames,~] = predict_vehicle_pose(low_obstacle,frame_times);
for k = 1:numel(frame_times)
    a = high_frames(k,:); b = low_frames(k,:);
    corners1 = bodyCorners(a,vehicle); corners2 = bodyCorners(b,vehicle);
    set(body1,'XData',corners1(:,1),'YData',corners1(:,2));
    set(body2,'XData',corners2(:,1),'YData',corners2(:,2));
    set(rear1,'XData',a(1),'YData',a(2)); set(rear2,'XData',b(1),'YData',b(2));
    set(heading1,'XData',a(1),'YData',a(2),'UData',cos(a(3)),'VData',sin(a(3)));
    set(heading2,'XData',b(1),'YData',b(2),'UData',cos(b(3)),'VData',sin(b(3)));
    clock_text.String = sprintf('t = %.2f s | vehicle 002: %.2f m/s | vehicle 003: %.2f m/s', ...
        frame_times(k),a(4),b(4));
    drawnow limitrate;
    frame = getframe(animation);
    [indexed,palette] = rgb2ind(frame.cdata,256);
    if k == 1
        imwrite(indexed,palette,files.animation_gif,'gif','LoopCount',Inf, ...
            'DelayTime',1/planner.day6.animation_frame_rate);
    else
        imwrite(indexed,palette,files.animation_gif,'gif','WriteMode','append', ...
            'DelayTime',1/planner.day6.animation_frame_rate);
    end
end
clear cleanup;
end

function drawMap(ax,map)
%DRAWMAP 独立轴显示原场景边界和静态障碍，固定比例与范围。
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
plot(ax,map.boundary_xy(:,1),map.boundary_xy(:,2),'k-','LineWidth',1.5,'DisplayName','parking boundary');
for k = 1:numel(map.obstacles)
    obstacle = map.obstacles{k};
    patch(ax,obstacle(:,1),obstacle(:,2),[0.45,0.45,0.45],'HandleVisibility','off');
end
xlim(ax,map.bounds([1,3])+[-0.5,0.5]); ylim(ax,map.bounds([2,4])+[-0.5,0.5]);
xlabel(ax,'x / m'); ylabel(ax,'y / m');
end

function drawPath(ax,path,vehicle,color,label)
%DRAWPATH 显示同一搜索器生成的节点、连续路径及起终点车身航向。
if ~isempty(path.search_nodes)
    samples = unique(round(linspace(1,numel(path.search_nodes),min(10000,numel(path.search_nodes)))));
    nodes = path.search_nodes(samples);
    scatter(ax,[nodes.x],[nodes.y],5,color,'filled','MarkerFaceAlpha',0.10,'HandleVisibility','off');
end
drawBody(ax,[path.x(1),path.y(1),path.theta(1)],vehicle,color,0.13);
drawBody(ax,[path.x(end),path.y(end),path.theta(end)],vehicle,color,0.23);
plot(ax,path.x,path.y,'-','Color',color,'LineWidth',1.8,'DisplayName',label);
plot(ax,path.x(1),path.y(1),'o','Color',color,'MarkerFaceColor',color,'HandleVisibility','off');
plot(ax,path.x(end),path.y(end),'p','Color',color,'MarkerSize',12,'HandleVisibility','off');
samples = unique(round(linspace(1,numel(path.x),min(9,numel(path.x)))));
quiver(ax,path.x(samples),path.y(samples),0.8*cos(path.theta(samples)),0.8*sin(path.theta(samples)), ...
    0,'Color',color,'LineWidth',1.0,'HandleVisibility','off');
end

function body = drawBody(ax,pose,vehicle,color,alpha)
%DRAWBODY 绘制物理车身矩形；安全裕度只用于检测，不夸大实车图形。
corners = bodyCorners(pose,vehicle);
body = patch(ax,corners(:,1),corners(:,2),color,'FaceAlpha',alpha,'EdgeColor',color, ...
    'LineWidth',1.2,'HandleVisibility','off');
end

function corners = bodyCorners(pose,vehicle)
%BODYCORNERS 后轴参考点+车身局部四角，通过旋转矩阵得到全局坐标。
vehicle.collision_margin_m = struct('front',0,'rear',0,'side',0);
footprint = inflate_vehicle_occupancy(vehicle,0);
rotation = [cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
corners = footprint.corners_local*rotation.'+pose(1:2);
end

function drawSpeed(ax,high,low,end_time,colors)
%DRAWSPEED 原路径速度含加/减速和搜索等待，到达后延伸v=0以对齐两车时钟。
hold(ax,'on'); grid(ax,'on');
plot(ax,[high.t(:);end_time],[high.v(:);0],'-','Color',colors(1,:),'LineWidth',1.6, ...
    'DisplayName','vehicle\_002 (high)');
plot(ax,[low.t(:);end_time],[low.v(:);0],'-','Color',colors(2,:),'LineWidth',1.6, ...
    'DisplayName','vehicle\_003 (low)');
xlabel(ax,'t / s'); ylabel(ax,'v / m/s');
title(ax,'Both start from rest; acceleration/braking from the vehicle model');
xlim(ax,[0,max(end_time,eps)]); legend(ax,'show','Location','best');
end
