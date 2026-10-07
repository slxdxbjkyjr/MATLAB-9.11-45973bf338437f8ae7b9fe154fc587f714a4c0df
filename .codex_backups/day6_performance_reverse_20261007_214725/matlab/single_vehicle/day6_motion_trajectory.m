function trajectory = day6_motion_trajectory(start, next, steering, acceleration, vehicle, planner)
%DAY6_MOTION_TRAJECTORY 对搜索边用同一个Day3运动学模型生成中间状态。
% 输入：首末Nx5状态、控制、车辆和配置。输出：带控制量的密集path。
% 时间与弧长约束共同限制采样；包括起点、终点及减速边中点。
duration = next(5)-start(5);
if duration <= 0
    error('DynamicCollision:InvalidTrajectory','搜索边时间必须严格增加。');
end
distance = 0.5*(start(4)+next(4))*duration;
count = max([2,ceil(duration/planner.day6.collision_sample_step_s), ...
    ceil(distance/planner.day6.collision_sample_step_m)]);
sample_times = unique([linspace(0,duration,count+1),duration/2]);
states = zeros(numel(sample_times),5); states(1,:) = start;
dyn = planner.dynamics;
for k = 2:numel(sample_times)
    [states(k,:),~,~,valid] = summon_vehicle_dynamic(start,steering,acceleration,sample_times(k), ...
        vehicle.dimensions_m.wheelbase,dyn.v_max_mps,dyn.v_min_mps,dyn.a_min_mps2, ...
        dyn.a_max_mps2,min(vehicle.steering.maximum_steer_rad,dyn.delta_max_rad));
    if ~valid
        error('DynamicCollision:InvalidTrajectory','搜索边违反运动学约束。');
    end
end
trajectory = struct('x',states(:,1),'y',states(:,2),'theta',states(:,3), ...
    'v',states(:,4),'t',states(:,5),'acceleration',[0;repmat(acceleration,numel(sample_times)-1,1)], ...
    'steering_angle',[0;repmat(steering,numel(sample_times)-1,1)],'valid',true);
end
