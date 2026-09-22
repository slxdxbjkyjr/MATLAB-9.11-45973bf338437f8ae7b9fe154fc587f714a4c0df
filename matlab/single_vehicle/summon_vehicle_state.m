function state = summon_vehicle_state(pose, vehicle_config, distance_m, steer_rad)
%SUMMON_VEHICLE_STATE 构造或推进召集阶段单车状态。
%
% 输入：
%   pose          - 当前/初始车辆位姿 [x, y, theta]，单位 m、rad。
%   vehicle_config- vehicle_config.json 解码后的车辆参数结构体。
%   distance_m    - 可选，沿车辆纵向行驶的有符号距离；正值前进，负值倒车。
%   steer_rad     - 可选，前轮转角，单位 rad。
%
% 输出：
%   state.pose             - 归一化后的 [x, y, theta]。
%   state.length_m/width_m- 车辆外包络尺寸。
%   state.wheelbase_m     - 轴距。
%   state.max_steer_rad   - 最大前轮转角。
%
% 状态变量和算法逻辑：
%   本函数是原 APA VehicleDynamic.m 的隔离适配器。它不读取
%   get_planner_global 的 persistent 状态，所有车辆参数都显式传入。
%   小转角时使用直线近似，其他情况使用运动学自行车圆弧模型。

if nargin < 2
    error('Summon:VehicleState:MissingInput', ...
        'pose 和 vehicle_config 为必需输入。');
end
if numel(pose) ~= 3 || any(~isfinite(double(pose)))
    error('Summon:VehicleState:InvalidPose', ...
        'pose 必须是有限的 [x,y,theta]。');
end

state = struct();
state.pose = double(pose(:).');
state.length_m = double(vehicle_config.dimensions_m.length);
state.width_m = double(vehicle_config.dimensions_m.width);
state.wheelbase_m = double(vehicle_config.dimensions_m.wheelbase);
state.rear_axle_to_rear_m = double(vehicle_config.dimensions_m.rear_axle_to_rear);
state.front_axle_to_front_m = double(vehicle_config.dimensions_m.front_axle_to_front);
state.max_steer_rad = double(vehicle_config.steering.maximum_steer_rad);

if nargin >= 4 
    state.pose = vehicleDynamic(state.pose, double(distance_m), double(steer_rad), ...
        state.wheelbase_m);
end
end

function next_pose = vehicleDynamic(pose, distance_m, steer_rad, wheelbase_m)
%VEHICLEDYNAMIC 隔离后的运动学自行车模型。
% 输入为连续位姿、带方向行驶距离、前轮转角和轴距；输出下一连续位姿。

theta = pose(3); 
if abs(steer_rad) < 1e-6  
    x_next = pose(1) + distance_m * cos(theta);
    y_next = pose(2) + distance_m * sin(theta);
    theta_next = theta;
else 
    radius_m = wheelbase_m / tan(steer_rad); 
    delta_theta = distance_m / radius_m;
    theta_next = theta + delta_theta;
    x_next = pose(1) + radius_m * (sin(theta_next) - sin(theta));
    y_next = pose(2) - radius_m * (cos(theta_next) - cos(theta));
end
next_pose = [x_next, y_next, wrapToPiLocal(theta_next)];
end

function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL 将航向角归一化到 [-pi, pi)。
angle = mod(angle + pi, 2 * pi) - pi;
end
