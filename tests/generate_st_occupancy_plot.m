function report = generate_st_occupancy_plot(output_dir)
%GENERATE_ST_OCCUPANCY_PLOT 使用 Day1 场景、Day4 独立轨迹生成 Day5 资源块示意。
%
% 输入：output_dir 可选输出目录，默认本工程 outputs。
% 输出：report 保存来源车辆、轨迹时间范围、占用块数量和图/数据文件路径。
% 逻辑：vehicle_002 与 vehicle_003 各自在 t=0、v=0 通过原 Day4 搜索规划。
%       然后仅标记静态地图和两条时空轨迹，允许显示冲突，不重新规划或动态避碰。
%       原场景没有静态障碍物；本例保留真实场景，不添加虚构障碍物。
%       Day5 的保守全航向静态膨胀可能与原 Day4 精确矩形边界判断不同。

this_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(this_dir);
if nargin < 1 || isempty(output_dir), output_dir = fullfile(project_dir,'outputs'); end
addpath(fullfile(project_dir,'matlab','single_vehicle'));
config_dir = fullfile(project_dir,'config');
planner_config = jsondecode(fileread(fullfile(config_dir,'planner_config.json')));
vehicle_config = jsondecode(fileread(fullfile(config_dir,'vehicle_config.json')));
map_config = jsondecode(fileread(fullfile(config_dir,'map_config.json')));
scenario = jsondecode(fileread(fullfile(config_dir,'scenario_001.json')));
geometry = summon_map(map_config);
% summon_map 默认读取地图级障碍物；若场景另含障碍物，合并到本次运行内存。
if isfield(scenario,'static_obstacles')
    for k = 1:numel(scenario.static_obstacles)
        geometry.obstacles{end+1,1} = double(scenario.static_obstacles(k).vertices_xy);
    end
end
resource_map = st_occupancy_map(planner_config,geometry);
resource_map = mark_static_obstacles(resource_map,geometry,vehicle_config);
vehicle_indices = [2,3];
paths = cell(numel(vehicle_indices),1);
vehicle_ids = cell(numel(vehicle_indices),1);
durations = zeros(numel(vehicle_indices),2);
for k = 1:numel(vehicle_indices)
    index = vehicle_indices(k);
    paths{k} = run_vhybrid_astar_demo(index,false);
    vehicle_ids{k} = char(scenario.vehicles(index).id);
    if ~paths{k}.valid
        error('Summon:STDemo:PlanningFailed', ...
            'Day4 车辆 %s 规划失败，error_code=%d；未生成成功图。', ...
            vehicle_ids{k},double(paths{k}.error_code));
    end
    resource_map = mark_trajectory_occupancy(resource_map,paths{k},vehicle_config,vehicle_ids{k});
    durations(k,:) = [paths{k}.t(1),paths{k}.t(end)];
end
if ~exist(output_dir,'dir'), mkdir(output_dir); end
figure_file = fullfile(output_dir,'day5_st_occupancy.png');
data_file = fullfile(output_dir,'day5_st_occupancy.mat');
plot_st_occupancy(resource_map,figure_file);
report = struct('figure_file',figure_file,'data_file',data_file, ...
    'vehicle_ids',{vehicle_ids},'trajectory_time_ranges_s',durations, ...
    'occupied_blocks',nnz(resource_map.occupancy), ...
    'boundary_blocks',nnz(resource_map.owner_type==uint8(1)), ...
    'static_obstacle_blocks',nnz(resource_map.owner_type==uint8(2)), ...
    'dynamic_blocks',nnz(resource_map.owner_type==uint8(3)), ...
    'multiple_owner_blocks',nnz(resource_map.owner_type==uint8(4)), ...
    'static_obstacle_count',numel(geometry.obstacles), ...
    'dynamic_avoidance_enabled',false, ...
    'note',['Day5 占用接口示意：两条 Day4 独立规划轨迹允许存在冲突；' ...
    '静态区域使用保守全航向车辆参考点膨胀；不属于 Day6 顺序避碰结果。']);
save(data_file,'resource_map','paths','report','-v7');
disp(report);
end
