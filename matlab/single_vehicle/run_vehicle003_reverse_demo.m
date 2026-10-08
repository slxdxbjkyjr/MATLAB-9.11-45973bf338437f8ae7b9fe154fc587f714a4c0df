function result = run_vehicle003_reverse_demo(save_outputs)
%RUN_VEHICLE003_REVERSE_DEMO 在独立场景副本中验证车辆3从静止倒车起步。
% 输入：save_outputs为是否生成轨迹图、v-t图、动画和报告，默认true。
% 输出：与Day6主Demo相同的result，含实际输入起点、搜索、验收和分阶段计时。
% 修复前scenario_001中两车起点的安全矩形已重叠。本对照加载独立场景JSON，
% 将车辆3起点y设为13.5m，保留车型、0.2m裕度、目标及车辆2的起终点。
% 两车仍同时从v=0、t=0开始运动；路径均由同一搜索器生成，没有预设轨迹。
% 图和数据单独写入outputs/vehicle003_reverse_feasible，不覆盖原场景输出。
if nargin < 1, save_outputs = true; end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
scenario = jsondecode(fileread(fullfile(root,'config','scenario_vehicle003_reverse_feasible.json')));
options = struct('scenario_config',scenario,'output_subdir','vehicle003_reverse_feasible');
fprintf('独立倒车验证：车辆3起点y=13.5m；本入口不修改任何场景文件。\n');
result = run_day6_sequential_planning_demo(save_outputs,options);
end
