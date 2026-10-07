function summary = validate_day6_sequential(save_outputs)
%VALIDATE_DAY6_SEQUENTIAL 执行Day3至Day6回归和真实两车顺序规划验收。
% 输入：save_outputs是否生成独立Day6图、GIF和JSON报告，默认true。
% 输出：summary记录MATLAB版本、测试结果、两车时间/速度/误差和动态净距。
% 验收失败时保留结果报告并报错，不能用单元测试代替实际Demo规划结果。
if nargin < 1, save_outputs = true; end
test_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(test_dir);
addpath(fullfile(project_dir,'matlab','single_vehicle'));
files = {'test_vehicle_dynamic.m','test_vhybrid_core.m', ...
    'test_st_occupancy_map.m','test_dynamic_collision.m', ...
    'test_reverse_motion.m','test_reverse_search.m', ...
    'test_resource_query_equivalence.m','test_day6_performance_equivalence.m', ...
    'test_motion_sample_cache.m'};
results = runtests(cellfun(@(name) fullfile(test_dir,name),files,'UniformOutput',false));
summary = struct('matlab_version',version,'checked_at',char(datetime('now')), ...
    'test_count',numel(results),'passed',sum([results.Passed]), ...
    'failed',sum([results.Failed]),'incomplete',sum([results.Incomplete]), ...
    'failed_tests',{{results([results.Failed]).Name}},'valid',false);
demo = run_day6_sequential_planning_demo(save_outputs);
summary.demo = demo.validation;
summary.timings = demo.timings;
summary.high_priority = pathSummary(demo.high_priority_path);
summary.low_priority = pathSummary(demo.low_priority_path);
summary.valid = summary.failed == 0 && summary.incomplete == 0 && demo.valid;
if save_outputs
    output_dir = fullfile(project_dir,'outputs');
    if ~exist(output_dir,'dir'), mkdir(output_dir); end
    fid = fopen(fullfile(output_dir,'day6_validation.json'),'w','n','UTF-8');
    if fid < 0, error('Day6:OutputFailure','无法保存Day6验收结果。'); end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,'PrettyPrint',true));
    clear cleanup;
end
disp(summary);
assert(summary.valid,'Day6:ValidationFailed','测试或真实两车规划未通过，请检查Day6输出报告。');
end

function value = pathSummary(path)
%PATHSUMMARY 保存实际搜索统计和轨迹首末状态，速度/航向必须来自模型输出。
value = struct('valid',path.valid,'error_code',path.error_code,'search_statistics',path.search_statistics);
if ~isempty(path.t)
    value.start_state = [path.x(1),path.y(1),path.theta(1),path.v(1),path.t(1)];
    value.final_state = [path.x(end),path.y(end),path.theta(end),path.v(end),path.t(end)];
    value.maximum_speed_mps = max(path.v);
    value.final_position_error_m = path.final_position_error_m;
    value.final_heading_error_rad = path.final_heading_error_rad;
    value.kinematic_validation = path.kinematic_validation;
    if isfield(path,'gear')
        value.reverse_samples = nnz(path.gear < 0 & path.v > 1e-9);
        value.gear_switches = nnz(diff(path.gear) ~= 0);
    end
end
end
