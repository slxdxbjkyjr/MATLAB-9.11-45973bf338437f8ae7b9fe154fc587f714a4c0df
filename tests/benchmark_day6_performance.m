function report = benchmark_day6_performance(baseline_dir,optimized_dir,repetitions)
%BENCHMARK_DAY6_PERFORMANCE 在同一Day1场景中公平比较性能、转角档数与倒车开关。
% 输入：优化前只读工程目录、优化后工程目录、重复次数(至少3，默认3)。
% 输出：四组暖机/交替重复记录、成功率和各阶段计算时间(s)，另存JSON报告。
% 组1旧版3档前进；组2新版3档前进；组3新版5档前进；组4新版5档可倒车。
% 只变转角采样与reverse_enabled，连接距离/速度/目标/安全采样/搜索上限均不变。
% save_outputs=false只关闭图像和结果文件；完整独立安全验收仍会执行。
% 原版没有options接口，因此将源码复制到临时目录并使用同一当前配置/场景。
if nargin < 2 || isempty(optimized_dir)
    optimized_dir = fileparts(fileparts(mfilename('fullpath')));
end
if nargin < 1 || isempty(baseline_dir)
    baseline_dir = fullfile(fileparts(optimized_dir),'day6_oct7_baseline');
end
if nargin < 3, repetitions = 3; end
validateattributes(repetitions,{'numeric'},{'scalar','integer','>=',3});
assert(isfolder(fullfile(baseline_dir,'matlab','single_vehicle')), ...
    'Day6Benchmark:MissingBaseline','缺少可运行的优化前只读工程目录。');
old_path = path;
path_cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
current_config_dir = fullfile(optimized_dir,'config');
current = jsondecode(fileread(fullfile(current_config_dir,'planner_config.json')));
maximum_steer = current.dynamics.delta_max_rad;
three_steers = [-maximum_steer,0,maximum_steer];
five_steers = [-maximum_steer,-maximum_steer/2,0,maximum_steer/2,maximum_steer];
configs = repmat({current},1,4);
configs{1}.vhybrid.control_steering_samples_rad = three_steers;
configs{2}.vhybrid.control_steering_samples_rad = three_steers;
configs{3}.vhybrid.control_steering_samples_rad = five_steers;
configs{4}.vhybrid.control_steering_samples_rad = five_steers;
for k = 1:4
    configs{k}.vhybrid.reverse_enabled = k == 4;
    configs{k}.day6.save_animation = false;
end

% 临时副本只用于适配旧接口，不覆盖baseline或用户工程文件。
temporary_baseline = tempname;
mkdir(temporary_baseline);
copyfile(fullfile(baseline_dir,'matlab'),fullfile(temporary_baseline,'matlab'));
copyfile(current_config_dir,fullfile(temporary_baseline,'config'));
writeJson(fullfile(temporary_baseline,'config','planner_config.json'),configs{1});
old_source = fullfile(temporary_baseline,'matlab','single_vehicle');
new_source = fullfile(optimized_dir,'matlab','single_vehicle');
old_names = sourceNames(old_source); new_names = sourceNames(new_source);
names = unique([old_names,new_names]);
case_names = {'baseline_3_forward','optimized_3_forward', ...
    'optimized_5_forward','optimized_5_reverse'};
report = struct('created_at',char(datetime('now')), ...
    'matlab_version',version,'baseline_dir',baseline_dir, ...
    'optimized_dir',optimized_dir,'temporary_baseline_dir',temporary_baseline, ...
    'repetitions',repetitions,'scenario', ...
    jsondecode(fileread(fullfile(current_config_dir,'scenario_001.json'))), ...
    'comparison_parameters',struct('steering_3_rad',three_steers, ...
    'steering_5_rad',five_steers,'current_planner_config',current), ...
    'measurement_note',['每组先暖机一次，正式轮次交替正/逆序；wall_compute_s包含完整安全验收，' ...
    '关闭PNG/GIF/JSON/MAT输出。失败及超时原样记录，不将失败速度当成成功加速。'], ...
    'warmups',struct([]),'runs',struct([]),'summary',struct([]));
output_dir = fullfile(optimized_dir,'outputs');
if ~isfolder(output_dir), mkdir(output_dir); end
report_file = fullfile(output_dir,'day6_performance_comparison.json');

for index = 1:4
    fprintf('Day6基准暖机 %d/4：%s\n',index,case_names{index});
    record = runCase(index,0,case_names{index},configs{index},old_source,new_source,names);
    report.warmups = appendRecord(report.warmups,record);
    writeJson(report_file,report);
end
for round_index = 1:repetitions
    order = 1:4;
    if mod(round_index,2) == 0, order = 4:-1:1; end
    for index = order
        fprintf('Day6基准轮次 %d/%d：%s\n',round_index,repetitions,case_names{index});
        record = runCase(index,round_index,case_names{index},configs{index}, ...
            old_source,new_source,names);
        report.runs = appendRecord(report.runs,record);
        writeJson(report_file,report);
    end
end
for index = 1:4
    runs = report.runs([report.runs.case_index] == index);
    valid = [runs.valid];
    case_summary = struct('case_name',case_names{index}, ...
        'successful_runs',sum(valid),'total_runs',numel(runs),'success_rate',mean(valid), ...
        'median_wall_compute_s',median([runs.wall_compute_s]), ...
        'median_successful_wall_compute_s',safeMedian([runs(valid).wall_compute_s]), ...
        'median_high_search_s',safeMedian([runs.high_search_s]), ...
        'median_low_search_s',safeMedian([runs.low_search_s]), ...
        'median_high_expanded_nodes',safeMedian([runs.high_expanded_nodes]), ...
        'median_low_expanded_nodes',safeMedian([runs.low_expanded_nodes]), ...
        'median_low_generated_nodes',safeMedian([runs.low_generated_nodes]), ...
        'minimum_successful_body_clearance_m',safeMinimum([runs(valid).minimum_distance_m]), ...
        'all_successful_runs_zero_resource_overlap', ...
        any(valid) && all([runs(valid).resource_overlap_blocks] == 0) && ...
        all([runs(valid).independent_resource_overlap_blocks] == 0), ...
        'failure_reasons',{{runs(~valid).reason}});
    report.summary = appendRecord(report.summary,case_summary);
end
report.report_file = report_file;
writeJson(report_file,report);
disp(struct2table(report.summary));
end

function records = appendRecord(records,record)
%APPENDRECORD 空struct([])没有字段，首条直接赋值后再追加同字段记录。
if isempty(records)
    records = record;
else
    assert(isequal(fieldnames(records),fieldnames(record)), ...
        'Day6Benchmark:RecordShapeMismatch','同组基准记录字段不一致。');
    records(end+1) = record;
end
end

function record = runCase(index,round_index,name,config,old_source,new_source,names)
%RUNCASE 清除同名函数缓存并切换路径，确保确实运行对应版本；不隐藏失败。
paths_to_remove = {old_source,new_source};
for k = 1:numel(paths_to_remove)
    if contains([path,pathsep],[paths_to_remove{k},pathsep]), rmpath(paths_to_remove{k}); end
end
clear(names{:});
if index == 1, source = old_source; else, source = new_source; end
addpath(source,'-begin');
record = struct('case_index',index,'case_name',name,'round',round_index, ...
    'actual_demo_file',which('run_day6_sequential_planning_demo'), ...
    'valid',false,'reason','not_run','exception_identifier','', ...
    'exception_message','','wall_compute_s',NaN,'high_search_s',NaN, ...
    'low_search_s',NaN,'resource_map_s',NaN,'independent_validation_s',NaN, ...
    'high_expanded_nodes',NaN,'low_expanded_nodes',NaN,'low_generated_nodes',NaN, ...
    'minimum_distance_m',NaN,'resource_overlap_blocks',NaN, ...
    'independent_resource_overlap_blocks',NaN,'both_kinematics_valid',false, ...
    'both_move_immediately',false,'reverse_samples',0,'gear_switches',0, ...
    'high_error_code','','low_error_code','','high_search_statistics',struct(), ...
    'low_search_statistics',struct());
timer_id = tic;
try
    if index == 1
        result = run_day6_sequential_planning_demo(false);
    else
        result = run_day6_sequential_planning_demo(false,struct('planner_config',config));
    end
    record.wall_compute_s = toc(timer_id);
    record.valid = result.valid;
    record.reason = result.validation.reason;
    record.high_search_s = statisticsValue(result.high_priority_path,'elapsed_sec');
    record.low_search_s = statisticsValue(result.low_priority_path,'elapsed_sec');
    record.high_expanded_nodes = statisticsValue(result.high_priority_path,'expanded_nodes');
    record.low_expanded_nodes = statisticsValue(result.low_priority_path,'expanded_nodes');
    record.low_generated_nodes = statisticsValue(result.low_priority_path,'generated_nodes');
    record.high_error_code = fieldValue(result.high_priority_path,'error_code','');
    record.low_error_code = fieldValue(result.low_priority_path,'error_code','');
    record.high_search_statistics = fieldValue(result.high_priority_path,'search_statistics',struct());
    record.low_search_statistics = fieldValue(result.low_priority_path,'search_statistics',struct());
    record.minimum_distance_m = result.validation.minimum_distance_m;
    record.resource_overlap_blocks = result.validation.resource_overlap_blocks;
    record.independent_resource_overlap_blocks = ...
        fieldValue(result.validation,'independent_resource_overlap_blocks',NaN);
    record.both_kinematics_valid = result.validation.both_kinematics_valid;
    record.both_move_immediately = result.validation.both_move_immediately;
    if isfield(result,'timings')
        record.resource_map_s = result.timings.resource_map_s;
        record.independent_validation_s = result.timings.independent_validation_s;
    end
    if isfield(result.low_priority_path,'gear')
        record.reverse_samples = nnz(result.low_priority_path.gear < 0 & result.low_priority_path.v > 1e-9);
        record.gear_switches = nnz(diff(result.low_priority_path.gear) ~= 0);
    end
catch exception
    record.wall_compute_s = toc(timer_id);
    record.reason = 'exception';
    record.exception_identifier = exception.identifier;
    record.exception_message = exception.message;
end
fprintf('  wall %.3f s；success %d；reason %s\n', ...
    record.wall_compute_s,record.valid,record.reason);
end

function names = sourceNames(source)
%SOURCENAMES 仅清理工程源码函数名，不用clear all，避免破坏用户工作区。
files = dir(fullfile(source,'*.m'));
names = cellfun(@(value) erase(value,'.m'),{files.name},'UniformOutput',false);
end

function value = statisticsValue(path,key)
%STATISTICSVALUE 尚未规划或缺少某个统计时记录NaN，禁止伪造零耗时。
value = NaN;
if isfield(path,'search_statistics'), value = fieldValue(path.search_statistics,key,NaN); end
end

function value = fieldValue(data,key,fallback)
%FIELDVALUE 兼容优化前后报告字段差异，未产生的数据使用显式缺省值。
value = fallback;
if isfield(data,key), value = data.(key); end
end

function value = safeMedian(values)
%SAFEMEDIAN 无成功样本时返回NaN，避免把失败运行用于成功效率结论。
value = NaN;
if ~isempty(values), value = median(values); end
end

function value = safeMinimum(values)
%SAFEMINIMUM 输出成功重复中的最小车身净距，无样本时保持NaN。
value = NaN;
if ~isempty(values), value = min(values); end
end

function writeJson(filename,data)
%WRITEJSON 保存每一轮原始记录；基准中断后仍能看到已完成运行及真实失败。
fid = fopen(filename,'w','n','UTF-8');
if fid < 0, error('Day6Benchmark:OutputFailure','无法保存基准报告。'); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(data,'PrettyPrint',true));
end
