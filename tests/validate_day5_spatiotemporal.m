function report = validate_day5_spatiotemporal(output_dir)
%VALIDATE_DAY5_SPATIOTEMPORAL 执行Day5验收和已有Day3/4回归，保存可复核结果。
% 输入：可选输出目录；输出：版本、逐项测试、静态分析和Day4两车占用示例。
% 逻辑：测试全部通过后生成四视图；仅写Day5新输出，不重写旧Day4图片。
project = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1, output_dir = fullfile(project,'outputs'); end
original_path = path;
cleanup_path = onCleanup(@() path(original_path)); %#ok<NASGU>
addpath(fullfile(project,'matlab','single_vehicle'));
addpath(fullfile(project,'tests'));
files = {fullfile(project,'tests','test_st_occupancy_map.m'), ...
    fullfile(project,'tests','test_vehicle_dynamic.m'),fullfile(project,'tests','test_vhybrid_core.m')};
results = runtests(files);
report = struct('matlab_version',version,'generated_at',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
    'tests',struct('total',numel(results),'passed',nnz([results.Passed]), ...
    'failed',nnz([results.Failed]),'incomplete',nnz([results.Incomplete])), ...
    'test_details',struct('name',{results.Name},'passed',num2cell([results.Passed]), ...
        'failed',num2cell([results.Failed]),'duration_s',num2cell([results.Duration])));
names = {'st_occupancy_map','validate_st_occupancy_config','inflate_vehicle_occupancy', ...
    'mark_static_obstacles','mark_trajectory_occupancy','query_occupancy','plot_st_occupancy', ...
    'st_resource_index','st_vehicle_cells','st_write_occupancy','st_polyline_distance'};
report.code_analysis = struct('file',{},'messages',{});
for k = 1:numel(names)
    messages = checkcode(fullfile(project,'matlab','single_vehicle',[names{k},'.m']),'-id');
    report.code_analysis(k) = struct('file',names{k},'messages',messages);
end
assert(all([results.Passed]),'Summon:Day5:TestsFailed','Day5或Day3/4回归未全部通过。');
report.demo = generate_st_occupancy_plot(output_dir);
if ~exist(output_dir,'dir'), mkdir(output_dir); end
file = fullfile(output_dir,'day5_validation.json');
fid = fopen(file,'w','n','UTF-8');
assert(fid >= 0,'Summon:Day5:WriteFailed','无法创建Day5报告。');
cleanup_file = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));
fprintf('Day5/regression: %d/%d passed. Report: %s\n',report.tests.passed,report.tests.total,file);
end
