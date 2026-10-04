function report = validate_day4_kinematics()
%VALIDATE_DAY4_KINEMATICS 运行单元测试和三个真实场景，保存可复核的验证报告。
% 输入：无，从本工程目录加载配置。输出：测试汇总、三车终点及重放误差。
% 逻辑：先运行核心/运动学测试，再分别规划车辆1/2/3；更新默认车辆3演示图。
project = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project,'matlab','single_vehicle'));
results = runtests({fullfile(project,'tests','test_vhybrid_core.m'), ...
    fullfile(project,'tests','test_vehicle_dynamic.m')});
report.tests = struct('total',numel(results),'passed',nnz([results.Passed]), ...
    'failed',nnz([results.Failed]),'incomplete',nnz([results.Incomplete]));
items = cell(3,1);
for index = 1:3
    path = run_vhybrid_astar_demo(index,index==3);
    % 搜索失败可能返回空路径；仍写入失败报告，避免索引异常掩盖错误码。
    initial_speed = NaN;
    if ~isempty(path.v), initial_speed = path.v(1); end
    items{index} = struct('vehicle_index',index,'valid',path.valid, ...
        'error_code',double(path.error_code),'path_points',numel(path.x), ...
        'expanded_nodes',path.search_statistics.expanded_nodes, ...
        'initial_speed_mps',initial_speed, ...
        'position_error_m',path.final_position_error_m, ...
        'heading_error_deg',rad2deg(path.final_heading_error_rad), ...
        'speed_error_mps',path.final_speed_error_mps, ...
        'kinematic_validation',path.kinematic_validation);
    fprintf('vehicle%d valid=%d error=%d initial_speed=%.3g position=%.3g heading_deg=%.3g speed=%.3g\n', ...
        index,path.valid,path.error_code,initial_speed,path.final_position_error_m, ...
        rad2deg(path.final_heading_error_rad),path.final_speed_error_mps);
end
report.vehicles = vertcat(items{:});
output = fullfile(project,'outputs','day4_kinematics_validation.json');
fid = fopen(output,'w','n','UTF-8');
if fid < 0, error('Summon:Validation:WriteFailed','无法写入验证报告。'); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));
fprintf('Tests: %d/%d passed. Report: %s\n',report.tests.passed,report.tests.total,output);
assert(all([results.Passed]),'Day4/运动学单元测试未全部通过。');
assert(all([report.vehicles.valid]),'至少一辆车未通过终点/运动学验收。');
assert(all([report.vehicles.initial_speed_mps]==0),'至少一辆车没有从静止状态起步。');
end
