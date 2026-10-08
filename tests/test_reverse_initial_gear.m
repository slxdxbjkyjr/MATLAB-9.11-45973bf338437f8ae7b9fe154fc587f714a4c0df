classdef test_reverse_initial_gear < matlab.unittest.TestCase
    %TEST_REVERSE_INITIAL_GEAR 起步挡位选择及原始起点不可行的公开接口验收。
    % 输入：独立窄通道、停车动态障碍及配置副本，状态单位为m/rad/m·s^-1/s。
    % 输出：真实搜索路径、起步挡位统计和明确初始冲突诊断；不修改工程JSON。
    % 静止车辆可在t=0之前选定起步挡位；运动中的换挡仍须停车并等待0.5s。

    properties (TestParameter)
        invalidMode = struct('unknown','choose_fastest','numeric',2,'empty','');
    end

    methods (TestClassSetup)
        function addFunctions(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'matlab','single_vehicle')));
        end
    end

    methods (Test)
        function autoReverseStartsImmediatelyWithoutArtificialShift(testCase)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.corridorFixture();
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            testCase.assertTrue(path.valid);
            [collision,report] = check_trajectory_conflict(path,obstacle,vehicle,planner,map);
            testCase.verifyEqual(path.gear(1),-1);
            testCase.verifyEqual(path.selected_initial_gear,-1);
            testCase.verifyEqual(path.search_statistics.initial_gears,[1,-1]);
            testCase.verifyEqual(path.search_statistics.initial_gear_mode,'auto');
            testCase.verifyEqual(path.t(1),0,'AbsTol',1e-12);
            testCase.verifyEqual(path.v(1),0,'AbsTol',1e-12);
            testCase.verifyGreaterThan(path.v(2),0);
            testCase.verifyLessThan(path.x(2),path.x(1));
            testCase.verifyGreaterThan(path.acceleration(2),0);
            testCase.verifyTrue(all(path.gear == -1));
            testCase.verifyFalse(any(path.is_gear_change));
            testCase.verifyEqual(path.search_statistics.path_gear_switches,0);
            testCase.verifyEqual(path.search_statistics.path_gear_wait_time_s,0,'AbsTol',1e-12);
            testCase.verifyEqual(path.v(end),0,'AbsTol',1e-9);
            testCase.verifyTrue(path.kinematic_validation.valid);
            testCase.verifyFalse(collision);
            testCase.verifyEmpty(report.resource_indices);
        end

        function fixedForwardCannotUseTheOtherRoot(testCase)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.corridorFixture();
            planner.vhybrid.initial_gear_mode = 'fixed';
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            testCase.verifyFalse(path.valid);
            testCase.verifyNotEqual(path.error_code,uint8(0));
            testCase.verifyEqual(path.search_statistics.initial_gears,1);
            testCase.verifyEqual(path.search_statistics.initial_gear_mode,'fixed');
        end

        function explicitForwardOverrideLocksAnAutoConfiguration(testCase)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.corridorFixture();
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle, ...
                struct('initial_gear',1));
            testCase.verifyFalse(path.valid);
            testCase.verifyEqual(path.search_statistics.initial_gears,1);
            testCase.verifyEqual(path.search_statistics.initial_gear_mode,'fixed');
        end

        function explicitReverseOverrideUsesOnlyTheReverseRoot(testCase)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.corridorFixture();
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle, ...
                struct('initial_gear',-1));
            testCase.assertTrue(path.valid);
            testCase.verifyEqual(path.search_statistics.initial_gears,-1);
            testCase.verifyEqual(path.gear(1),-1);
            testCase.verifyEqual(path.search_statistics.initial_gear_mode,'fixed');
            testCase.verifyEqual(path.search_statistics.path_gear_switches,0);
        end

        function movingRootCannotSelectTheOppositeGear(testCase)
            [planner,vehicle,geometry,map,obstacle,start,~] = testCase.corridorFixture();
            start = [12,start(2:3),1,0];
            goal = [15,start(2:3),0,0];
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            testCase.assertTrue(path.valid);
            testCase.verifyEqual(path.search_statistics.initial_gears,1);
            testCase.verifyEqual(path.gear(1),1);
            testCase.verifyEqual(path.selected_initial_gear,1);
            testCase.verifyEqual(path.v(1),1,'AbsTol',1e-12);
        end

        function tinyMovingRootStillCannotChooseTheOppositeGear(testCase)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.corridorFixture();
            start(4) = 1e-7;
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            % 终点速度容差只用于停车验收，不能让仍在运动的起点瞬间改变挡位。
            testCase.verifyLessThan(start(4),planner.vhybrid.goal_speed_tolerance_mps);
            testCase.verifyGreaterThan(start(4),0);
            testCase.verifyEqual(path.search_statistics.initial_gear_mode,'auto');
            testCase.verifyEqual(path.search_statistics.initial_gears,1);
        end

        function bothAutoRootsCountAgainstTheNodeBudget(testCase)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.corridorFixture();
            planner.vhybrid.max_search_nodes = 1;
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            testCase.verifyFalse(path.valid);
            testCase.verifyEqual(path.error_code,uint8(5));
            testCase.verifyEqual(path.search_statistics.initial_gears,[1,-1]);
            testCase.verifyEqual(path.search_statistics.expanded_nodes,0);
            testCase.verifyEqual(path.search_statistics.sampled_controls,0);
        end

        function reverseDisabledKeepsOnlyTheForwardRoot(testCase)
            [planner,vehicle,geometry,map,obstacle,start,~] = testCase.corridorFixture();
            planner.vhybrid.reverse_enabled = false;
            start = [8,start(2:5)];
            goal = [15,start(2:3),0,0];
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            testCase.assertTrue(path.valid);
            testCase.verifyEqual(path.search_statistics.initial_gears,1);
            testCase.verifyEqual(path.gear(1),1);
            testCase.verifyFalse(any(path.gear < 0));
        end

        function invalidInitialGearModeReturnsInputError(testCase,invalidMode)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.corridorFixture();
            planner.vhybrid.initial_gear_mode = invalidMode;
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            testCase.verifyFalse(path.valid);
            testCase.verifyEqual(path.error_code,uint8(1));
        end

        function originalStartsHaveSafetyOverlapEvenWithAutoReverse(testCase)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.originalStartFixture(15);
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            conflict = path.failure_diagnostics.initial_conflict;
            testCase.verifyFalse(path.valid);
            testCase.verifyEqual(path.error_code,uint8(9));
            testCase.verifyEqual(path.search_statistics.expanded_nodes,0);
            testCase.verifyEqual(path.search_statistics.sampled_controls,0);
            testCase.verifyEqual(path.failure_diagnostics.reason,'dynamic_start_collision');
            testCase.verifyTrue(conflict.resource_conflict);
            testCase.verifyTrue(conflict.safety_rectangle_collision);
            testCase.verifyFalse(conflict.physical_collision);
            testCase.verifyEqual(conflict.minimum_distance_m,.145,'AbsTol',1e-9);
            testCase.verifyEqual(conflict.obstacle_id,'vehicle_002');
            testCase.verifyEqual(conflict.conflict_time_s,0,'AbsTol',1e-12);
            testCase.verifyGreaterThanOrEqual(conflict.resource_block_count,20);
            testCase.verifyEmpty(path.failure_diagnostics.adjustable_entry_times_s);
        end

        function sharedCoarseBlocksRemainRejectedWithoutRectangleOverlap(testCase)
            [planner,vehicle,geometry,map,obstacle,start,goal] = testCase.originalStartFixture(14.6);
            path = summon_vhybrid_astar(start,goal,geometry,vehicle,planner,map,obstacle);
            conflict = path.failure_diagnostics.initial_conflict;
            testCase.verifyFalse(path.valid);
            testCase.verifyEqual(path.error_code,uint8(9));
            testCase.verifyTrue(conflict.resource_conflict);
            testCase.verifyFalse(conflict.safety_rectangle_collision);
            testCase.verifyFalse(conflict.physical_collision);
            testCase.verifyGreaterThan(conflict.resource_block_count,0);
            testCase.verifyGreaterThan(conflict.minimum_distance_m,.4);
        end

        function instantQueryDoesNotInflateFromFutureVehicleSpeed(testCase)
            [planner,vehicle,geometry,~,~,start,~] = testCase.originalStartFixture(15);
            high = [27,17,pi,0,0;26.75,17,pi,1,.5];
            obstacle = dynamic_vehicle_obstacle(high,vehicle,'vehicle_002',planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner,geometry), ...
                high,vehicle,'vehicle_002');
            [collision,report] = check_trajectory_conflict(start,obstacle,vehicle,planner,map);
            % 单一时刻无需覆盖时间采样间隙；未来速度不应降低当前净距下界。
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.resource_conflict);
            testCase.verifyTrue(report.rectangle_collision);
            testCase.verifyFalse(report.physical_collision);
            testCase.verifyEqual(report.minimum_distance_m,.145,'AbsTol',1e-9);
            testCase.verifyEqual(report.minimum_distance_lower_bound_m,.145,'AbsTol',1e-9);
            testCase.verifyEqual(report.checked_samples,1);
        end
    end

    methods (Static,Access=private)
        function [planner,vehicle,geometry,map,obstacle,start,goal] = corridorFixture()
            % 窄通道不容掉头；高车停在左端且与低车完整倒车路径保持安全距离。
            % 仅正常扩展采样正加速度，使固定前进根不能靠先前进再停车换挡绕过测试。
            planner = test_reverse_initial_gear.planner();
            vehicle = test_reverse_initial_gear.vehicle();
            geometry = struct('boundary_xy',[0,0;20,0;20,2.6;0,2.6;0,0], ...
                'obstacles',{{}},'bounds',[0,0,20,2.6]);
            planner.st_occupancy.x_max = 20;
            planner.st_occupancy.y_max = 2.6;
            planner.st_occupancy.dy = .2;
            high = [2,1.3,0,0,0;2,1.3,0,0,19.9];
            obstacle = dynamic_vehicle_obstacle(high,vehicle,'parked_vehicle',planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner,geometry), ...
                high,vehicle,'parked_vehicle');
            start = [15,1.3,0,0,0];
            goal = [8,1.3,0,0,0];
        end

        function [planner,vehicle,geometry,map,obstacle,start,goal] = originalStartFixture(start_y)
            % 原场景两车车身实际净距0.145m，但各侧0.2m裕度产生0.255m重叠。
            % y=14.6的附加案例只产生粗资源块共享，用于区分两类初始不可行。
            planner = test_reverse_initial_gear.planner();
            vehicle = test_reverse_initial_gear.vehicle();
            geometry = struct('boundary_xy',[0,0;30,0;30,20;0,20;0,0], ...
                'obstacles',{{}},'bounds',[0,0,30,20]);
            high = [27,17,pi,0,0;27,17,pi,0,.5];
            obstacle = dynamic_vehicle_obstacle(high,vehicle,'vehicle_002',planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner,geometry), ...
                high,vehicle,'vehicle_002');
            start = [24,start_y,0,0,0];
            goal = [4,17,0,0,0];
        end

        function planner = planner()
            % 所有相关参数显式覆盖，测试不依赖当前用户场景和搜索预算。
            root = fileparts(fileparts(mfilename('fullpath')));
            planner = jsondecode(fileread(fullfile(root,'config','planner_config.json')));
            planner.vhybrid.initial_gear_mode = 'auto';
            planner.vhybrid.initial_gear = 1;
            planner.vhybrid.reverse_enabled = true;
            planner.vhybrid.max_reverse_speed_mps = 2;
            planner.vhybrid.reverse_penalty = 2;
            % 7m解析连接以0.1m细分，预算必须覆盖整段70多个连接节点与起点。
            planner.vhybrid.max_search_nodes = 200;
            planner.vhybrid.max_search_time_s = 5;
            planner.vhybrid.goal_connection_enabled = true;
            planner.vhybrid.goal_speed_tolerance_mps = 1e-6;
            planner.vhybrid.goal_connection_distance_m = 40;
            planner.vhybrid.control_steering_samples_rad = 0;
            planner.vhybrid.control_acceleration_samples_mps2 = 2;
            planner.vhybrid.heuristic_method = 'euclidean';
            planner.vhybrid.heuristic_weight = 1;
            planner.vhybrid.reference_speed_mps = 2;
            planner.vhybrid.goal_connection_speed_samples_mps = [0,.5,1,1.5];
            planner.vhybrid.time_step_s = .5;
            planner.vhybrid.time_grid_resolution_s = .5;
            planner.dynamics.direction_change_time_s = .5;
            planner.st_occupancy.x_min = 0; planner.st_occupancy.x_max = 30;
            planner.st_occupancy.y_min = 0; planner.st_occupancy.y_max = 20;
            planner.st_occupancy.t_min = 0; planner.st_occupancy.t_max = 20;
            planner.st_occupancy.dx = .5; planner.st_occupancy.dy = .5;
            planner.st_occupancy.dt = .5;
            planner.st_occupancy.safety_distance_m = 0;
            planner.st_occupancy.trajectory_end_policy = 'release';
            planner.day6.minimum_safety_distance_m = 0;
            planner.day6.require_immediate_start = true;
            planner.day6.goal_hold_enabled = false;
        end

        function vehicle = vehicle()
            % 使用原车型尺寸与原0.2m裕度，不人为缩小车辆来通过倒车测试。
            root = fileparts(fileparts(mfilename('fullpath')));
            vehicle = jsondecode(fileread(fullfile(root,'config','vehicle_config.json')));
        end
    end
end
