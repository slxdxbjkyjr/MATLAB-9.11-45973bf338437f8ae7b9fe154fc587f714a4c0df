classdef test_reverse_motion < matlab.unittest.TestCase
    %TEST_REVERSE_MOTION 倒车运动学、档位语义和换挡驻留的公开接口测试。
    % 输入：小型测试车、集中配置和可复现控制/轨迹；单位 m、rad、s。
    % 输出：matlab.unittest 测试结果；不写场景或改原 Day3 模型。
    % 逻辑：速度大小始终非负，档位决定运动方向，停稳等待期间仍检查碰撞。

    properties (TestParameter)
        turningCase = struct( ...
            'leftSteering',struct('delta',0.2,'heading_sign',-1,'lateral_sign',1), ...
            'rightSteering',struct('delta',-0.2,'heading_sign',1,'lateral_sign',-1));
    end

    methods (TestClassSetup)
        function addSources(testCase)
            project = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(project,'matlab','single_vehicle')));
        end
    end

    methods (Test)
        function forwardAdapterMatchesOriginalModel(testCase)
            planner = testCase.planner();
            vehicle = testCase.vehicle();
            dyn = planner.dynamics;
            start = [10,10,0.1,1,0];
            [actual,ds,curvature,valid] = summon_vehicle_dynamic_gear( ...
                start,0.2,1,0.5,vehicle,planner,1);
            [expected,expected_ds,expected_curvature,expected_valid] = ...
                summon_vehicle_dynamic(start,0.2,1,0.5,vehicle.dimensions_m.wheelbase, ...
                dyn.v_max_mps,dyn.v_min_mps,dyn.a_min_mps2,dyn.a_max_mps2,dyn.delta_max_rad);
            testCase.verifyTrue(valid && expected_valid);
            testCase.verifyEqual(actual,expected,'AbsTol',1e-12);
            testCase.verifyEqual(ds,expected_ds,'AbsTol',1e-12);
            testCase.verifyEqual(curvature,expected_curvature,'AbsTol',1e-12);
        end

        function reverseAcceleratesFromRestWithoutFlippingBodyHeading(testCase)
            [state,ds,curvature,valid] = summon_vehicle_dynamic_gear( ...
                [10,10,0,0,0],0,2,0.5,testCase.vehicle(),testCase.planner(),-1);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(state,[9.75,10,0,1,0.5],'AbsTol',1e-12);
            testCase.verifyEqual(ds,-0.25,'AbsTol',1e-12);
            testCase.verifyEqual(curvature,0,'AbsTol',1e-12);
        end

        function reverseTurnUsesBodyHeadingAndSignedYawRate(testCase,turningCase)
            vehicle = testCase.vehicle();
            [state,ds,curvature,valid] = summon_vehicle_dynamic_gear( ...
                [10,10,0,1,0],turningCase.delta,0,0.5,vehicle,testCase.planner(),-1);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(ds,-0.5,'AbsTol',1e-12);
            testCase.verifyEqual(curvature,tan(turningCase.delta)/vehicle.dimensions_m.wheelbase, ...
                'AbsTol',1e-12);
            testCase.verifyEqual(state(3),ds*curvature,'AbsTol',1e-12);
            testCase.verifyGreaterThan(state(3)*turningCase.heading_sign,0);
            testCase.verifyLessThan(state(1),10);
            testCase.verifyGreaterThan((state(2)-10)*turningCase.lateral_sign,0);
            testCase.verifyEqual(state(4),1,'AbsTol',1e-12);
        end

        function reverseBrakingStopsWithNonnegativeSpeed(testCase)
            [state,ds,~,valid] = summon_vehicle_dynamic_gear( ...
                [10,10,0,1,0],0,-2,0.5,testCase.vehicle(),testCase.planner(),-1);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(state,[9.75,10,0,0,0.5],'AbsTol',1e-12);
            testCase.verifyEqual(ds,-0.25,'AbsTol',1e-12);
        end

        function reverseCannotCrossZeroSpeedByBraking(testCase)
            start = [10,10,0,0.5,0];
            [state,~,~,valid] = summon_vehicle_dynamic_gear( ...
                start,0,-2,0.5,testCase.vehicle(),testCase.planner(),-1);
            testCase.verifyFalse(valid);
            testCase.verifyEqual(state,start,'AbsTol',1e-12);
        end

        function invalidGearIsRejected(testCase)
            start = [10,10,0,0,0];
            [state,~,~,valid] = summon_vehicle_dynamic_gear( ...
                start,0,2,0.5,testCase.vehicle(),testCase.planner(),0);
            testCase.verifyFalse(valid);
            testCase.verifyEqual(state,start,'AbsTol',1e-12);
        end

        function optionalReverseSpeedLimitRejectsFasterReverseAction(testCase)
            planner = testCase.planner();
            planner.vhybrid.max_reverse_speed_mps = 2;
            start = [10,10,0,2,0];
            [state,~,~,valid] = summon_vehicle_dynamic_gear( ...
                start,0,1,0.5,testCase.vehicle(),planner,-1);
            testCase.verifyFalse(valid);
            testCase.verifyEqual(state,start,'AbsTol',1e-12);
        end

        function optionalReverseSpeedLimitDoesNotRestrictForwardAction(testCase)
            planner = testCase.planner();
            planner.vhybrid.max_reverse_speed_mps = 2;
            [state,ds,~,valid] = summon_vehicle_dynamic_gear( ...
                [10,10,0,2,0],0,1,0.5,testCase.vehicle(),planner,1);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(state(4),2.5,'AbsTol',1e-12);
            testCase.verifyEqual(ds,1.125,'AbsTol',1e-12);
        end

        function optionalReverseSpeedLimitRejectsAlreadyTooFastInput(testCase)
            planner = testCase.planner();
            planner.vhybrid.max_reverse_speed_mps = 2;
            start = [10,10,0,3,0];
            [state,~,~,valid] = summon_vehicle_dynamic_gear( ...
                start,0,-2,0.5,testCase.vehicle(),planner,-1);
            testCase.verifyFalse(valid);
            testCase.verifyEqual(state,start,'AbsTol',1e-12);
        end

        function reverseMotionCarriesGearAndUsesSameModelAtMidpoint(testCase)
            vehicle = testCase.vehicle();
            planner = testCase.planner();
            start = [10,10,0,0,0];
            [last,~,~,valid] = summon_vehicle_dynamic_gear(start,0.2,2,0.5,vehicle,planner,-1);
            trajectory = day6_motion_trajectory(start,last,0.2,2,vehicle,planner,-1);
            [expected,~,~,~] = summon_vehicle_dynamic_gear(start,0.2,2,0.25,vehicle,planner,-1);
            states = day6_trajectory_states(trajectory);
            middle = states(abs(states(:,5)-0.25)<1e-12,:);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(trajectory.gear,-ones(size(trajectory.t)));
            testCase.verifyGreaterThanOrEqual(min(trajectory.v),0);
            testCase.verifyEqual(middle,expected,'AbsTol',1e-12);
            testCase.verifyEqual(states(end,:),last,'AbsTol',1e-12);
        end

        function publicTrajectoryRejectsInvalidGearVector(testCase)
            path = testCase.stationaryShift(0.5);
            path.gear = [1;0];
            testCase.verifyError(@() day6_trajectory_states(path), ...
                'DynamicCollision:InvalidTrajectory');
        end

        function gearChangeIsValidOnlyAfterHalfSecondAtRest(testCase)
            report = vhybrid_validate_trajectory(testCase.stationaryShift(0.5), ...
                testCase.geometry(),testCase.vehicle(),testCase.planner());
            testCase.verifyTrue(report.valid);
            testCase.verifyEqual(report.gear_change_count,1);
            testCase.verifyEqual(report.gear_change_failures,0);
            testCase.verifyEqual(report.minimum_gear_change_wait_s,0.5,'AbsTol',1e-12);
        end

        function shorterGearChangeWaitIsRejected(testCase)
            report = vhybrid_validate_trajectory(testCase.stationaryShift(0.25), ...
                testCase.geometry(),testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.gear_change_failures,1);
        end

        function movingGearChangeIsRejectedEvenWithCorrectReverseEndpoint(testCase)
            path = testCase.stationaryShift(0.5);
            path.v = [1;1];
            path.x = [10;9.5];
            report = vhybrid_validate_trajectory(path,testCase.geometry(), ...
                testCase.vehicle(),testCase.planner());
            testCase.verifyFalse(report.valid);
            testCase.verifyEqual(report.gear_change_failures,1);
        end

        function sampledStationaryWaitAccumulatesBeforeGearChange(testCase)
            path = testCase.stationaryShift(0.5);
            path.x = [10;10;10]; path.y = [10;10;10];
            path.theta = [0;0;0]; path.v = [0;0;0]; path.t = [0;0.25;0.5];
            path.acceleration = [0;0;0]; path.steering_angle = [0;0;0];
            path.gear = [1;1;-1]; path.is_gear_change = [false;false;true];
            report = vhybrid_validate_trajectory(path,testCase.geometry(), ...
                testCase.vehicle(),testCase.planner());
            testCase.verifyTrue(report.valid);
            testCase.verifyEqual(report.minimum_gear_change_wait_s,0.5,'AbsTol',1e-12);
        end

        function stationaryGearChangeStillChecksIntermediateDynamicCollision(testCase)
            planner = testCase.planner();
            vehicle = testCase.tinyVehicle();
            obstacle = dynamic_vehicle_obstacle( ...
                [10,9,pi/2,4,0;10,11,pi/2,0,0.5],vehicle,'crossing_vehicle',planner);
            [collision,report] = check_trajectory_conflict(testCase.stationaryShift(0.5), ...
                obstacle,vehicle,planner);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.has_intermediate_conflict);
            testCase.verifyTrue(report.physical_collision);
            testCase.verifyGreaterThan(report.conflict_time_s,0);
            testCase.verifyLessThan(report.conflict_time_s,0.5);
        end

        function reverseTrajectoryReplaysDuringConflictChecks(testCase)
            planner = testCase.planner();
            vehicle = testCase.tinyVehicle();
            start = [10,10,0,0,0];
            [last,~,~,~] = summon_vehicle_dynamic_gear(start,0,2,0.5,vehicle,planner,-1);
            trajectory = day6_motion_trajectory(start,last,0,2,vehicle,planner,-1);
            obstacle = dynamic_vehicle_obstacle( ...
                [9.6,10,0,0,0;9.6,10,0,0,0.5],vehicle,'behind_vehicle',planner);
            [collision,report] = check_trajectory_conflict(trajectory,obstacle,vehicle,planner);
            testCase.verifyTrue(collision);
            testCase.verifyTrue(report.physical_collision);
            testCase.verifyGreaterThan(report.checked_samples,2);
        end

        function reverseTrajectoryPassesKinematicValidation(testCase)
            planner = testCase.planner();
            vehicle = testCase.vehicle();
            start = [10,10,0,1,0];
            [last,~,~,~] = summon_vehicle_dynamic_gear(start,-0.2,-2,0.5,vehicle,planner,-1);
            path = day6_motion_trajectory(start,last,-0.2,-2,vehicle,planner,-1);
            report = vhybrid_validate_trajectory(path,testCase.geometry(),vehicle,planner);
            testCase.verifyTrue(report.valid);
            testCase.verifyLessThan(report.max_position_residual_m,1e-10);
            testCase.verifyLessThan(report.max_heading_residual_rad,1e-10);
            testCase.verifyEqual(path.v(end),0,'AbsTol',1e-12);
        end
    end

    methods (Static,Access=private)
        function planner = planner()
            root = fileparts(fileparts(mfilename('fullpath')));
            planner = jsondecode(fileread(fullfile(root,'config','planner_config.json')));
            planner.dynamics.direction_change_time_s = 0.5;
            planner.dynamics.v_min_mps = 0;
            planner.dynamics.v_max_mps = 5;
            planner.dynamics.a_min_mps2 = -2;
            planner.dynamics.a_max_mps2 = 2;
            planner.day6.collision_sample_step_s = 0.05;
            planner.day6.collision_sample_step_m = 0.1;
            planner.day6.minimum_safety_distance_m = 0;
            planner.day6.goal_hold_enabled = true;
        end

        function vehicle = vehicle()
            vehicle = struct('dimensions_m',struct('width',0.4,'length',1.5, ...
                'wheelbase',0.6,'rear_axle_to_rear',0.5,'front_axle_to_front',0.4), ...
                'collision_margin_m',struct('front',0,'rear',0,'side',0), ...
                'steering',struct('maximum_steer_rad',0.4680017179));
        end

        function vehicle = tinyVehicle()
            vehicle = test_reverse_motion.vehicle();
            vehicle.dimensions_m = struct('width',0.1,'length',0.2, ...
                'wheelbase',0.06,'rear_axle_to_rear',0.1,'front_axle_to_front',0.04);
        end

        function map = geometry()
            map = struct('boundary_xy',[0,0;30,0;30,20;0,20;0,0], ...
                'obstacles',{{}},'bounds',[0,0,30,20]);
        end

        function path = stationaryShift(wait)
            % gear(k) 是该点档位；is_gear_change(k) 表示前一边为驻留换挡边。
            path = struct('x',[10;10],'y',[10;10],'theta',[0;0],'v',[0;0], ...
                't',[0;wait],'acceleration',[0;0],'steering_angle',[0;0], ...
                'gear',[1;-1],'is_gear_change',[false;true],'valid',true);
        end
    end
end
