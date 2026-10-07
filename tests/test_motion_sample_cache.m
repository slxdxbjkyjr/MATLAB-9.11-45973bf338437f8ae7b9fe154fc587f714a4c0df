classdef test_motion_sample_cache < matlab.unittest.TestCase
    %TEST_MOTION_SAMPLE_CACHE 已验证模型样本复用必须保持原轨迹和约束结果。
    % 输入：同一首状态、控制和档位分别使用或不使用模型积分缓存。
    % 输出：逐采样时刻完整状态、档位和控制等价的测试结果。
    % 逻辑：改变静态/动态采样步长以覆盖完整命中、部分命中和未命中；
    %       不允许用插值填补任何时刻，也不允许缓存绕过运动约束。

    properties (TestParameter)
        motionCase = struct( ...
            'forwardAcceleration',struct('gear',1,'v',0,'a',2,'delta',0.2,'samples',10,'step',0.05), ...
            'reverseConstantSpeed',struct('gear',-1,'v',2,'a',0,'delta',-0.2,'samples',10,'step',0.05), ...
            'reverseBrakingDenseStatic',struct('gear',-1,'v',1,'a',-2,'delta',0.2,'samples',20,'step',0.05), ...
            'differentTimeStep',struct('gear',1,'v',1,'a',1,'delta',-0.2,'samples',10,'step',0.07), ...
            'differentStaticStep',struct('gear',-1,'v',1,'a',-2,'delta',0,'samples',7,'step',0.05));
    end

    methods (TestClassSetup)
        function addSources(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'matlab','single_vehicle')));
        end
    end

    methods (Test)
        function cachedAndUncachedMotionHaveIdenticalRequiredSamples(testCase,motionCase)
            planner = testCase.planner();
            planner.day6.collision_sample_step_s = motionCase.step;
            vehicle = testCase.vehicle();
            start = [10,10,0.1,motionCase.v,12.5];
            samples = testCase.integrateSamples(start,motionCase,vehicle,planner);
            expected = day6_motion_trajectory(start,samples(end,:),motionCase.delta, ...
                motionCase.a,vehicle,planner,motionCase.gear);
            actual = day6_motion_trajectory(start,samples(end,:),motionCase.delta, ...
                motionCase.a,vehicle,planner,motionCase.gear,samples);
            testCase.verifyEqual(day6_trajectory_states(actual),day6_trajectory_states(expected), ...
                'AbsTol',2e-12);
            testCase.verifyEqual(actual.gear,expected.gear);
            testCase.verifyEqual(actual.acceleration,expected.acceleration,'AbsTol',1e-12);
            testCase.verifyEqual(actual.steering_angle,expected.steering_angle,'AbsTol',1e-12);
            testCase.verifyEqual(actual.t,expected.t,'AbsTol',1e-12);
        end

        function partialCacheDoesNotRemoveMissingSamplingTimes(testCase)
            planner = testCase.planner();
            vehicle = testCase.vehicle();
            motion = struct('gear',1,'v',0,'a',2,'delta',0.2,'samples',2);
            start = [10,10,0,0,0];
            cache = testCase.integrateSamples(start,motion,vehicle,planner);
            expected = day6_motion_trajectory(start,cache(end,:),motion.delta,motion.a,vehicle,planner,1);
            actual = day6_motion_trajectory(start,cache(end,:),motion.delta,motion.a,vehicle,planner,1,cache);
            testCase.verifyGreaterThan(numel(actual.t),size(cache,1));
            testCase.verifyEqual(day6_trajectory_states(actual),day6_trajectory_states(expected), ...
                'AbsTol',2e-12);
        end

        function cacheCannotBypassIllegalSteeringConstraint(testCase)
            planner = testCase.planner();
            vehicle = testCase.vehicle();
            cache = [10,10,0,0,0;10,10,0,0,0.5];
            testCase.verifyError(@() day6_motion_trajectory(cache(1,:),cache(end,:), ...
                1,0,vehicle,planner,1,cache),'DynamicCollision:InvalidTrajectory');
        end

        function cacheCannotBypassIllegalGearConstraint(testCase)
            planner = testCase.planner();
            vehicle = testCase.vehicle();
            cache = [10,10,0,0,0;10,10,0,0,0.5];
            testCase.verifyError(@() day6_motion_trajectory(cache(1,:),cache(end,:), ...
                0,0,vehicle,planner,0,cache),'DynamicCollision:InvalidTrajectory');
        end

        function cacheCannotBypassZeroCrossingSpeedConstraint(testCase)
            planner = testCase.planner();
            vehicle = testCase.vehicle();
            cache = [10,10,0,0.25,0;10,10,0,0,0.5];
            testCase.verifyError(@() day6_motion_trajectory(cache(1,:),cache(end,:), ...
                0,-1,vehicle,planner,-1,cache),'DynamicCollision:InvalidTrajectory');
        end

        function malformedCachedStatesAreRejected(testCase)
            planner = testCase.planner();
            vehicle = testCase.vehicle();
            start = [10,10,0,0,0];
            last = [10,10,0,0,0.5];
            cache = [start;last]; cache(2,1) = NaN;
            testCase.verifyError(@() day6_motion_trajectory(start,last,0,0,vehicle,planner,1,cache), ...
                'DynamicCollision:InvalidTrajectory');
        end
    end

    methods (Static,Access=private)
        function states = integrateSamples(start,motion,vehicle,planner)
            % 缓存源仅用原模型适配器积分；辅助循环不复制运动学方程。
            states = zeros(motion.samples+1,5);
            states(1,:) = start;
            for k = 1:motion.samples
                [states(k+1,:),~,~,valid] = summon_vehicle_dynamic_gear(start,motion.delta, ...
                    motion.a,0.5*k/motion.samples,vehicle,planner,motion.gear);
                assert(valid,'MotionCacheTest:InvalidFixture','测试缓存必须来自有效模型积分。');
            end
        end

        function planner = planner()
            root = fileparts(fileparts(mfilename('fullpath')));
            planner = jsondecode(fileread(fullfile(root,'config','planner_config.json')));
            planner.dynamics.v_min_mps = 0; planner.dynamics.v_max_mps = 5;
            planner.dynamics.a_min_mps2 = -2; planner.dynamics.a_max_mps2 = 2;
            planner.day6.collision_sample_step_s = 0.05;
            planner.day6.collision_sample_step_m = 0.1;
        end

        function vehicle = vehicle()
            vehicle = struct('dimensions_m',struct('width',0.4,'length',1.5, ...
                'wheelbase',0.6,'rear_axle_to_rear',0.5,'front_axle_to_front',0.4), ...
                'collision_margin_m',struct('front',0,'rear',0,'side',0), ...
                'steering',struct('maximum_steer_rad',0.4680017179));
        end
    end
end
