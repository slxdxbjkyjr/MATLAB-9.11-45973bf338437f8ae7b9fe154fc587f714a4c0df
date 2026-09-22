classdef test_vehicle_dynamic < matlab.unittest.TestCase
    %TEST_VEHICLE_DYNAMIC 测试带速度和时间的单车运动学模型。

    methods (TestClassSetup)
        function addSourcePath(testCase)
            projectDir = fileparts(fileparts(mfilename('fullpath')));
            sourceDir = fullfile(projectDir, 'matlab', 'single_vehicle');
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(sourceDir));
        end
    end

    methods (Test)
        function testStraightAcceleration(testCase)
            state = [0, 0, 0, 1, 0];
            [next_state, distance, curvature, valid] = summon_vehicle_dynamic( ...
                state, 0, 1, 0.1, 2.78, 5, 0, -2, 2, 0.468);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(next_state(4), 1.1, 'AbsTol', 1e-12);
            testCase.verifyEqual(distance, 0.105, 'AbsTol', 1e-12);
            testCase.verifyEqual(curvature, 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(next_state(5), 0.1, 'AbsTol', 1e-12);
        end

        function testStraightDeceleration(testCase)
            state = [0, 0, 0, 2, 0];
            [next_state, distance, curvature, valid] = summon_vehicle_dynamic( ...
                state, 0, -1, 0.1, 2.78, 5, 0, -2, 2, 0.468);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(next_state(4), 1.9, 'AbsTol', 1e-12);
            testCase.verifyEqual(distance, 0.195, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(distance, 0);
            testCase.verifyEqual(curvature, 0, 'AbsTol', 1e-12);
        end

        function testLeftTurn(testCase)
            state = [0, 0, 0, 1, 0];
            [next_state, ~, curvature, valid] = summon_vehicle_dynamic( ...
                state, 0.2, 0, 0.1, 2.78, 5, 0, -2, 2, 0.468);
            testCase.verifyTrue(valid);
            testCase.verifyGreaterThan(curvature, 0);
            testCase.verifyGreaterThan(next_state(3), 0);
        end

        function testRightTurn(testCase)
            state = [0, 0, 0, 1, 0];
            [next_state, ~, curvature, valid] = summon_vehicle_dynamic( ...
                state, -0.2, 0, 0.1, 2.78, 5, 0, -2, 2, 0.468);
            testCase.verifyTrue(valid);
            testCase.verifyLessThan(curvature, 0);
            testCase.verifyLessThan(next_state(3), 0);
        end

        function testZeroSteering(testCase)
            state = [1, 2, 0.3, 1, 4];
            [next_state, distance, curvature, valid] = summon_vehicle_dynamic( ...
                state, 0, 0, 0.2, 2.78, 5, 0, -2, 2, 0.468);
            testCase.verifyTrue(valid);
            testCase.verifyEqual(curvature, 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(next_state(1), state(1) + distance*cos(state(3)), 'AbsTol', 1e-12);
            testCase.verifyEqual(next_state(2), state(2) + distance*sin(state(3)), 'AbsTol', 1e-12);
            testCase.verifyEqual(next_state(5), 4.2, 'AbsTol', 1e-12);
        end

        function testSpeedLimitViolation(testCase)
            state = [0, 0, 0, 4.9, 0];
            [next_state, distance, curvature, valid] = summon_vehicle_dynamic( ...
                state, 0, 2, 0.1, 2.78, 5, 0, -2, 2, 0.468);
            testCase.verifyFalse(valid);
            testCase.verifyEqual(next_state, state, 'AbsTol', 1e-12);
            testCase.verifyEqual(distance, 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(curvature, 0, 'AbsTol', 1e-12);
        end

        function testSteeringLimitViolation(testCase)
            state = [0, 0, 0, 1, 0];
            [next_state, distance, curvature, valid] = summon_vehicle_dynamic( ...
                state, 0.5, 0, 0.1, 2.78, 5, 0, -2, 2, 0.468);
            testCase.verifyFalse(valid);
            testCase.verifyEqual(next_state, state, 'AbsTol', 1e-12);
            testCase.verifyEqual(distance, 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(curvature, 0, 'AbsTol', 1e-12);
        end

        function testAccelerationLimitViolation(testCase)
            state = [0, 0, 0, 1, 0];
            [next_state, ~, ~, valid] = summon_vehicle_dynamic( ...
                state, 0, 3, 0.1, 2.78, 5, 0, -2, 2, 0.468);
            testCase.verifyFalse(valid);
            testCase.verifyEqual(next_state, state, 'AbsTol', 1e-12);
        end
    end
end
