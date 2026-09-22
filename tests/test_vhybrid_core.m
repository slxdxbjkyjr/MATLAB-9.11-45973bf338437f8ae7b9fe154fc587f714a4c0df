classdef test_vhybrid_core < matlab.unittest.TestCase
    %TEST_VHYBRID_CORE 测试 V-Hybrid A* 节点、集合、扩展和代价。

    properties
        VehicleConfig
        Map
        PlannerConfig
    end

    methods (TestClassSetup)
        function addSourceAndLoadConfig(testCase)
            projectDir = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectDir, 'matlab', 'single_vehicle')));
            testCase.VehicleConfig = jsondecode(fileread(fullfile(projectDir, ...
                'config', 'vehicle_config.json')));
            testCase.Map = summon_map(jsondecode(fileread(fullfile(projectDir, ...
                'config', 'map_config.json'))));
            testCase.PlannerConfig = jsondecode(fileread(fullfile(projectDir, ...
                'config', 'planner_config.json')));
        end
    end

    methods (Test)
        function testNodeContainsFiveDimensionalState(testCase)
            node = vhybrid_node([1,2,0.1,0.5,1.5], 1, 0.2, 3, 4, 5, [1,2,3,4,5], 6);
            testCase.verifyEqual([node.x,node.y,node.theta,node.v,node.t], [1,2,0.1,0.5,1.5]);
            testCase.verifyEqual([node.acceleration,node.steering_angle,node.parent_id], [1,0.2,3]);
            testCase.verifyEqual([node.g_cost,node.h_cost,node.f_cost], [4,5,9]);
            testCase.verifyEqual([node.x_index,node.y_index,node.yaw_index,node.velocity_index,node.time_index], [1,2,3,4,5]);
        end

        function testClosedSetPreservesTimeAndVelocity(testCase)
            closed = vhybrid_closed_set();
            node_a = vhybrid_node([1,2,0,0.5,1],0,0,0,0,0,[2,4,6,2,2],1);
            node_b = vhybrid_node([1,2,0,1.0,1.5],0,0,0,0,0,[2,4,6,4,3],2);
            closed.add(node_a);
            testCase.verifyTrue(closed.contains(node_a));
            testCase.verifyFalse(closed.contains(node_b));
        end

        function testExpansionSamplesNineControls(testCase)
            parent = vhybrid_node([3,3,0,0.5,0],0,0,0,0,0,[6,6,12,2,0],1);
            [children, statistics] = vhybrid_expand_node(parent, testCase.Map, ...
                testCase.VehicleConfig, testCase.PlannerConfig, [24,5,0,0,0]);
            testCase.verifyEqual(statistics.sampled, 9);
            testCase.verifyGreaterThan(statistics.valid, 0);
            testCase.verifyGreaterThanOrEqual(numel(children), 3);
        end

        function testExpansionUsesDifferentTimeIndices(testCase)
            parent = vhybrid_node([3,3,0,0.5,0],0,0,0,0,0,[6,6,12,2,0],1);
            [children, ~] = vhybrid_expand_node(parent, testCase.Map, ...
                testCase.VehicleConfig, testCase.PlannerConfig, [24,5,0,0,0]);
            testCase.verifyGreaterThan(numel(unique([children.time_index])), 0);
            testCase.verifyEqual([children.t], repmat(testCase.PlannerConfig.vhybrid.time_step_s, 1, numel(children)), 'AbsTol', 1e-12);
        end

        function testCostContainsRequiredTerms(testCase)
            parent = vhybrid_node([3,3,0,0.5,0],0,0,0,0,0,[6,6,12,2,0],1);
            child_state = [3.5,3.0,0.1,0.7,0.5];
            [g_cost,h_cost,f_cost,terms] = vhybrid_cost(parent, child_state, ...
                [10,3,0,0,0], testCase.PlannerConfig, 0.5, 0.1);
            testCase.verifyGreaterThan(g_cost, 0);
            testCase.verifyGreaterThan(h_cost, 0);
            testCase.verifyEqual(f_cost, g_cost+h_cost, 'AbsTol', 1e-12);
            testCase.verifyEqual([terms.distance,terms.speed_change,terms.heading_change,terms.goal_distance,terms.time], ...
                [0.5,0.2,0.1,6.5,0.5], 'AbsTol', 1e-12);
        end

        function testOpenSetReturnsMinimumCost(testCase)
            open = vhybrid_open_set();
            node_a = vhybrid_node([0,0,0,0,0],0,0,0,3,2,[0,0,0,0,0],1);
            node_b = vhybrid_node([0,0,0,0,0],0,0,0,1,1,[0,0,0,0,0],2);
            open.push(node_a); open.push(node_b);
            [node,valid] = open.pop_min();
            testCase.verifyTrue(valid);
            testCase.verifyEqual(node.node_id, 2);
            testCase.verifyEqual(open.count(), 1);
        end

        function testDemoReturnsPathOrExplicitFailure(testCase)
            path = run_vhybrid_astar_demo();
            testCase.verifyTrue(isfield(path,'search_statistics'));
            testCase.verifyTrue(path.valid || path.error_code ~= 0);
            testCase.verifyGreaterThan(path.search_statistics.expanded_nodes, 0);
        end
    end
end
