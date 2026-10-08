classdef test_reverse_search < matlab.unittest.TestCase
    %TEST_REVERSE_SEARCH 倒车搜索、挡位查重和完整换挡驻留的公开接口验收。
    % 输入为独立场景和配置副本，不修改项目JSON或绘制预设轨迹。
    % 输出为明确的逐项测试结果；测试轨迹由实际搜索器/连接器生成。
    methods (TestClassSetup)
        function addFunctions(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'matlab','single_vehicle')));
        end
    end
    methods (Test)
        function oldNodeInterfaceDefaultsToForward(testCase)
            node = vhybrid_node([10,10,0,0,0],0,0,0,0,0,zeros(1,5),1);
            testCase.verifyEqual(node.gear,1);
            testCase.verifyFalse(node.is_gear_change);
        end
        function openRetainsBothGearsAtZeroSpeed(testCase)
            forward = vhybrid_node([10,10,0,0,0],0,0,0,2,0,zeros(1,5),1,1);
            reverse = vhybrid_node([10,10,0,0,0],0,0,0,1,0,zeros(1,5),2,-1);
            queue = vhybrid_open_set();
            firstAccepted = queue.push_or_update(forward);
            secondAccepted = queue.push_or_update(reverse);
            [first,firstValid] = queue.pop_min();
            [second,secondValid] = queue.pop_min();
            testCase.verifyTrue(firstAccepted && secondAccepted && firstValid && secondValid);
            testCase.verifyEqual([first.gear,second.gear],[-1,1]);
        end
        function closedRetainsBothGearsAtZeroSpeed(testCase)
            forward = vhybrid_node([10,10,0,0,0],0,0,0,0,0,zeros(1,5),1,1);
            reverse = vhybrid_node([10,10,0,0,0],0,0,0,0,0,zeros(1,5),2,-1);
            closed = vhybrid_closed_set(); closed.add(forward);
            reverseBefore = closed.contains(reverse);
            closed.add(reverse);
            testCase.verifyFalse(reverseBefore);
            testCase.verifyTrue(closed.contains(forward));
            testCase.verifyTrue(closed.contains(reverse));
            testCase.verifyEqual(closed.count(),uint64(2));
        end
        function openTieBreakKeepsInsertionOrder(testCase)
            first = vhybrid_node([10,10,0,1,0],0,0,0,2,0,[1,0,0,0,0],20);
            second = vhybrid_node([11,10,0,1,0],0,0,0,2,0,[2,0,0,0,0],10);
            queue = vhybrid_open_set(); queue.push(first); queue.push(second);
            [a,validA] = queue.pop_min(); [b,validB] = queue.pop_min();
            testCase.verifyTrue(validA && validB);
            testCase.verifyEqual([a.node_id,b.node_id],[20,10]);
        end
        function reverseTravelHasConfiguredPenalty(testCase)
            planner = testCase.planner();
            parent = vhybrid_node([10,10,0,1,0],0,0,0,0,0,zeros(1,5),1);
            child = [10.5,10,0,1,.5]; goal = [20,10,0,0,0];
            [forwardG,~,~,forwardTerms] = vhybrid_cost(parent,child,goal,planner,.5,0,1,false);
            [reverseG,~,~,reverseTerms] = vhybrid_cost(parent,child,goal,planner,-.5,0,-1,false);
            testCase.verifyEqual(reverseG-forwardG,planner.vhybrid.weight_distance* ...
                .5*(planner.vhybrid.reverse_penalty-1),'AbsTol',1e-12);
            testCase.verifyGreaterThan(reverseG,forwardG);
            testCase.verifyEqual(forwardTerms.reverse_distance,0,'AbsTol',1e-12);
            testCase.verifyEqual(reverseTerms.reverse_distance,.5,'AbsTol',1e-12);
        end
        function gearSwitchAddsPenaltyAndWaitTime(testCase)
            planner = testCase.planner();
            parent = vhybrid_node([10,10,0,0,0],0,0,0,0,0,zeros(1,5),1);
            child = [10,10,0,0,.5]; goal = [5,10,0,0,0];
            [plainG,~,~,~] = vhybrid_cost(parent,child,goal,planner,0,0,1,false);
            [shiftG,~,~,terms] = vhybrid_cost(parent,child,goal,planner,0,0,-1,true);
            testCase.verifyEqual(shiftG-plainG,planner.vhybrid.gear_switch_penalty,'AbsTol',1e-12);
            testCase.verifyEqual(terms.time,planner.dynamics.direction_change_time_s,'AbsTol',1e-12);
        end
        function movingVehicleCannotChangeGear(testCase)
            planner = testCase.planner(); vehicle = testCase.vehicle();
            parent = vhybrid_node([10,10,0,1,0],0,0,1,0,0,zeros(1,5),2,1);
            [children,statistics] = vhybrid_expand_node(parent,testCase.openMap(), ...
                vehicle,planner,[20,10,0,0,0]);
            testCase.assertNotEmpty(children);
            testCase.verifyFalse(any([children.is_gear_change]));
            testCase.verifyEqual(unique([children.gear]),1);
            testCase.verifyEqual(statistics.gear_switch_nodes,0);
        end
        function stoppedVehicleGetsOneCompleteShiftWait(testCase)
            planner = testCase.planner(); vehicle = testCase.vehicle();
            parent = vhybrid_node([10,10,0,0,2],0,0,1,0,0,zeros(1,5),2,1);
            [children,statistics] = vhybrid_expand_node(parent,testCase.openMap(), ...
                vehicle,planner,[5,10,0,0,0]);
            shift = children([children.is_gear_change]);
            testCase.assertNumElements(shift,1);
            testCase.verifyEqual([shift.x,shift.y,shift.theta,shift.v],[10,10,0,0],'AbsTol',1e-12);
            testCase.verifyEqual(shift.t-parent.t,planner.dynamics.direction_change_time_s,'AbsTol',1e-12);
            testCase.verifyEqual(shift.gear,-1);
            testCase.verifyEqual(shift.direction,0);
            testCase.verifyEqual(statistics.gear_switch_nodes,1);
        end
        function collisionDuringShiftWaitRejectsSwitch(testCase)
            planner = testCase.planner(); vehicle = testCase.smallVehicle();
            planner.day6.goal_hold_enabled = false;
            planner.vhybrid.control_acceleration_samples_mps2 = 0;
            planner.vhybrid.control_steering_samples_rad = 0;
            parent = vhybrid_node([10,10,0,0,0],0,0,1,0,0,zeros(1,5),2,1);
            high = dynamic_vehicle_obstacle([10,8,pi/2,8,0;10,12,pi/2,0,.5], ...
                vehicle,'crossing_vehicle',planner);
            [children,statistics] = vhybrid_expand_node(parent,testCase.openMap(), ...
                vehicle,planner,[5,10,0,0,0],[],high);
            testCase.verifyEmpty(children);
            testCase.verifyEqual(statistics.gear_switch_pruned,1);
            testCase.verifyGreaterThan(statistics.dynamic_pruned,0);
        end
        function immediateStartDoesNotBeginWithShiftWait(testCase)
            planner = testCase.planner(); vehicle = testCase.smallVehicle();
            parent = vhybrid_node([10,10,0,0,0],0,0,0,0,0,zeros(1,5),1,1);
            high = dynamic_vehicle_obstacle([25,15,0,0,0;25,15,0,0,10], ...
                vehicle,'distant_vehicle',planner);
            [children,statistics] = vhybrid_expand_node(parent,testCase.openMap(), ...
                vehicle,planner,[20,10,0,0,0],[],high);
            testCase.assertNotEmpty(children);
            testCase.verifyFalse(any([children.is_gear_change]));
            testCase.verifyTrue(all([children.travelled_distance]>0));
            testCase.verifyEqual(statistics.gear_switch_nodes,0);
        end
        function fiveSteeringAnglesAreActuallySampled(testCase)
            planner = testCase.planner(); vehicle = testCase.smallVehicle();
            planner.vhybrid.control_acceleration_samples_mps2 = 0;
            planner.vhybrid.reverse_enabled = false;
            parent = vhybrid_node([10,10,0,1,0],0,0,1,0,0,zeros(1,5),2,1);
            [children,statistics] = vhybrid_expand_node(parent,testCase.openMap(), ...
                vehicle,planner,[20,10,0,0,0]);
            testCase.verifyEqual(statistics.sampled,5);
            testCase.verifyNumElements(unique([children.steering_angle]),5);
        end
        function confinedCorridorRequiresReverseAndStopsAtGoal(testCase)
            planner = testCase.planner(); vehicle = testCase.vehicle();
            planner.vhybrid.max_search_nodes = 200;
            planner.vhybrid.goal_connection_distance_m = 40;
            planner.vhybrid.control_steering_samples_rad = 0;
            forwardPlanner = planner; forwardPlanner.vhybrid.reverse_enabled = false;
            geometry = struct('boundary_xy',[0,0;20,0;20,2.6;0,2.6;0,0], ...
                'obstacles',{{}},'bounds',[0,0,20,2.6]);
            start = [15,1.3,0,0,0]; goal = [5,1.3,0,0,0];
            forwardPath = summon_vhybrid_astar(start,goal,geometry,vehicle,forwardPlanner);
            reversePath = summon_vhybrid_astar(start,goal,geometry,vehicle,planner);
            testCase.verifyFalse(forwardPath.valid);
            testCase.assertTrue(reversePath.valid);
            testCase.verifyTrue(any(reversePath.direction<0));
            testCase.verifyTrue(all(reversePath.v>=0));
            testCase.verifyLessThanOrEqual(max(reversePath.v),planner.vhybrid.max_reverse_speed_mps+1e-10);
            testCase.verifyEqual(reversePath.v(end),0,'AbsTol',1e-9);
            testCase.verifyEqual([reversePath.x(end),reversePath.y(end)],goal(1:2), ...
                'AbsTol',planner.vhybrid.goal_position_tolerance_m);
            testCase.verifyEqual(reversePath.search_statistics.path_gear_switches,1);
            testCase.verifyEqual(reversePath.search_statistics.path_gear_wait_time_s,.5,'AbsTol',1e-12);
            testCase.verifyTrue(reversePath.kinematic_validation.valid);
        end
        function searchRejectsControlsThatWouldCrossZeroSpeed(testCase)
            planner = testCase.planner(); vehicle = testCase.smallVehicle();
            planner.vhybrid.reverse_enabled = false;
            planner.vhybrid.goal_connection_enabled = false;
            planner.vhybrid.control_acceleration_samples_mps2 = -1;
            planner.vhybrid.control_steering_samples_rad = 0;
            planner.vhybrid.max_search_nodes = 10;
            path = summon_vhybrid_astar([10,10,0,.25,0], ...
                [10.03125,10,0,0,0],testCase.openMap(),vehicle,planner);
            % 原Day3模型禁止v+a*dt跨到负速，不以截断速度掩盖不可行动作。
            testCase.verifyFalse(path.valid);
            testCase.verifyNotEqual(path.error_code,uint8(0));
            testCase.verifyGreaterThan(path.search_statistics.constraint_pruned_nodes,0);
        end
        function curvedReverseConnectorUsesRealPhysicalYaw(testCase)
            planner = testCase.planner(); vehicle = testCase.vehicle();
            planner.vhybrid.goal_connection_distance_m = 100;
            geometry = struct('boundary_xy',[-50,-50;50,-50;50,50;-50,50;-50,-50], ...
                'obstacles',{{}},'bounds',[-50,-50,50,50]);
            start = [0,0,0,0,0]; goal = [-10,5,-pi/2,0,0];
            parent = vhybrid_node(start,0,0,0,0,0,zeros(1,5),1,-1);
            [connected,nodes,~] = vhybrid_goal_connection(parent,goal,geometry,vehicle,planner);
            testCase.assertTrue(connected);
            residual = testCase.replay(start,nodes,vehicle,planner);
            testCase.verifyEqual(unique([nodes.gear]),-1);
            testCase.verifyEqual(unique([nodes.direction]),-1);
            testCase.verifyEqual([nodes(end).x,nodes(end).y],goal(1:2), ...
                'AbsTol',planner.vhybrid.goal_position_tolerance_m);
            testCase.verifyEqual(mod(nodes(end).theta-goal(3)+pi,2*pi)-pi,0, ...
                'AbsTol',planner.vhybrid.goal_heading_tolerance_rad);
            testCase.verifyEqual(nodes(end).v,0,'AbsTol',1e-9);
            testCase.verifyLessThan(max(residual(:)),1e-8);
        end
    end
    methods (Static,Access=private)
        function planner = planner()
            root = fileparts(fileparts(mfilename('fullpath')));
            planner = jsondecode(fileread(fullfile(root,'config','planner_config.json')));
            planner.vhybrid.reverse_enabled = true;
            planner.vhybrid.reverse_penalty = 2;
            planner.vhybrid.gear_switch_penalty = 1;
            planner.vhybrid.max_reverse_speed_mps = 2;
            planner.vhybrid.initial_gear = 1;
            planner.vhybrid.control_steering_samples_rad = linspace(-planner.dynamics.delta_max_rad, ...
                planner.dynamics.delta_max_rad,5);
            planner.vhybrid.heuristic_method = 'euclidean';
            planner.vhybrid.heuristic_weight = 1;
            planner.vhybrid.max_search_time_s = 10;
        end
        function vehicle = vehicle()
            root = fileparts(fileparts(mfilename('fullpath')));
            vehicle = jsondecode(fileread(fullfile(root,'config','vehicle_config.json')));
        end
        function vehicle = smallVehicle()
            vehicle = struct('dimensions_m',struct('width',.4,'length',1.5, ...
                'wheelbase',.6,'rear_axle_to_rear',.5,'front_axle_to_front',.4), ...
                'collision_margin_m',struct('front',0,'rear',0,'side',0), ...
                'steering',struct('maximum_steer_rad',.4680017179));
        end
        function geometry = openMap()
            geometry = struct('boundary_xy',[0,0;30,0;30,20;0,20;0,0], ...
                'obstacles',{{}},'bounds',[0,0,30,20]);
        end
        function residuals = replay(start,nodes,vehicle,planner)
            % 辅助重放不复制自行车方程；测试方法本身保持 Arrange-Act-Assert。
            residuals = zeros(numel(nodes),5);
            previous = start;
            for k=1:numel(nodes)
                actual = [nodes(k).x,nodes(k).y,nodes(k).theta,nodes(k).v,nodes(k).t];
                [expected,~,~,valid] = summon_vehicle_dynamic_gear(previous, ...
                    nodes(k).steering_angle,nodes(k).acceleration,actual(5)-previous(5), ...
                    vehicle,planner,nodes(k).gear);
                assert(valid,'ReverseTest:InvalidFixture','连接节点的运动学输入必须有效。');
                delta = actual-expected; delta(3) = mod(delta(3)+pi,2*pi)-pi;
                residuals(k,:) = abs(delta); previous = actual;
            end
        end
    end
end
