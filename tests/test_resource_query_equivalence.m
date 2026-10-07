classdef test_resource_query_equivalence < matlab.unittest.TestCase
    %TEST_RESOURCE_QUERY_EQUIVALENCE 验证批量稀疏矩形查询与完整栅格查询等价。
    % 输入：旋转、时间层端点、扫掠、停车、倒车与多归属等确定性轨迹。
    % 输出：公开 day6_resource_blocks 接口的碰撞布尔值和命中子集验收。
    % 逻辑：full collect 分支保持完整资源集；搜索分支允许首命中即返回，
    %       但每个返回块必须属于完整动态交集，且不能把有冲突误判为自由。
    % 不依赖历史备份目录，不比较私有辅助函数，不改变原工程配置。

    properties (TestParameter)
        resourceCase = struct( ...
            'rotatedRectangles',struct('name','rotation','expected',true), ...
            'halfOpenLayerEndpoint',struct('name','layer_endpoint','expected',true), ...
            'interiorSweepOnly',struct('name','sweep','expected',true), ...
            'parkedVehicleHold',struct('name','hold','expected',true), ...
            'sameSpaceDifferentTime',struct('name','time_separated','expected',false), ...
            'reverseSpeedMagnitude',struct('name','reverse','expected',true), ...
            'overlappingVehicleOwners',struct('name','owners','expected',true));
    end

    methods (TestClassSetup)
        function addSources(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'matlab','single_vehicle')));
        end
    end

    methods (Test)
        function sparseSearchMatchesFullDynamicIntersection(testCase,resourceCase)
            [states,map,footprint] = testCase.fixture(resourceCase.name);
            [fast,fast_outside] = day6_resource_blocks(states,map,footprint,true);
            [full,full_outside] = day6_resource_blocks(states,map,footprint,false);
            full_hits = full(map.dynamic_occupancy(full));
            testCase.verifyEqual(fast_outside,full_outside);
            testCase.verifyFalse(full_outside);
            testCase.verifyEqual(~isempty(fast),~isempty(full_hits));
            testCase.verifyEqual(~isempty(fast),resourceCase.expected);
            testCase.verifyTrue(all(ismember(fast,full_hits)));
            testCase.verifyTrue(all(map.dynamic_occupancy(fast)));
        end

        function exactTimeBoundaryEndpointChecksItsNewHalfOpenLayer(testCase)
            [states,map,footprint] = testCase.fixture('layer_endpoint');
            [hits,outside] = day6_resource_blocks(states,map,footprint,true);
            [~,~,layers] = ind2sub([map.nx,map.ny,map.nt],hits);
            testCase.verifyFalse(outside);
            testCase.verifyNotEmpty(hits);
            testCase.verifyEqual(unique(layers),2);
        end

        function sweepFindsConflictDespiteSafeRawEndpoints(testCase)
            [states,map,footprint] = testCase.fixture('sweep');
            [first,~] = day6_resource_blocks(states(1,:),map,footprint,true);
            [last,~] = day6_resource_blocks(states(end,:),map,footprint,true);
            [swept,~] = day6_resource_blocks(states,map,footprint,true);
            testCase.verifyEmpty(first);
            testCase.verifyEmpty(last);
            testCase.verifyNotEmpty(swept);
        end

        function sharedOwnersRemainInDynamicMaskAndFirstHitSubset(testCase)
            [states,map,footprint] = testCase.fixture('owners');
            [fast,~] = day6_resource_blocks(states,map,footprint,true);
            [full,~] = day6_resource_blocks(states,map,footprint,false);
            overlap_blocks = [map.overlaps.linear_index].';
            full_hits = full(map.dynamic_occupancy(full));
            testCase.verifyNotEmpty(overlap_blocks);
            testCase.verifyTrue(all(map.dynamic_occupancy(overlap_blocks)));
            testCase.verifyTrue(all(ismember(fast,full_hits)));
            testCase.verifyTrue(any(ismember(fast,overlap_blocks)));
        end

        function twentySeededFiniteSegmentsHaveEquivalentDecisions(testCase)
            comparisons = testCase.randomComparisons();
            testCase.verifySize(comparisons,[20,4]);
            testCase.verifyEqual(comparisons(:,1),comparisons(:,2));
            testCase.verifyTrue(all(comparisons(:,3)));
            testCase.verifyFalse(any(comparisons(:,4)));
            testCase.verifyTrue(any(comparisons(:,1)));
            testCase.verifyFalse(all(comparisons(:,1)));
        end

        function outsideTimeBoundsHaveSamePermanentFailure(testCase)
            [states,map,footprint] = testCase.fixture('rotation');
            states(end,5) = map.t_max;
            [fast,fast_outside] = day6_resource_blocks(states,map,footprint,true);
            [full,full_outside] = day6_resource_blocks(states,map,footprint,false);
            testCase.verifyTrue(fast_outside && full_outside);
            testCase.verifyEmpty(fast);
            testCase.verifyEmpty(full);
        end
    end

    methods (Static,Access=private)
        function [states,map,footprint] = fixture(name)
            planner = test_resource_query_equivalence.planner();
            vehicle = test_resource_query_equivalence.vehicle();
            high = [15,10,0,0,0;15,10,0,0,0.5];
            states = high;
            second = [];
            switch name
                case 'rotation'
                    high(:,3) = -pi/4;
                    states(:,3) = pi/4;
                case 'layer_endpoint'
                    high(:,5) = [0.5;1];
                case 'sweep'
                    states = [10,10,0,20,0;20,10,0,20,0.5];
                case 'hold'
                    planner.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
                    states(:,5) = [2;3];
                case 'time_separated'
                    states(:,5) = [2;3];
                case 'reverse'
                    % 公共 v 保持速度大小；后轴沿车身反向移动，不能用负速度。
                    states = [20,10,0,20,0;10,10,0,20,0.5];
                case 'owners'
                    second = high;
                    second(:,1) = second(:,1)+0.1;
                otherwise
                    error('ResourceQueryTest:InvalidFixture','未知测试场景。');
            end
            obstacle = dynamic_vehicle_obstacle(high,vehicle,'vehicle_high',planner);
            map = mark_trajectory_occupancy(st_occupancy_map(planner),high,vehicle,'vehicle_high');
            if ~isempty(second)
                map = mark_trajectory_occupancy(map,second,vehicle,'vehicle_second');
                obstacle(2) = dynamic_vehicle_obstacle(second,vehicle,'vehicle_second',planner);
            end
            map = prepare_dynamic_context(map,obstacle);
            footprint = inflate_vehicle_occupancy(vehicle,map.config.safety_distance_m);
        end

        function comparisons = randomComparisons()
            % 测试方法不含循环；辅助函数生成有限样本并恢复调用者随机数状态。
            previous = rng;
            restore_rng = onCleanup(@() rng(previous)); %#ok<NASGU>
            rng(20261007,'twister');
            planner = test_resource_query_equivalence.planner();
            vehicle = test_resource_query_equivalence.vehicle();
            footprint = inflate_vehicle_occupancy(vehicle,0);
            comparisons = false(20,4);
            for k = 1:20
                origin = [6+18*rand,4+12*rand,2*pi*rand-pi];
                target = origin+[4*rand-2,4*rand-2,rand-0.5];
                states = [origin,2*rand,0;target,2*rand,0.5];
                high = [origin,0,0;origin,0,0.5];
                if mod(k,2) == 0, high(:,5) = [1;1.5]; end
                obstacle = dynamic_vehicle_obstacle(high,vehicle,'vehicle_random',planner);
                map = prepare_dynamic_context(mark_trajectory_occupancy( ...
                    st_occupancy_map(planner),high,vehicle,'vehicle_random'),obstacle);
                [fast,fast_outside] = day6_resource_blocks(states,map,footprint,true);
                [full,full_outside] = day6_resource_blocks(states,map,footprint,false);
                full_hits = full(map.dynamic_occupancy(full));
                comparisons(k,:) = [~isempty(fast),~isempty(full_hits), ...
                    all(ismember(fast,full_hits)),fast_outside || full_outside];
            end
        end

        function planner = planner()
            root = fileparts(fileparts(mfilename('fullpath')));
            planner = jsondecode(fileread(fullfile(root,'config','planner_config.json')));
            planner.st_occupancy.x_min = 0; planner.st_occupancy.x_max = 30;
            planner.st_occupancy.y_min = 0; planner.st_occupancy.y_max = 20;
            planner.st_occupancy.t_min = 0; planner.st_occupancy.t_max = 4;
            planner.st_occupancy.dx = 0.5; planner.st_occupancy.dy = 0.5;
            planner.st_occupancy.dt = 0.5;
            planner.st_occupancy.safety_distance_m = 0;
            planner.st_occupancy.sweep_max_dt_s = 0.05;
            planner.st_occupancy.sweep_max_distance_m = 0.1;
            planner.st_occupancy.trajectory_end_policy = 'release';
            planner.vhybrid.time_step_s = 0.5;
            planner.vhybrid.time_grid_resolution_s = 0.5;
            planner.day6.goal_hold_enabled = false;
        end

        function vehicle = vehicle()
            vehicle = struct('dimensions_m',struct('width',0.4,'length',1.5, ...
                'wheelbase',0.6,'rear_axle_to_rear',0.5,'front_axle_to_front',0.4), ...
                'collision_margin_m',struct('front',0,'rear',0,'side',0), ...
                'steering',struct('maximum_steer_rad',0.4680017179));
        end
    end
end
