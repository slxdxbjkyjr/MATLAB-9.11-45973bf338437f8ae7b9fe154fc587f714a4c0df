classdef test_day6_performance_equivalence < matlab.unittest.TestCase
    %TEST_DAY6_PERFORMANCE_EQUIVALENCE 验证运行效率优化保留原几何和归属判据。
    % 输入为独立地图/车型副本；输出为逐项等价与安全边界测试结果。
    % 不用运行时间阈值判断正确性；效率数据由单独的benchmark生成。
    properties (TestParameter)
        poseCase = struct( ...
            'free',struct('pose',[5,5,0],'hit',false), ...
            'overlap',struct('pose',[12.5,10,0],'hit',true), ...
            'edgeContact',struct('pose',[12,10,0],'hit',true), ...
            'outside',struct('pose',[29.8,10,0],'hit',true), ...
            'rotated',struct('pose',[13,8.5,pi/2],'hit',true));
    end
    methods (TestClassSetup)
        function addFunctions(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'matlab','single_vehicle')));
        end
    end
    methods (Test)
        function cachedStaticResultMatchesUncached(testCase,poseCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry();
            geometry.obstacles = {[13,9;15,9;15,11;13,11;13,9]};
            cached = prepare_static_context(geometry,vehicle);
            [rawHit,rawDetail] = day6_static_collision_check(poseCase.pose,geometry,vehicle);
            [cachedHit,cachedDetail] = day6_static_collision_check(poseCase.pose,cached,vehicle);
            testCase.verifyEqual(cachedHit,rawHit);
            testCase.verifyEqual(cachedHit,poseCase.hit);
            testCase.verifyEqual(cachedDetail.type,rawDetail.type);
            testCase.verifyEqual(cachedDetail.obstacle_index,rawDetail.obstacle_index);
            testCase.verifyEqual(cachedDetail.corners_xy,rawDetail.corners_xy,'AbsTol',1e-12);
        end
        function marginChangeInvalidatesCachedVehicle(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry(); pose = [28.9,10,0];
            cached = prepare_static_context(geometry,vehicle);
            enlarged = vehicle; enlarged.collision_margin_m.front = .2;
            oldHit = day6_static_collision_check(pose,cached,vehicle);
            changedHit = day6_static_collision_check(pose,cached,enlarged);
            rawHit = day6_static_collision_check(pose,geometry,enlarged);
            testCase.verifyFalse(oldHit);
            testCase.verifyTrue(changedHit);
            testCase.verifyEqual(changedHit,rawHit);
        end
        function dimensionsChangeInvalidatesCachedVehicle(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry(); pose = [28.9,10,0];
            cached = prepare_static_context(geometry,vehicle);
            enlarged = vehicle; enlarged.dimensions_m.wheelbase = .9;
            enlarged.dimensions_m.length = 1.8;
            changedHit = day6_static_collision_check(pose,cached,enlarged);
            rawHit = day6_static_collision_check(pose,geometry,enlarged);
            testCase.verifyTrue(changedHit);
            testCase.verifyEqual(changedHit,rawHit);
        end
        function obstacleMutationInvalidatesCachedGeometry(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry();
            cached = prepare_static_context(geometry,vehicle);
            cached.obstacles = {[10.2,9.95;10.4,9.95;10.4,10.05;10.2,10.05]};
            raw = rmfield(cached,'collision_context');
            cachedHit = day6_static_collision_check([10,10,0],cached,vehicle);
            rawHit = day6_static_collision_check([10,10,0],raw,vehicle);
            testCase.verifyTrue(cachedHit);
            testCase.verifyEqual(cachedHit,rawHit);
        end
        function boundaryMutationInvalidatesCachedGeometry(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry();
            cached = prepare_static_context(geometry,vehicle);
            cached.boundary_xy = [0,0;10.8,0;10.8,20;0,20;0,0];
            raw = rmfield(cached,'collision_context');
            [cachedHit,cachedDetail] = day6_static_collision_check([10,10,0],cached,vehicle);
            rawHit = day6_static_collision_check([10,10,0],raw,vehicle);
            testCase.verifyTrue(cachedHit);
            testCase.verifyEqual(cachedHit,rawHit);
            testCase.verifyEqual(cachedDetail.type,'road_boundary');
        end
        function roadBoundaryContactPreservesInclusiveGeometricConvention(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry();
            cached = prepare_static_context(geometry,vehicle);
            hit = day6_static_collision_check([29,10,0],cached,vehicle);
            % 原inpolygon把边界上的车角视为在内；半开资源图另行处理右/上界。
            testCase.verifyFalse(hit);
        end
        function concaveBoundaryRejectsEdgesCrossingOutsideDespiteInsideCorners(testCase)
            vehicle = testCase.vehicle(); vehicle.dimensions_m.width = 4;
            boundary = [0,0;6,0;6,6;4,6;4,3;2,3;2,6;0,6;0,0];
            geometry = struct('boundary_xy',boundary,'obstacles',{{}},'bounds',[0,0,6,6]);
            cached = prepare_static_context(geometry,vehicle);
            [hit,detail] = day6_static_collision_check([3,4.5,pi/2],cached,vehicle);
            cornersInside = inpolygon(detail.corners_xy(:,1),detail.corners_xy(:,2), ...
                boundary(:,1),boundary(:,2));
            testCase.verifyTrue(all(cornersInside));
            testCase.verifyTrue(hit);
            testCase.verifyEqual(detail.type,'road_boundary');
        end
        function concaveObstacleGapRemainsFreeDespiteAabbOverlap(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry();
            geometry.obstacles = {[8,8;12,8;12,12;11,12;11,9;9,9;9,12;8,12;8,8]};
            cached = prepare_static_context(geometry,vehicle);
            [hit,detail] = day6_static_collision_check([10,10,pi/2],cached,vehicle);
            testCase.verifyFalse(hit);
            testCase.verifyEqual(detail.type,'none');
        end
        function obstacleCompletelyContainedByVehicleIsCollision(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry();
            obstacle = [10.2,9.95;10.4,9.95;10.4,10.05;10.2,10.05];
            geometry.obstacles = {obstacle};
            cached = prepare_static_context(geometry,vehicle);
            [hit,detail] = day6_static_collision_check([10,10,0],cached,vehicle);
            vehicleCornersInObstacle = inpolygon(detail.corners_xy(:,1),detail.corners_xy(:,2), ...
                obstacle(:,1),obstacle(:,2));
            testCase.verifyFalse(any(vehicleCornersInObstacle));
            testCase.verifyTrue(hit);
            testCase.verifyEqual(detail.type,'static_obstacle');
        end
        function edgeIntersectionWithoutContainedVerticesIsCollision(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry();
            obstacle = [10.2,9;10.3,9;10.3,11;10.2,11];
            geometry.obstacles = {obstacle};
            cached = prepare_static_context(geometry,vehicle);
            [hit,detail] = day6_static_collision_check([10,10,0],cached,vehicle);
            body = detail.corners_xy;
            verticesInBody = inpolygon(obstacle(:,1),obstacle(:,2),body(:,1),body(:,2));
            verticesInObstacle = inpolygon(body(:,1),body(:,2),obstacle(:,1),obstacle(:,2));
            testCase.verifyFalse(any(verticesInBody) || any(verticesInObstacle));
            testCase.verifyTrue(hit);
        end
        function contactWithObstacleEdgeIsCollision(testCase)
            vehicle = testCase.vehicle(); geometry = testCase.geometry();
            geometry.obstacles = {[11,9;12,9;12,11;11,11]};
            cached = prepare_static_context(geometry,vehicle);
            [hit,detail] = day6_static_collision_check([10,10,0],cached,vehicle);
            testCase.verifyTrue(hit);
            testCase.verifyEqual(detail.type,'static_obstacle');
        end
        function mixedStaticDynamicBlockReportsVehicleOwner(testCase)
            planner = testCase.planner(); vehicle = testCase.vehicle();
            ego = [10.25,10.25,0,0,.25;10.25,10.25,0,0,.45];
            high = dynamic_vehicle_obstacle(ego,vehicle,'vehicle_002',planner);
            map = st_occupancy_map(planner);
            index = sub2ind([map.nx,map.ny,map.nt],21,21,1);
            map = st_write_occupancy(map,index,'static_001',2);
            map = st_write_occupancy(map,index,'vehicle_002',3);
            prepared = prepare_dynamic_context(map,high);
            [rawHit,rawReport] = check_trajectory_conflict(ego,high,vehicle,planner,map, ...
                struct('collect_distance_samples',false));
            [cachedHit,cachedReport] = check_trajectory_conflict(ego,high,vehicle,planner,prepared, ...
                struct('collect_distance_samples',false));
            testCase.verifyTrue(rawHit && cachedHit);
            testCase.verifyTrue(cachedReport.resource_conflict);
            testCase.verifyEqual(cachedReport.obstacle_id,'vehicle_002');
            testCase.verifyEqual(cachedReport.obstacle_id,rawReport.obstacle_id);
            testCase.verifyTrue(any(cachedReport.resource_indices==index));
        end
        function multipleVehiclesDoNotReturnStaticOwner(testCase)
            planner = testCase.planner(); vehicle = testCase.vehicle();
            ego = [10.25,10.25,0,0,.25;10.25,10.25,0,0,.45];
            high2 = dynamic_vehicle_obstacle(ego,vehicle,'vehicle_002',planner);
            high4 = dynamic_vehicle_obstacle(ego,vehicle,'vehicle_004',planner);
            map = st_occupancy_map(planner);
            index = sub2ind([map.nx,map.ny,map.nt],21,21,1);
            map = st_write_occupancy(map,index,'static_001',2);
            map = st_write_occupancy(map,index,'vehicle_002',3);
            map = st_write_occupancy(map,index,'vehicle_004',3);
            [hit,report] = check_trajectory_conflict(ego,[high2,high4],vehicle,planner,map, ...
                struct('collect_distance_samples',false));
            testCase.verifyTrue(hit);
            testCase.verifyTrue(any(strcmp(report.obstacle_id,{'vehicle_002','vehicle_004'})));
            testCase.verifyNotEqual(report.obstacle_id,'static_001');
        end
        function writingNewVehicleInvalidatesDynamicCache(testCase)
            planner = testCase.planner(); vehicle = testCase.vehicle();
            oldEgo = [10.25,10.25,0,0,.25;10.25,10.25,0,0,.45];
            newEgo = [20.25,10.25,0,0,.25;20.25,10.25,0,0,.45];
            oldHigh = dynamic_vehicle_obstacle(oldEgo,vehicle,'vehicle_002',planner);
            newHigh = dynamic_vehicle_obstacle(newEgo,vehicle,'vehicle_003',planner);
            map = st_occupancy_map(planner);
            oldIndex = sub2ind([map.nx,map.ny,map.nt],21,21,1);
            newIndex = sub2ind([map.nx,map.ny,map.nt],41,21,1);
            map = st_write_occupancy(map,oldIndex,'vehicle_002',3);
            map = prepare_dynamic_context(map,oldHigh);
            changed = st_write_occupancy(map,newIndex,'vehicle_003',3);
            [hit,report] = check_trajectory_conflict(newEgo,[oldHigh,newHigh],vehicle,planner,changed, ...
                struct('collect_distance_samples',false));
            testCase.verifyTrue(hit);
            testCase.verifyEqual(report.obstacle_id,'vehicle_003');
            testCase.verifyTrue(any(report.resource_indices==newIndex));
        end
        function highPriorityIdRegisteredAsStaticIsRejected(testCase)
            planner = testCase.planner(); vehicle = testCase.vehicle();
            ego = [10.25,10.25,0,0,.25;10.25,10.25,0,0,.45];
            high = dynamic_vehicle_obstacle(ego,vehicle,'vehicle_002',planner);
            map = st_occupancy_map(planner);
            index = sub2ind([map.nx,map.ny,map.nt],21,21,1);
            map = st_write_occupancy(map,index,'vehicle_002',2);
            testCase.verifyError(@() prepare_dynamic_context(map,high), ...
                'DynamicCollision:InvalidContext');
        end
        function changedObstacleListChecksUnwrittenVehicleDespitePreparedMap(testCase)
            [planner,vehicle,prepared,high2,~,ego] = testCase.reservedVehicles();
            unwritten = dynamic_vehicle_obstacle(ego,vehicle,'vehicle_004',planner);
            testCase.verifyError(@() check_trajectory_conflict(ego,[high2,unwritten], ...
                vehicle,planner,prepared,struct('collect_distance_samples',false)), ...
                'DynamicCollision:InvalidContext');
        end
        function subsetObstacleListStillChecksEveryReservedDynamicVehicle(testCase)
            [planner,vehicle,prepared,high2,~,ego] = testCase.reservedVehicles();
            [hit,report] = check_trajectory_conflict(ego,high2,vehicle,planner,prepared, ...
                struct('collect_distance_samples',false));
            testCase.verifyTrue(hit);
            testCase.verifyTrue(report.resource_conflict);
            testCase.verifyEqual(report.obstacle_id,'vehicle_003');
        end
        function emptyObstacleListStillChecksReservedDynamicVehicles(testCase)
            [planner,vehicle,prepared,~,~,ego] = testCase.reservedVehicles();
            [hit,report] = check_trajectory_conflict(ego,[],vehicle,planner,prepared, ...
                struct('collect_distance_samples',false));
            testCase.verifyTrue(hit);
            testCase.verifyTrue(report.resource_conflict);
            testCase.verifyEqual(report.obstacle_id,'vehicle_003');
        end
        function heapUpdateSkipsStaleEntryAndRejectsWorseDuplicate(testCase)
            old = vhybrid_node([1,0,0,1,0],0,0,0,5,0,[1,0,0,0,0],1);
            other = vhybrid_node([2,0,0,1,0],0,0,0,3,0,[2,0,0,0,0],2);
            improved = vhybrid_node([1,0,0,1,0],0,0,0,1,0,[1,0,0,0,0],3);
            queue = vhybrid_open_set();
            acceptedOld = queue.push_or_update(old);
            acceptedOther = queue.push_or_update(other);
            acceptedImproved = queue.push_or_update(improved);
            acceptedWorse = queue.push_or_update(old);
            [first,validFirst] = queue.pop_min(); [second,validSecond] = queue.pop_min();
            [~,validStale] = queue.pop_min();
            testCase.verifyTrue(acceptedOld && acceptedOther && acceptedImproved);
            testCase.verifyFalse(acceptedWorse);
            testCase.verifyTrue(validFirst && validSecond);
            testCase.verifyEqual([first.node_id,second.node_id],[3,2]);
            testCase.verifyFalse(validStale);
            testCase.verifyEqual(queue.count(),0);
        end
        function heapProducesSameStableOrderAsLinearMinimum(testCase)
            insertion = (1:250).'; costs = mod(insertion*13,17);
            [~,expectedOrder] = sortrows([costs,insertion],[1,2]);
            expectedIds = 251-expectedOrder;
            actualIds = testCase.popAllNodes(costs);
            testCase.verifyEqual(actualIds,expectedIds);
        end
    end
    methods (Static,Access=private)
        function vehicle = vehicle()
            vehicle = struct('dimensions_m',struct('width',.4,'length',1.5, ...
                'wheelbase',.6,'rear_axle_to_rear',.5,'front_axle_to_front',.4), ...
                'collision_margin_m',struct('front',0,'rear',0,'side',0), ...
                'steering',struct('maximum_steer_rad',.4680017179));
        end
        function geometry = geometry()
            geometry = struct('boundary_xy',[0,0;30,0;30,20;0,20;0,0], ...
                'obstacles',{{}},'bounds',[0,0,30,20]);
        end
        function planner = planner()
            root = fileparts(fileparts(mfilename('fullpath')));
            planner = jsondecode(fileread(fullfile(root,'config','planner_config.json')));
            planner.st_occupancy.t_min = 0; planner.st_occupancy.t_max = 2;
            planner.st_occupancy.x_min = 0; planner.st_occupancy.x_max = 30;
            planner.st_occupancy.y_min = 0; planner.st_occupancy.y_max = 20;
            planner.st_occupancy.dx = .5; planner.st_occupancy.dy = .5; planner.st_occupancy.dt = .5;
            planner.st_occupancy.trajectory_end_policy = 'release';
            planner.day6.goal_hold_enabled = false;
        end
        function ids = popAllNodes(costs)
            % 公开堆接口批量准备；排序参考由测试方法的sortrows独立给出。
            queue = vhybrid_open_set(); count = numel(costs); ids = zeros(count,1);
            for k=1:count
                queue.push(vhybrid_node([k,0,0,1,0],0,0,0,costs(k),0,[k,0,0,0,0],count+1-k));
            end
            for k=1:count
                [node,valid] = queue.pop_min();
                assert(valid,'EquivalenceTest:MissingNode','全部不重复状态应能取出。');
                ids(k) = node.node_id;
            end
        end
        function [planner,vehicle,map,high2,high3,ego3] = reservedVehicles()
            % 用真实Day5轨迹标记接口准备两辆已保留车辆，公共查询决定冲突。
            planner = test_day6_performance_equivalence.planner();
            vehicle = test_day6_performance_equivalence.vehicle();
            ego2 = [10.25,10.25,0,0,.25;10.25,10.25,0,0,.45];
            ego3 = [20.25,10.25,0,0,.25;20.25,10.25,0,0,.45];
            high2 = dynamic_vehicle_obstacle(ego2,vehicle,'vehicle_002',planner);
            high3 = dynamic_vehicle_obstacle(ego3,vehicle,'vehicle_003',planner);
            map = st_occupancy_map(planner);
            map = mark_trajectory_occupancy(map,ego2,vehicle,'vehicle_002');
            map = mark_trajectory_occupancy(map,ego3,vehicle,'vehicle_003');
            map = prepare_dynamic_context(map,[high2,high3]);
        end
    end
end
