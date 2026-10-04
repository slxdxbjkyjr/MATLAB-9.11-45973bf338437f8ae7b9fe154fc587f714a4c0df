classdef test_st_occupancy_map < matlab.unittest.TestCase
    %TEST_ST_OCCUPANCY_MAP Day5 时空资源块公共接口的独立验收测试。
    % 输入：通过私有辅助函数生成的小地图、车辆矩形及带时间轨迹。
    % 输出：matlab.unittest.TestResult 中逐项通过/失败结果。
    % 算法逻辑：每个用例按准备、执行、断言顺序检查公开函数；不调用
    % V-Hybrid A* 主循环，也不保存图片或修改已有工程/输出文件。

    methods (TestClassSetup)
        function addDay5Functions(testCase)
            % 将本工程 MATLAB 函数加入路径，测试完成后恢复原路径。
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            originalPath = path;
            testCase.addTeardown(@() path(originalPath));
            addpath(genpath(fullfile(projectRoot, 'matlab')));
        end
    end

    methods (Test)
        function resourceDimensionsAndCoordinates(testCase)
            % 三维数组、边界坐标与中心坐标使用同一半开资源块约定。
            planner = testCase.smallPlanner();
            map = st_occupancy_map(planner);
            testCase.verifySize(map.occupancy, [60, 40, 20]);
            testCase.verifyClass(map.occupancy, 'logical');
            testCase.verifyClass(map.owner_type, 'uint8');
            testCase.verifyClass(map.owner_id, 'uint32');
            testCase.verifyClass(map.reason, 'uint8');
            testCase.verifyEqual([map.nx, map.ny, map.nt], [60, 40, 20]);
            testCase.verifyEqual(map.x_edges(:).', 0:0.5:30, 'AbsTol', 1e-12);
            testCase.verifyEqual(map.y_edges(:).', 0:0.5:20, 'AbsTol', 1e-12);
            testCase.verifyEqual(map.t_edges(:).', 0:0.5:10, 'AbsTol', 1e-12);
            testCase.verifyEqual(map.x_coords(:).', 0.25:0.5:29.75, 'AbsTol', 1e-12);
            testCase.verifyEqual(map.y_coords(:).', 0.25:0.5:19.75, 'AbsTol', 1e-12);
            testCase.verifyEqual(map.t_coords(:).', 0.25:0.5:9.75, 'AbsTol', 1e-12);
        end

        function lowerBoundsBelongToFirstBlock(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            [occupied, type, owner, indices, detail] = query_occupancy(0, 0, 0, map);
            testCase.verifyFalse(occupied);
            testCase.verifyEqual(type, 'free');
            testCase.verifyEmpty(owner);
            testCase.verifyEqual(indices, [1, 1, 1]);
            testCase.verifyEmpty(detail.owner_ids);
        end

        function interiorEdgesBelongToNextBlock(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            [occupied, type, ~, indices] = query_occupancy(0.5, 0.5, 0.5, map);
            testCase.verifyFalse(occupied);
            testCase.verifyEqual(type, 'free');
            testCase.verifyEqual(indices, [2, 2, 2]);
        end

        function representableCoordinatesBelowUpperBoundsUseLastBlocks(testCase)
            % 严格小于上界的最后一个可表示浮点数仍属于最后的半开块。
            map = st_occupancy_map(testCase.smallPlanner());
            [occupied, type, ~, indices] = query_occupancy(30 - eps(30), ...
                20 - eps(20), 10 - eps(10), map);
            testCase.verifyFalse(occupied);
            testCase.verifyEqual(type, 'free');
            testCase.verifyEqual(indices, [60, 40, 20]);
        end

        function decimalResolutionEdgesUseSameHalfOpenConvention(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.x_max = 1;
            planner.st_occupancy.y_max = 1;
            planner.st_occupancy.t_max = 1;
            planner.st_occupancy.dx = 0.1;
            planner.st_occupancy.dy = 0.1;
            planner.st_occupancy.dt = 0.1;
            map = st_occupancy_map(planner);
            [occupied, type, ~, indices] = query_occupancy(0.3, 0.3, 0.3, map);
            testCase.verifyFalse(occupied);
            testCase.verifyEqual(type, 'free');
            testCase.verifyEqual(indices, [4, 4, 4]);
        end

        function spatialUpperBoundIsPermanentlyInfeasible(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            [occupied, type, ~, indices, detail] = query_occupancy(30, 10, 1, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'out_of_bounds');
            testCase.verifySize(indices, [1, 3]);
            testCase.verifyNotEmpty(detail.reason);
        end

        function timeUpperBoundIsPermanentlyInfeasible(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            [occupied, type] = query_occupancy(10, 10, 10, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'out_of_bounds');
        end

        function negativeCoordinateIsPermanentlyInfeasible(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            [occupied, type] = query_occupancy(-0.01, 10, 1, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'out_of_bounds');
        end

        function nonfiniteQueryIsPermanentlyInfeasible(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            [occupied, type] = query_occupancy(NaN, 10, 1, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'out_of_bounds');
        end

        function searchStepMayBeIntegerMultipleOfLayerStep(testCase)
            planner = testCase.smallPlanner();
            planner.vhybrid.time_step_s = 1;
            map = st_occupancy_map(planner);
            testCase.verifyEqual(map.nt, 20);
        end

        function mismatchedSearchStepRejected(testCase)
            planner = testCase.smallPlanner();
            planner.vhybrid.time_step_s = 0.75;
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:TimeStepMismatch');
        end

        function nonnumericSearchStepRejected(testCase)
            planner = testCase.smallPlanner();
            planner.vhybrid.time_step_s = '1';
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:TimeStepMismatch');
        end

        function fractionalSpatialSpanRejected(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.x_max = 30.1;
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:InvalidConfig');
        end

        function nonpositiveResolutionRejected(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.dt = 0;
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:InvalidConfig');
        end

        function unsupportedCoordinateConventionRejected(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.coordinate_convention = 'closed';
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:InvalidConfig');
        end

        function nonfiniteConfigFieldRejected(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.dx = NaN;
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:InvalidConfig');
        end

        function nonscalarConfigFieldRejected(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.dx = [0.5, 0.5];
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:InvalidConfig');
        end

        function nonscalarPlannerConfigRejected(testCase)
            planner = repmat(testCase.smallPlanner(), 1, 2);
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:InvalidConfig');
        end

        function nonfiniteResourceLimitRejected(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.max_resource_blocks = NaN;
            testCase.verifyError(@() st_occupancy_map(planner), 'STOccupancy:InvalidConfig');
        end

        function vehicleInflationIncludesAsymmetryAndMargins(testCase)
            vehicle = testCase.smallVehicle();
            vehicle.collision_margin_m = struct('front', 0.1, 'rear', 0.05, 'side', 0.02);
            footprint = inflate_vehicle_occupancy(vehicle, 0.2);
            testCase.verifyEqual(footprint.front, 1.3, 'AbsTol', 1e-12);
            testCase.verifyEqual(footprint.rear, 0.75, 'AbsTol', 1e-12);
            testCase.verifyEqual(footprint.half_width, 0.42, 'AbsTol', 1e-12);
            testCase.verifyEqual(footprint.radius, hypot(1.3, 0.42), 'AbsTol', 1e-12);
            testCase.verifySize(footprint.corners_local, [4, 2]);
        end

        function nonnumericVehicleDimensionRejected(testCase)
            vehicle = testCase.smallVehicle();
            vehicle.dimensions_m.width = '0.4';
            testCase.verifyError(@() inflate_vehicle_occupancy(vehicle), 'STOccupancy:InvalidVehicle');
        end

        function missingVehicleDimensionRejected(testCase)
            vehicle = testCase.smallVehicle();
            vehicle.dimensions_m = rmfield(vehicle.dimensions_m, 'wheelbase');
            testCase.verifyError(@() inflate_vehicle_occupancy(vehicle), 'STOccupancy:InvalidVehicle');
        end

        function nonfiniteVehicleDimensionRejected(testCase)
            vehicle = testCase.smallVehicle();
            vehicle.dimensions_m.width = NaN;
            testCase.verifyError(@() inflate_vehicle_occupancy(vehicle), 'STOccupancy:InvalidVehicle');
        end

        function missingVehicleSafetyMarginRejected(testCase)
            vehicle = testCase.smallVehicle();
            vehicle.collision_margin_m = rmfield(vehicle.collision_margin_m, 'side');
            testCase.verifyError(@() inflate_vehicle_occupancy(vehicle), 'STOccupancy:InvalidVehicle');
        end

        function complexVehicleSafetyMarginRejected(testCase)
            vehicle = testCase.smallVehicle();
            vehicle.collision_margin_m.side = 0.1 + 0.1i;
            testCase.verifyError(@() inflate_vehicle_occupancy(vehicle), 'STOccupancy:InvalidVehicle');
        end

        function staticObstacleOccupiesEveryTimeLayer(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            geometry = testCase.rectangleGeometry();
            geometry.obstacles = {[10, 8; 12, 8; 12, 12; 10, 12]};
            geometry.obstacle_ids = {'wall-A'};
            map = mark_static_obstacles(map, geometry, testCase.smallVehicle());
            [occupied, type, owner, indices, detail] = query_occupancy(11, 10, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'static_obstacle');
            testCase.verifyEqual(owner, 'wall-A');
            testCase.verifyTrue(all(squeeze(map.occupancy(indices(1), indices(2), :))));
            testCase.verifyTrue(ismember('wall-A', detail.owner_ids));
        end

        function longPolygonEdgeInflatedAwayFromVertices(testCase)
            % 查询点接近长边中部，远离所有角点；仅膨胀顶点会漏检。
            map = st_occupancy_map(testCase.smallPlanner());
            geometry = testCase.rectangleGeometry();
            geometry.obstacles = {[10, 2; 11, 2; 11, 18; 10, 18]};
            map = mark_static_obstacles(map, geometry, testCase.smallVehicle());
            [occupied, type] = query_occupancy(9.25, 10.25, 1.25, map);
            [freePoint, freeType] = query_occupancy(5.25, 10.25, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'static_obstacle');
            testCase.verifyFalse(freePoint);
            testCase.verifyEqual(freeType, 'free');
        end

        function parkingBoundaryOccupiesEveryTimeLayer(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            map = mark_static_obstacles(map, testCase.rectangleGeometry(), testCase.smallVehicle());
            [occupied, type, ~, indices] = query_occupancy(0.25, 10.25, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'boundary');
            testCase.verifyTrue(all(squeeze(map.occupancy(indices(1), indices(2), :))));
        end

        function concaveBoundaryAndItsInnerEdgesHandled(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            geometry = testCase.rectangleGeometry();
            geometry.boundary_xy = [0, 0; 30, 0; 30, 20; 15, 20; 15, 10; 0, 10; 0, 0];
            map = mark_static_obstacles(map, geometry, testCase.smallVehicle());
            [outside, outsideType] = query_occupancy(5.25, 15.25, 1.25, map);
            [nearNotch, notchType] = query_occupancy(15.75, 10.75, 1.25, map);
            [inside, insideType] = query_occupancy(25.25, 15.25, 1.25, map);
            testCase.verifyTrue(outside);
            testCase.verifyEqual(outsideType, 'boundary');
            testCase.verifyTrue(nearNotch);
            testCase.verifyEqual(notchType, 'boundary');
            testCase.verifyFalse(inside);
            testCase.verifyEqual(insideType, 'free');
        end

        function dynamicOccupancyLimitedToTrajectoryTime(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 1.2; 10, 10, 0, 0, 2.1];
            map = mark_trajectory_occupancy(map, trajectory, testCase.smallVehicle(), 'vehicle-A');
            [before, beforeType] = query_occupancy(10, 10, 0.75, map);
            [during, duringType, owner] = query_occupancy(10, 10, 1.25, map);
            [after, afterType] = query_occupancy(10, 10, 2.75, map);
            testCase.verifyFalse(before);
            testCase.verifyEqual(beforeType, 'free');
            testCase.verifyTrue(during);
            testCase.verifyEqual(duringType, 'dynamic_vehicle');
            testCase.verifyEqual(owner, 'vehicle-A');
            testCase.verifyFalse(after);
            testCase.verifyEqual(afterType, 'free');
        end

        function sameSpaceDifferentTimeDoesNotConflict(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectoryA = [10, 10, 0, 0, 1; 10, 10, 0, 0, 2];
            trajectoryB = [10, 10, 0, 0, 5; 10, 10, 0, 0, 6];
            map = mark_trajectory_occupancy(map, trajectoryA, testCase.smallVehicle(), 'vehicle-A');
            map = mark_trajectory_occupancy(map, trajectoryB, testCase.smallVehicle(), 'vehicle-B');
            [first, firstType, firstOwner, ~, firstDetail] = query_occupancy(10, 10, 1.25, map);
            [between, betweenType] = query_occupancy(10, 10, 3.25, map);
            [second, secondType, secondOwner, ~, secondDetail] = query_occupancy(10, 10, 5.25, map);
            testCase.verifyTrue(first);
            testCase.verifyEqual(firstType, 'dynamic_vehicle');
            testCase.verifyEqual(firstOwner, 'vehicle-A');
            testCase.verifyEqual(firstDetail.owner_ids, {'vehicle-A'});
            testCase.verifyFalse(between);
            testCase.verifyEqual(betweenType, 'free');
            testCase.verifyTrue(second);
            testCase.verifyEqual(secondType, 'dynamic_vehicle');
            testCase.verifyEqual(secondOwner, 'vehicle-B');
            testCase.verifyEqual(secondDetail.owner_ids, {'vehicle-B'});
        end

        function sameTimeSameSpacePreservesBothVehicleIds(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 1; 10, 10, 0, 0, 2];
            map = mark_trajectory_occupancy(map, trajectory, testCase.smallVehicle(), 'vehicle-A');
            map = mark_trajectory_occupancy(map, trajectory, testCase.smallVehicle(), 'vehicle-B');
            [occupied, type, owner, ~, detail] = query_occupancy(10, 10, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'multiple_owners');
            testCase.verifyNotEmpty(owner);
            testCase.verifyEqual(sort(detail.owner_ids(:)), {'vehicle-A'; 'vehicle-B'});
        end

        function threeOverlappingOwnersAreAllReported(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 1; 10, 10, 0, 0, 2];
            vehicle = testCase.smallVehicle();
            map = mark_trajectory_occupancy(map, trajectory, vehicle, 'vehicle-A');
            map = mark_trajectory_occupancy(map, trajectory, vehicle, 'vehicle-B');
            map = mark_trajectory_occupancy(map, trajectory, vehicle, 'vehicle-C');
            [occupied, type, ~, ~, detail] = query_occupancy(10, 10, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'multiple_owners');
            testCase.verifyEqual(sort(detail.owner_ids(:)), ...
                {'vehicle-A'; 'vehicle-B'; 'vehicle-C'});
        end

        function repeatedOwnerMarkingDoesNotCreateFalseConflict(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 1; 10, 10, 0, 0, 2];
            vehicle = testCase.smallVehicle();
            map = mark_trajectory_occupancy(map, trajectory, vehicle, 'vehicle-A');
            map = mark_trajectory_occupancy(map, trajectory, vehicle, 'vehicle-A');
            [occupied, type, owner, ~, detail] = query_occupancy(10, 10, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'dynamic_vehicle');
            testCase.verifyEqual(owner, 'vehicle-A');
            testCase.verifyEqual(detail.owner_ids, {'vehicle-A'});
            testCase.verifyEqual(numel(map.owners), 1);
        end

        function dynamicOverlapDoesNotEraseStaticOwner(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            geometry = testCase.rectangleGeometry();
            geometry.obstacles = {[10, 8; 12, 8; 12, 12; 10, 12]};
            geometry.obstacle_ids = {'wall-A'};
            map = mark_static_obstacles(map, geometry, testCase.smallVehicle());
            trajectory = [11, 10, 0, 0, 1; 11, 10, 0, 0, 2];
            map = mark_trajectory_occupancy(map, trajectory, testCase.smallVehicle(), 'vehicle-A');
            [occupied, type, ~, ~, detail] = query_occupancy(11, 10, 1.25, map);
            [later, laterType, laterOwner] = query_occupancy(11, 10, 5.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'multiple_owners');
            testCase.verifyEqual(sort(detail.owner_ids(:)), {'vehicle-A'; 'wall-A'});
            testCase.verifyTrue(later);
            testCase.verifyEqual(laterType, 'static_obstacle');
            testCase.verifyEqual(laterOwner, 'wall-A');
        end

        function rotatedAsymmetricFootprintUsesRearAxleOrigin(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            vehicle = testCase.longVehicle();
            trajectory = [10, 10, pi / 2, 0, 1];
            map = mark_trajectory_occupancy(map, trajectory, vehicle, 'vehicle-A');
            [frontOccupied, frontType] = query_occupancy(10.25, 11.75, 1.25, map);
            [behindRear, rearType] = query_occupancy(10.25, 8.75, 1.25, map);
            [unrotatedFront, unrotatedType] = query_occupancy(11.75, 10.25, 1.25, map);
            testCase.verifyTrue(frontOccupied);
            testCase.verifyEqual(frontType, 'dynamic_vehicle');
            testCase.verifyFalse(behindRear);
            testCase.verifyEqual(rearType, 'free');
            testCase.verifyFalse(unrotatedFront);
            testCase.verifyEqual(unrotatedType, 'free');
        end

        function footprintCellIntersectionDoesNotRequireCellCenterInside(testCase)
            % 小车矩形跨过 x=10.5 栅格边线，右侧块中心不在矩形内。
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10.49, 10.25, 0, 0, 1];
            map = mark_trajectory_occupancy(map, trajectory, testCase.tinyVehicle(), 'vehicle-A');
            [occupied, type] = query_occupancy(10.75, 10.25, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'dynamic_vehicle');
        end

        function fastMotionBetweenTimeCentersUsesSweptOccupancy(testCase)
            % 同一层内扫过 x=8；只检查层中心 t=1.25 时的 x=10 会漏检。
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [5, 10, 0, 25, 1.01; 15, 10, 0, 25, 1.49];
            map = mark_trajectory_occupancy(map, trajectory, testCase.tinyVehicle(), 'vehicle-A');
            [occupied, type] = query_occupancy(8.25, 10.25, 1.25, map);
            [nextLayer, nextType] = query_occupancy(8.25, 10.25, 1.75, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'dynamic_vehicle');
            testCase.verifyFalse(nextLayer);
            testCase.verifyEqual(nextType, 'free');
        end

        function angularInterpolationTakesShortRouteAcrossPi(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 170 * pi / 180, 0, 1; ...
                10, 10, -170 * pi / 180, 0, 2];
            map = mark_trajectory_occupancy(map, trajectory, testCase.longVehicle(), 'vehicle-A');
            [leftFront, leftType] = query_occupancy(8.25, 10.25, 1.75, map);
            [falseNorth, northType] = query_occupancy(10.25, 11.75, 1.75, map);
            testCase.verifyTrue(leftFront);
            testCase.verifyEqual(leftType, 'dynamic_vehicle');
            testCase.verifyFalse(falseNorth);
            testCase.verifyEqual(northType, 'free');
        end

        function rotatingSegmentSweepsBetweenEndpointFootprints(testCase)
            % 占用模块负责给定姿态序列的保守扫掠，不负责轨迹运动学验收。
            % 45 度处的前车角扫过右上区域，两端的矩形都未覆盖该位置。
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 1.01; 10, 10, pi / 2, 0, 1.49];
            map = mark_trajectory_occupancy(map, trajectory, testCase.longVehicle(), 'vehicle-A');
            [occupied, type] = query_occupancy(11.25, 11.25, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'dynamic_vehicle');
        end

        function pathStructInterfaceAccepted(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = struct('x', [10, 10], 'y', [10, 10], ...
                'theta', [0, 0], 'v', [0, 0], 't', [1, 2], 'valid', true);
            map = mark_trajectory_occupancy(map, trajectory, testCase.smallVehicle(), 'vehicle-A');
            [occupied, type, owner] = query_occupancy(10, 10, 1.25, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'dynamic_vehicle');
            testCase.verifyEqual(owner, 'vehicle-A');
        end

        function singletonTrajectoryOccupiesOneReleaseLayer(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            map = mark_trajectory_occupancy(map, [10, 10, 0, 0, 1.2], ...
                testCase.smallVehicle(), 'vehicle-A');
            [occupied, type] = query_occupancy(10, 10, 1.25, map);
            [after, afterType] = query_occupancy(10, 10, 1.75, map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'dynamic_vehicle');
            testCase.verifyFalse(after);
            testCase.verifyEqual(afterType, 'free');
        end

        function nearUpperBoundSingletonUsesLastValidBlocks(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [30 - eps(30), 20 - eps(20), 0, 0, 10 - eps(10)];
            map = mark_trajectory_occupancy(map, trajectory, testCase.tinyVehicle(), 'vehicle-A');
            [occupied, type, owner, indices] = query_occupancy(trajectory(1), ...
                trajectory(2), trajectory(5), map);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'dynamic_vehicle');
            testCase.verifyEqual(owner, 'vehicle-A');
            testCase.verifyEqual(indices, [60, 40, 20]);
        end

        function terminalPointOnTimeEdgeOccupiesItsOwnLayer(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 1; 10, 10, 0, 0, 2];
            map = mark_trajectory_occupancy(map, trajectory, testCase.smallVehicle(), 'vehicle-A');
            [endpoint, endpointType] = query_occupancy(10, 10, 2.25, map);
            [released, releaseType] = query_occupancy(10, 10, 2.75, map);
            testCase.verifyTrue(endpoint);
            testCase.verifyEqual(endpointType, 'dynamic_vehicle');
            testCase.verifyFalse(released);
            testCase.verifyEqual(releaseType, 'free');
        end

        function heldTerminalVehicleOccupiesRemainingHorizon(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
            map = st_occupancy_map(planner);
            trajectory = [10, 10, 0, 0, 1; 10, 10, 0, 0, 2];
            map = mark_trajectory_occupancy(map, trajectory, testCase.smallVehicle(), 'vehicle-A');
            [held, heldType, owner] = query_occupancy(10, 10, 9.75, map);
            [before, beforeType] = query_occupancy(10, 10, 0.75, map);
            testCase.verifyTrue(held);
            testCase.verifyEqual(heldType, 'dynamic_vehicle');
            testCase.verifyEqual(owner, 'vehicle-A');
            testCase.verifyFalse(before);
            testCase.verifyEqual(beforeType, 'free');
        end

        function movingTerminalVehicleCannotBeHeldAsParked(testCase)
            planner = testCase.smallPlanner();
            planner.st_occupancy.trajectory_end_policy = 'hold_until_t_max';
            map = st_occupancy_map(planner);
            trajectory = [10, 10, 0, 1, 1; 11, 10, 0, 1, 2];
            testCase.verifyError(@() mark_trajectory_occupancy(map, trajectory, ...
                testCase.smallVehicle(), 'vehicle-A'), 'STOccupancy:InvalidTrajectory');
        end

        function duplicateTrajectoryTimesRejected(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 1; 11, 10, 0, 0, 1];
            testCase.verifyError(@() mark_trajectory_occupancy(map, trajectory, ...
                testCase.smallVehicle(), 'vehicle-A'), 'STOccupancy:InvalidTrajectory');
        end

        function descendingTrajectoryTimesRejected(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 2; 11, 10, 0, 0, 1];
            testCase.verifyError(@() mark_trajectory_occupancy(map, trajectory, ...
                testCase.smallVehicle(), 'vehicle-A'), 'STOccupancy:InvalidTrajectory');
        end

        function nonfiniteTrajectoryRejected(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, NaN, 0, 1];
            testCase.verifyError(@() mark_trajectory_occupancy(map, trajectory, ...
                testCase.smallVehicle(), 'vehicle-A'), 'STOccupancy:InvalidTrajectory');
        end

        function unequalTrajectoryFieldLengthsRejected(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = struct('x', [10, 10], 'y', 10, 'theta', [0, 0], ...
                'v', [0, 0], 't', [1, 2]);
            testCase.verifyError(@() mark_trajectory_occupancy(map, trajectory, ...
                testCase.smallVehicle(), 'vehicle-A'), 'STOccupancy:InvalidTrajectory');
        end

        function invalidPlannerPathRejected(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = struct('x', 10, 'y', 10, 'theta', 0, 'v', 0, ...
                't', 1, 'valid', false);
            testCase.verifyError(@() mark_trajectory_occupancy(map, trajectory, ...
                testCase.smallVehicle(), 'vehicle-A'), 'STOccupancy:InvalidTrajectory');
        end

        function trajectoryTimeOutsideHorizonRejected(testCase)
            map = st_occupancy_map(testCase.smallPlanner());
            trajectory = [10, 10, 0, 0, 9; 10, 10, 0, 0, 10];
            testCase.verifyError(@() mark_trajectory_occupancy(map, trajectory, ...
                testCase.smallVehicle(), 'vehicle-A'), 'STOccupancy:TrajectoryOutsideTime');
        end

        function markingMapCopyDoesNotMutateOriginal(testCase)
            originalMap = st_occupancy_map(testCase.smallPlanner());
            markedMap = mark_trajectory_occupancy(originalMap, [10, 10, 0, 0, 1], ...
                testCase.smallVehicle(), 'vehicle-A');
            [originalOccupied, originalType, ~, ~, originalDetail] = ...
                query_occupancy(10, 10, 1.25, originalMap);
            [markedOccupied, markedType] = query_occupancy(10, 10, 1.25, markedMap);
            testCase.verifyFalse(originalOccupied);
            testCase.verifyEqual(originalType, 'free');
            testCase.verifyEmpty(originalDetail.owner_ids);
            testCase.verifyEmpty(originalMap.owners);
            testCase.verifyTrue(markedOccupied);
            testCase.verifyEqual(markedType, 'dynamic_vehicle');
        end

        function actualProjectConfigLoadsWithoutChangingDay4(testCase)
            % 集成检查使用当前 JSON，不执行路径搜索，也不写入输出。
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
            planner = jsondecode(fileread(fullfile(projectRoot, 'config', 'planner_config.json')));
            vehicle = jsondecode(fileread(fullfile(projectRoot, 'config', 'vehicle_config.json')));
            geometryConfig = jsondecode(fileread(fullfile(projectRoot, 'config', 'map_config.json')));
            geometry = summon_map(geometryConfig);
            map = st_occupancy_map(planner, geometry);
            map = mark_static_obstacles(map, geometry, vehicle);
            ratio = planner.vhybrid.time_step_s / planner.st_occupancy.dt;
            [occupied, type] = query_occupancy(planner.st_occupancy.x_min + ...
                planner.st_occupancy.dx / 2, 10, planner.st_occupancy.t_min, map);
            testCase.verifyEqual(ratio, round(ratio), 'AbsTol', 1e-12);
            testCase.verifyGreaterThanOrEqual(ratio, 1);
            testCase.verifyTrue(occupied);
            testCase.verifyEqual(type, 'boundary');
            % Day6可开启顺序规划；公共占用接口不应绑定于旧Day5阶段开关。
            testCase.verifyClass(planner.priority_planning_enabled, 'logical');
            testCase.verifySize(planner.priority_planning_enabled, [1, 1]);
            % Day6的搜索启发权重只注入Demo的内存副本，原Day4参数仍保持。
            testCase.verifyFalse(isfield(planner.vhybrid,'heuristic_weight'));
            testCase.verifyEqual(planner.vhybrid.time_step_s, 0.5);
            testCase.verifyEqual(planner.vhybrid.time_grid_resolution_s, 0.5);
            testCase.verifyEqual(planner.vhybrid.initial_speed_mps, 0);
            testCase.verifyEqual(planner.vhybrid.terminal_speed_mps, 0);
        end
    end

    methods (Static, Access = private)
        function planner = smallPlanner()
            % 输入：无。输出：测试专用集中配置，距离 m、时间 s。
            % dt 与 Day4 搜索步长同为 0.5 s；默认只释放终点后的时间层。
            settings = struct('x_min', 0, 'x_max', 30, 'y_min', 0, 'y_max', 20, ...
                't_min', 0, 't_max', 10, 'dx', 0.5, 'dy', 0.5, 'dt', 0.5, ...
                'safety_distance_m', 0, 'sweep_max_dt_s', 0.1, ...
                'sweep_max_distance_m', 0.1, 'coordinate_convention', 'half_open', ...
                'trajectory_end_policy', 'release');
            planner = struct('st_occupancy', settings, 'vhybrid', struct('time_step_s', 0.5));
        end

        function vehicle = smallVehicle()
            % 后轴原点的测试矩形：前向 1 m、后向 0.5 m、宽度 0.4 m。
            dimensions = struct('width', 0.4, 'length', 1.5, 'wheelbase', 0.6, ...
                'rear_axle_to_rear', 0.5, 'front_axle_to_front', 0.4);
            vehicle = struct('dimensions_m', dimensions, ...
                'collision_margin_m', struct('front', 0, 'rear', 0, 'side', 0));
        end

        function vehicle = longVehicle()
            % 长矩形用来区分航向旋转及前后不对称，而非圆形占用。
            vehicle = test_st_occupancy_map.smallVehicle();
            vehicle.dimensions_m.width = 0.5;
            vehicle.dimensions_m.length = 2.5;
            vehicle.dimensions_m.wheelbase = 1.6;
        end

        function vehicle = tinyVehicle()
            % 小矩形使快速扫掠/栅格相交的验收不会被车辆长度掩盖。
            vehicle = test_st_occupancy_map.smallVehicle();
            vehicle.dimensions_m.width = 0.1;
            vehicle.dimensions_m.length = 0.2;
            vehicle.dimensions_m.wheelbase = 0.06;
            vehicle.dimensions_m.front_axle_to_front = 0.04;
            vehicle.dimensions_m.rear_axle_to_rear = 0.1;
        end

        function geometry = rectangleGeometry()
            % 闭合矩形停车场；障碍物由相应用例显式赋值。
            geometry = struct('boundary_xy', [0, 0; 30, 0; 30, 20; 0, 20; 0, 0], ...
                'obstacles', {{}});
        end
    end
end
