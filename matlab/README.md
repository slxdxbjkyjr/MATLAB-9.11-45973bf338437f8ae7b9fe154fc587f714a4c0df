# MATLAB 预留接口

Day 1 不实现路径搜索。

后续建议新增：

- `loadSummonScenario.m`：读取 JSON 并完成字段校验；
- `vehicleDynamicAdapter.m`：通过显式车辆参数调用或复刻 `VehicleDynamic`；
- `checkSpatiotemporalCollision.m`：独立于 APA 车位逻辑的时空碰撞检查；
- `planSummonTrajectory.m`：未来 V-Hybrid A* 入口。
