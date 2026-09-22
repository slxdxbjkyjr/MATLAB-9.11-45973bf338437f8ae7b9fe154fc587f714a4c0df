param(
    [string]$Root = (Join-Path $PSScriptRoot '..')
)

$ErrorActionPreference = 'Stop'

function Read-Json([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "缺少配置文件: $Path"
    }
    return Get-Content -Raw -LiteralPath $Path | ConvertFrom-Json
}

$configDir = Join-Path $Root 'config'
$outputDir = Join-Path $Root 'outputs'
$vehicleConfig = Read-Json (Join-Path $configDir 'vehicle_config.json')
$mapConfig = Read-Json (Join-Path $configDir 'map_config.json')
$plannerConfig = Read-Json (Join-Path $configDir 'planner_config.json')
$scenario = Read-Json (Join-Path $configDir 'scenario_001.json')

if ($scenario.vehicles.Count -ne 3) { throw 'scenario_001 必须包含 3 辆车。' }
if ($vehicleConfig.summon_prototype.vehicle_count -ne 3) { throw 'vehicle_config.vehicle_count 不为 3。' }
if ($mapConfig.boundary.vertices_xy.Count -lt 4) { throw '道路边界顶点不足。' }
$first = $mapConfig.boundary.vertices_xy[0]
$last = $mapConfig.boundary.vertices_xy[$mapConfig.boundary.vertices_xy.Count - 1]
if (($first[0] -ne $last[0]) -or ($first[1] -ne $last[1])) { throw '道路边界没有闭合。' }
if ($plannerConfig.path_search_enabled) { throw 'Day 1 不应启用路径搜索。' }

$vehicleIds = @($scenario.vehicles | ForEach-Object { $_.id })
if (($vehicleIds | Sort-Object -Unique).Count -ne 3) { throw '车辆 ID 不唯一。' }
$goalIds = @($scenario.vehicles | ForEach-Object { $_.summon_goal.id })
if (($goalIds | Sort-Object -Unique).Count -ne 3) { throw '召集点 ID 不唯一。' }

foreach ($vehicle in $scenario.vehicles) {
    foreach ($field in @('x_m', 'y_m', 'yaw_rad')) {
        if ($null -eq $vehicle.start_pose.$field) { throw "$($vehicle.id) 缺少 start_pose.$field。" }
        if ($null -eq $vehicle.summon_goal.$field) { throw "$($vehicle.id) 缺少 summon_goal.$field。" }
    }
}

$poses = @($scenario.vehicles | ForEach-Object { $_.start_pose.yaw_rad })
if (($poses | Sort-Object -Unique).Count -lt 3) { throw '三辆车的初始航向必须不同。' }
$startPoses = @($scenario.vehicles | ForEach-Object { "$($_.start_pose.x_m),$($_.start_pose.y_m)" })
if (($startPoses | Sort-Object -Unique).Count -ne 3) { throw '三辆车的初始位置必须不同。' }
$goalPoses = @($scenario.vehicles | ForEach-Object { "$($_.summon_goal.x_m),$($_.summon_goal.y_m)" })
if (($goalPoses | Sort-Object -Unique).Count -ne 3) { throw '三个召集终点位置必须不同。' }
$goalYaws = @($scenario.vehicles | ForEach-Object { $_.summon_goal.yaw_rad })
if (($goalYaws | Sort-Object -Unique).Count -ne 3) { throw '三个召集终点航向必须不同。' }

$svg = Join-Path $outputDir 'scenario_001.svg'
if (-not (Test-Path -LiteralPath $svg)) { throw "缺少场景可视化图: $svg" }
$svgText = Get-Content -Raw -LiteralPath $svg
foreach ($id in $vehicleIds) {
    if ($svgText -notmatch [regex]::Escape($id)) { throw "可视化图缺少 $id。" }
}

Write-Output 'PASS: scenario_001 配置检查通过。'
Write-Output "车辆数: $($scenario.vehicles.Count)"
Write-Output "召集点数: $($goalIds.Count)"
Write-Output "边界顶点数: $($mapConfig.boundary.vertices_xy.Count)"
Write-Output '路径搜索: disabled (按 Day 1 要求)'
