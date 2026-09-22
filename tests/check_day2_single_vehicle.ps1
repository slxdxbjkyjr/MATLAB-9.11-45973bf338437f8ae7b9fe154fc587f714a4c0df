param(
    [string]$Root = 'F:\study\postgraduate\联培\路径规划论文\代码\辅助\9.11'
)

$ErrorActionPreference = 'Stop'
$singleDir = Join-Path $Root 'matlab\single_vehicle'
$required = @(
    'summon_vehicle_state.m',
    'summon_map.m',
    'summon_hybrid_astar.m',
    'summon_collision_check.m',
    'summon_plot_path.m',
    'run_single_vehicle_demo.m'
)
foreach ($name in $required) {
    $path = Join-Path $singleDir $name
    if (-not (Test-Path -LiteralPath $path)) { throw "缺少 Day 2 文件: $path" }
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch '%') { throw "$name 缺少中文注释。" }
}

$planner = Get-Content -Raw -LiteralPath (Join-Path $Root 'config\planner_config.json') | ConvertFrom-Json
$scenario = Get-Content -Raw -LiteralPath (Join-Path $Root 'config\scenario_001.json') | ConvertFrom-Json
if ($planner.path_search_enabled -ne $false) { throw 'Day 2 基础配置必须保持 path_search_enabled=false。' }
if ($planner.search.max_expansions -le 0) { throw '缺少 max_expansions。' }
if ($scenario.vehicles.Count -lt 1) { throw 'scenario_001 没有车辆。' }

$hybrid = Get-Content -Raw -LiteralPath (Join-Path $singleDir 'summon_hybrid_astar.m')
foreach ($symbol in @('HybridAStar_Update','CalcNextNode','getFinalPath','heuristicCost','getTotalCost')) {
    if ($hybrid -notmatch $symbol) { throw "缺少核心函数: $symbol" }
}
foreach ($symbol in @('path.x','path.y','path.theta','path.direction','path.valid','path.error_code','path.search_statistics')) {
    if ($hybrid -notmatch [regex]::Escape($symbol)) { throw "缺少统一输出字段: $symbol" }
}

Write-Output 'PASS: Day 2 MATLAB 文件和接口静态检查通过。'
Write-Output "文件数: $($required.Count)"
Write-Output 'MATLAB 执行: 由外部运行环境决定，静态检查不等同于仿真通过。'
