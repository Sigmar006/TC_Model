# 构建 RIR 三个工程 + 聚合库（Windows/msbuild）
#
# 背景：RIR 运行期分散在三个工程里，只编其中一个会把**旧对象**打进聚合库，
#   于是验收日志描述的二进制并不包含本次改动（2026-10-20 已两次踩中）：
#     rir_engine  : RirSignalDetector / RirTrackFilter / RirImmFilter / RirTrackLifecycle …
#     rir_core    : RirController / RirAcceptanceRecords / RirSession / RirPipeline …
#   1q_lib 按**对象文件**归档，所以三个工程都必须先重编，再编 1q_lib。
#
# 用法：
#   pwsh scripts/win/build_rir_libs.ps1              # 只编 v141（开发 + 单测）
#   pwsh scripts/win/build_rir_libs.ps1 -V120        # 只编 v120（交付/宿主）
#   pwsh scripts/win/build_rir_libs.ps1 -All         # 两套都编
#   pwsh scripts/win/build_rir_libs.ps1 -All -Deploy # 再拷贝 1q.lib 到宿主并重建 Sensors.dll
#
param(
  [switch]$V120,
  [switch]$All,
  [switch]$Deploy
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$msbuild15 = 'C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\MSBuild\15.0\Bin\MSBuild.exe'
$msbuild12 = 'C:\Program Files (x86)\MSBuild\12.0\Bin\MSBuild.exe'
$args_common = @('/p:Configuration=Release', '/p:Platform=x64',
                 '/p:BuildProjectReferences=false', '/m', '/v:minimal', '/nologo')
$projects = @('rir_engine', 'rir_core')

function Build-Target([string]$msbuild, [string]$buildRoot, [string]$label) {
  foreach ($name in $projects) {
    $proj = Join-Path $root "build\$buildRoot\src\remote_identification_radar\$name.vcxproj"
    Write-Host "== $label $name =="
    & $msbuild $proj $args_common '/t:Build' 2>&1 |
      Select-String -Pattern 'error|vcxproj ->' | Select-Object -First 4
  }
  $lib = Join-Path $root "build\$buildRoot\src\1q_lib.vcxproj"
  Write-Host "== $label 1q_lib =="
  & $msbuild $lib $args_common '/t:Build' 2>&1 |
    Select-String -Pattern 'error|vcxproj ->' | Select-Object -First 3
}

if ($V120 -or $All) { Build-Target $msbuild12 'acceptance-v120-fosim' 'v120' }
if ((-not $V120) -or $All) { Build-Target $msbuild15 'VisualStudio.15.0-amd64' 'v141' }

if ($Deploy) {
  $src = Join-Path $root 'build\acceptance-v120-fosim\Release\lib\1q.lib'
  $dst = 'C:\software\workspace\1qfosim\BJ_10Y_WorkSpace\ModelSource\lib\x64\Release\1q.lib'
  Copy-Item $src $dst -Force
  Write-Host "== 部署 1q.lib -> 宿主 =="
  cmd /c 'C:\software\workspace\1qfosim\BJ_10Y_WorkSpace\ModelSource\build_x64_v120.cmd'
}

Write-Host '完成。提示：改过 1q 公共头时，需同步 ModelSource\include\1q\... 副本后宿主编译才认新接口。'
