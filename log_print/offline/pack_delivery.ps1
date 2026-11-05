# Packs only what the isolated machine needs into dist\log_print_offline.zip.
#   powershell -NoProfile -ExecutionPolicy Bypass -File pack_delivery.ps1
# Excluded: .venv, viz\.cache, __pycache__, log, log.zip, dist, .runtime

param([string]$Out = "")

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$app  = (Resolve-Path (Join-Path $here "..")).Path
if (-not $Out) {
    New-Item -ItemType Directory -Force -Path (Join-Path $app "dist") | Out-Null
    $Out = Join-Path $app "dist\log_print_offline.zip"
}

$stage = Join-Path $env:TEMP ("lp_pack_" + [guid]::NewGuid().ToString("N").Substring(0, 8))
$dst = Join-Path $stage "log_print"
New-Item -ItemType Directory -Force -Path $dst | Out-Null

Copy-Item (Join-Path $app "plot_all.py") $dst -Force
Copy-Item (Join-Path $app "viz") $dst -Recurse -Force
Copy-Item (Join-Path $app "offline") $dst -Recurse -Force
if (Test-Path (Join-Path $app "figures")) { Copy-Item (Join-Path $app "figures") $dst -Recurse -Force }

Get-ChildItem (Join-Path $dst "viz") -Recurse -Force |
    Where-Object { $_.Name -in @("__pycache__", ".cache") } |
    Remove-Item -Recurse -Force

if (Test-Path $Out) { Remove-Item $Out -Force }
Compress-Archive -Path $dst -DestinationPath $Out -CompressionLevel Optimal
Remove-Item $stage -Recurse -Force

"wrote {0} ({1:N1} MB)" -f $Out, ((Get-Item $Out).Length / 1MB)
