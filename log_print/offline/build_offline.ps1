# Builds the offline bundle (run on a machine WITH internet):
#   powershell -NoProfile -ExecutionPolicy Bypass -File build_offline.ps1
#
# Produces:
#   runtime\python-3.12.8-embed-amd64.zip   Python runtime (no install needed)
#   wheels\*.whl                            all wheels incl. transitive deps
#
# The whole "offline" folder plus the program folder is then copied to the
# isolated machine, where install.bat does the rest without network access.

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$app  = Resolve-Path (Join-Path $here "..")

$py = Join-Path $app ".venv\Scripts\python.exe"
if (-not (Test-Path $py)) { $py = "python" }

$ver  = "3.12.8"
$name = "python-$ver-embed-amd64.zip"
$url  = "https://www.python.org/ftp/python/$ver/$name"

$runtime = Join-Path $here "runtime"
$wheels  = Join-Path $here "wheels"
New-Item -ItemType Directory -Force -Path $runtime, $wheels | Out-Null

# 1. Python runtime -----------------------------------------------------------
$zip = Join-Path $runtime $name
if (Test-Path $zip) {
    Write-Host "[1/3] runtime zip already present: $zip"
} else {
    Write-Host "[1/3] downloading $url"
    Invoke-WebRequest -Uri $url -OutFile $zip -UseBasicParsing
}

# 2. Wheels -------------------------------------------------------------------
Write-Host "[2/3] downloading wheels into $wheels"
& $py -m pip download --only-binary=:all: --platform win_amd64 `
      --python-version 3.12 --implementation cp --abi cp312 `
      --dest $wheels -r (Join-Path $here "requirements.txt")
if ($LASTEXITCODE -ne 0) { throw "pip download failed" }

# pip itself is needed to bootstrap the embedded runtime
& $py -m pip download --only-binary=:all: --no-deps --dest $wheels pip
if ($LASTEXITCODE -ne 0) { throw "pip download failed" }

# 3. Verify -------------------------------------------------------------------
Write-Host "[3/3] bundle contents"
Get-ChildItem $wheels -Filter *.whl | Sort-Object Name |
    ForEach-Object { "  {0,10:N1} MB  {1}" -f ($_.Length / 1MB), $_.Name }
$total = (Get-ChildItem $here -Recurse -File | Measure-Object Length -Sum).Sum / 1MB
Write-Host ("  bundle size: {0:N1} MB" -f $total)
Write-Host "done."
