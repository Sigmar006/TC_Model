@echo off
REM ============================================================================
REM fetch_third_party.bat
REM ----------------------------------------------------------------------------
REM 无 Conan（PACKAGE_MANAGER=none）模式的第三方依赖拉取脚本。
REM
REM 本脚本把 5 个第三方库源码 git clone 到工作空间 third_party/ 下，
REM 锁定与 conanfile.py 完全一致的版本 tag。VendorPackages.cmake 会从该目录
REM 以 add_subdirectory / header-only 方式消费它们，无需 conan、无需 toolchain。
REM
REM 用法（在仓库根目录执行）：
REM   scripts\win\fetch_third_party.bat
REM
REM 前置条件：本机已安装 git，且 PATH 中可调用 git。
REM
REM 幂等：每个库若 third_party\<lib> 已存在则跳过，便于中断后重跑。
REM third_party/ 被 .gitignore 忽略，不会进入提交。
REM
REM 依赖矩阵（版本与 conanfile.py 对齐）：
REM   eigen       3.4.0    header-only    git clone  gitlab.com/libeigen/eigen
REM   nanoflann   v1.3.2   header-only    git clone  github.com/jlblancoc/nanoflann
REM   flatbuffers v1.12.0  header + flatc git clone  github.com/google/flatbuffers
REM   zlib        v1.3.1   编译库         git clone  github.com/madler/zlib
REM   sqlite3     3.53.4   编译库         amalgamation zip 下载 sqlite.org/2026（GitHub 镜像
REM                                       只含 src/*.c 拆分源，amalgamation 由 sqlite.org 发布包提供）
REM   boost       1.85.0   header-only    源码包下载 archives.boost.io（modular superproject
REM                                       无法直接 git clone 出预合并头；airborne_radar 使用
REM                                       boost/math/special_functions/gamma.hpp 与
REM                                       boost/pool/object_pool.hpp，故必须提供真实头）
REM ============================================================================

setlocal EnableDelayedExpansion

REM --- 定位仓库根目录（脚本位于 <root>\scripts\ 下） ---
set "SCRIPT_DIR=%~dp0"
pushd "%SCRIPT_DIR%.." >nul 2>&1
set "ROOT_DIR=%CD%"
popd >nul 2>&1
set "TP_DIR=%ROOT_DIR%\third_party"

echo [fetch_third_party] root    = %ROOT_DIR%
echo [fetch_third_party] dest    = %TP_DIR%
echo.

REM --- 校验 git 可用 ---
where git >nul 2>&1
if errorlevel 1 (
    echo [fetch_third_party] 错误: 未找到 git，请先安装 Git 并加入 PATH。 >&2
    exit /b 1
)

if not exist "%TP_DIR%" mkdir "%TP_DIR%"

REM --- 单库拉取子例程：参数 (子目录, git url, tag) ---
REM 若目标目录已存在且非空，跳过；否则浅克隆指定 tag。
goto :main

:fetch_one
set "_sub=%~1"
set "_url=%~2"
set "_tag=%~3"
set "_target=%TP_DIR%\%_sub%"

if exist "%_target%\.git" (
    echo [fetch_third_party] 跳过 %_sub%（已存在）
    exit /b 0
)
if exist "%_target%" (
    if not exist "%_target%\*" (
        rmdir "%_target%" 2>nul
    ) else (
        echo [fetch_third_party] 跳过 %_sub%（目录非空，非 git 仓库） >&2
        exit /b 0
    )
)

echo [fetch_third_party] 拉取 %_sub% @ %_tag%
git clone --depth 1 --branch "%_tag%" "%_url%" "%_target%"
if errorlevel 1 (
    echo [fetch_third_party] 错误: 拉取 %_sub% 失败（%_url%@%_tag%） >&2
    exit /b 1
)
exit /b 0

REM --- boost 源码包子例程：参数 (子目录, 版本号) ---
REM boost 是 modular superproject，源码包 boost_<ver_underscore>.zip 自带预合并的 boost/ 头目录
REM （无需 b2 headers）。用 PowerShell 下载 + Expand-Archive 解压（Windows 原生）。
REM 幂等：若 third_party\boost\boost 已存在则跳过。
:fetch_boost
set "_sub=%~1"
set "_ver=%~2"
set "_target=%TP_DIR%\%_sub%"
REM boost 源码包顶层目录名形如 boost_1_85_0（版本点替换为下划线）。
set "_ver_us=%_ver:.=_%"
set "_url=https://archives.boost.io/release/%_ver%/source/boost_%_ver_us%.zip"

if exist "%_target%\boost" (
    echo [fetch_third_party] 跳过 %_sub%（已存在）
    exit /b 0
)

echo [fetch_third_party] 下载 boost %_ver% 源码包（~205MB，来自 archives.boost.io）
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "$ProgressPreference='SilentlyContinue';" ^
  "$tmp=[System.IO.Path]::GetTempFileName() + '.zip';" ^
  "Invoke-WebRequest -Uri '%_url%' -OutFile $tmp;" ^
  "Expand-Archive -Path $tmp -DestinationPath '%TP_DIR%\_boost_extract' -Force;" ^
  "Move-Item -Path '%TP_DIR%\_boost_extract\boost_%_ver_us%' -Destination '%_target%' -Force;" ^
  "Remove-Item -Recurse -Force '%TP_DIR%\_boost_extract';" ^
  "Remove-Item $tmp"
if errorlevel 1 (
    echo [fetch_third_party] 错误: 下载/解压 boost %_ver% 失败（%_url%） >&2
    if exist "%TP_DIR%\_boost_extract" rmdir /s /q "%TP_DIR%\_boost_extract" 2>nul
    exit /b 1
)
exit /b 0

REM --- sqlite3 amalgamation 子例程：参数 (子目录, 版本号, 发布年份) ---
REM GitHub sqlite 镜像只含 src/*.c 拆分源，amalgamation（sqlite3.c 单文件）由 sqlite.org
REM 发布包提供。包名形如 sqlite-amalgamation-3530400.zip，置于 sqlite.org/<year>/ 下
REM （year 为该版本发布年份，随版本变化，故作为显式参数传入）。用 PowerShell 下载 +
REM Expand-Archive 解压（与 fetch_boost 同构）。幂等：若 third_party\sqlite\sqlite3.c
REM 已存在则跳过。目录名用 sqlite（与 VendorPackages.cmake 期望的 third_party/sqlite 一致）。
:fetch_sqlite
set "_sub=%~1"
set "_ver=%~2"
set "_year=%~3"
set "_target=%TP_DIR%\%_sub%"
REM amalgamation 包名版本号约定 XYYZZ00：3.53.4 -> 3530400（major 直取、minor/patch
REM 各补零到 2 位、末尾固定 00）。拆分版本段。
for /f "tokens=1,2,3 delims=." %%a in ("%_ver%") do (
    set "_major=%%a"
    set "_minor=%%b"
    set "_patch=%%c"
)
REM minor/patch 补零到 2 位（如 5 -> 05，53 -> 53）。
if 1%_minor% LSS 100 set "_minor=0%_minor%"
if 1%_patch% LSS 100 set "_patch=0%_patch%"
set "_pkgver=%_major%%_minor%%_patch%00"
set "_pkgdir=sqlite-amalgamation-%_pkgver%"
set "_url=https://www.sqlite.org/%_year%/sqlite-amalgamation-%_pkgver%.zip"

if exist "%_target%\sqlite3.c" (
    echo [fetch_third_party] 跳过 %_sub%（已存在）
    exit /b 0
)

echo [fetch_third_party] 下载 sqlite3 amalgamation %_ver%（来自 sqlite.org/%_year%）
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "$ProgressPreference='SilentlyContinue';" ^
  "$tmp=[System.IO.Path]::GetTempFileName() + '.zip';" ^
  "Invoke-WebRequest -Uri '%_url%' -OutFile $tmp;" ^
  "Expand-Archive -Path $tmp -DestinationPath '%TP_DIR%\_sqlite_extract' -Force;" ^
  "Move-Item -Path '%TP_DIR%\_sqlite_extract\%_pkgdir%' -Destination '%_target%' -Force;" ^
  "Remove-Item -Recurse -Force '%TP_DIR%\_sqlite_extract';" ^
  "Remove-Item $tmp"
if errorlevel 1 (
    echo [fetch_third_party] 错误: 下载/解压 sqlite3 amalgamation %_ver% 失败（%_url%） >&2
    if exist "%TP_DIR%\_sqlite_extract" rmdir /s /q "%TP_DIR%\_sqlite_extract" 2>nul
    exit /b 1
)
exit /b 0

:main
REM --- eigen 3.4.0（header-only） ---
call :fetch_one eigen      https://gitlab.com/libeigen/eigen.git        3.4.0
if errorlevel 1 exit /b 1

REM --- nanoflann v1.3.2（header-only） ---
call :fetch_one nanoflann  https://github.com/jlblancoc/nanoflann.git  v1.3.2
if errorlevel 1 exit /b 1

REM --- flatbuffers v1.12.0（运行时头 + 构建期 flatc） ---
call :fetch_one flatbuffers https://github.com/google/flatbuffers.git   v1.12.0
if errorlevel 1 exit /b 1

REM --- zlib v1.3.1（需编译库，含 gzopen 运行时 API） ---
call :fetch_one zlib       https://github.com/madler/zlib.git           v1.3.1
if errorlevel 1 exit /b 1

REM --- sqlite3 3.53.4（需编译库；amalgamation 来自 sqlite.org 发布包，2026 发布路径） ---
call :fetch_sqlite sqlite  3.53.4  2026
if errorlevel 1 exit /b 1

REM --- boost 1.85.0（header-only；airborne_radar 使用 boost/math 与 boost/pool） ---
call :fetch_boost boost    1.85.0
if errorlevel 1 exit /b 1

echo.
echo [fetch_third_party] 完成。third_party/ 已就绪：
dir /b "%TP_DIR%"
echo.
echo 下一步（无需 conan）：
echo   cmake --preset VisualStudio.14.0-amd64-none
echo   cmake --build --preset VisualStudio.14.0-amd64-none-release

endlocal
exit /b 0
