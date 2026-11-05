@echo off
rem ===========================================================================
rem  log_print launcher - uses the bundled runtime, no system Python needed.
rem  Usage:
rem    run.bat                      plot all 25 figures
rem    run.bat --doctor             environment / data self-check
rem    run.bat --only 1-3           plot selected figures
rem    run.bat --log-dir D:\logs    read logs from another folder
rem    run.bat --out-dir D:\out     write figures to another folder
rem ===========================================================================
setlocal
chcp 65001 >nul

set "HERE=%~dp0"
set "APP=%HERE%.."
set "PY=%APP%\.runtime\python.exe"

if not exist "%PY%" (
    echo [ERROR] runtime not installed.
    echo         run "%HERE%install.bat" first.
    exit /b 1
)

rem keep matplotlib cache and any temp files inside the program folder
set "MPLCONFIGDIR=%APP%\.runtime\mplconfig"
set "PYTHONIOENCODING=utf-8"
set "PYTHONDONTWRITEBYTECODE=1"

"%PY%" "%APP%\plot_all.py" %*
exit /b %errorlevel%
