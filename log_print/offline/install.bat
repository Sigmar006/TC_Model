@echo off
rem ===========================================================================
rem  log_print offline installer
rem  Installs the bundled Python runtime + wheels into ..\.runtime
rem  No network, no admin rights, no registry changes.
rem  Safe to run twice: an existing runtime is reused.
rem ===========================================================================
setlocal EnableExtensions
chcp 65001 >nul

set "HERE=%~dp0"
set "APP=%HERE%.."
set "RTP=%APP%\.runtime"
set "PY=%RTP%\python.exe"
set "SP=%RTP%\Lib\site-packages"
set "WHEELS=%HERE%wheels"
set "ZIP=%HERE%runtime\python-3.12.8-embed-amd64.zip"
for %%I in ("%APP%") do set "APPN=%%~fI"

echo ============================================================
echo  log_print : offline install
echo  app dir  : %APPN%
echo ============================================================

if not exist "%ZIP%"   ( echo [ERROR] missing "%ZIP%" & goto :fail )
if not exist "%WHEELS%\" ( echo [ERROR] missing "%WHEELS%" & goto :fail )

rem ---------------------------------------------------------------- 1. runtime
if exist "%PY%" (
    echo [1/4] python runtime already installed, reusing
) else (
    echo [1/4] extracting python runtime ...
    if not exist "%RTP%" mkdir "%RTP%"
    tar -xf "%ZIP%" -C "%RTP%" 2>nul
    if not exist "%PY%" (
        powershell -NoProfile -ExecutionPolicy Bypass -Command "Expand-Archive -LiteralPath '%ZIP%' -DestinationPath '%RTP%' -Force" 2>nul
    )
    if not exist "%PY%" ( echo [ERROR] cannot extract "%ZIP%" & goto :fail )
)

rem ---------------------------------------------------------------- 2. site-packages
if not exist "%SP%" mkdir "%SP%"
"%PY%" -c "import pathlib,sys;v=sys.version_info;p=pathlib.Path(sys.executable).with_name(f'python{v[0]}{v[1]}._pth');p.write_text(f'python{v[0]}{v[1]}.zip\n.\nLib\\site-packages\nimport site\n',encoding='utf-8')"
if errorlevel 1 ( echo [ERROR] cannot patch the ._pth file & goto :fail )

rem ---------------------------------------------------------------- 3. pip
if exist "%SP%\pip\__init__.py" goto :pip_ready
echo [2/4] bootstrapping pip ...
set "PIPWHL="
for %%F in ("%WHEELS%\pip-*.whl") do if not defined PIPWHL set "PIPWHL=%%~fF"
if not defined PIPWHL ( echo [ERROR] no pip-*.whl in "%WHEELS%" & goto :fail )
"%PY%" -m zipfile -e "%PIPWHL%" "%SP%"
if errorlevel 1 ( echo [ERROR] cannot unpack pip & goto :fail )
goto :pip_done
:pip_ready
echo [2/4] pip already present
:pip_done

rem ---------------------------------------------------------------- 4. wheels
echo [3/4] installing wheels from "%WHEELS%" ...
set "PIP_NO_INDEX=1"
set "PIP_DISABLE_PIP_VERSION_CHECK=1"
set "PIP_CONFIG_FILE=NUL"
"%PY%" -m pip install --no-index --no-warn-script-location --find-links "%WHEELS%" -r "%HERE%requirements.txt"
if errorlevel 1 ( echo [ERROR] pip install failed & goto :fail )

rem ---------------------------------------------------------------- 5. verify
echo [4/4] verifying ...
"%PY%" -c "import numpy,pandas,matplotlib,scipy,sys;print('   python     :',sys.version.split()[0]);print('   numpy      :',numpy.__version__);print('   pandas     :',pandas.__version__);print('   matplotlib :',matplotlib.__version__);print('   scipy      :',scipy.__version__)"
if errorlevel 1 ( echo [ERROR] dependency check failed & goto :fail )

echo.
echo Install OK.
echo   1. put the run logs into "%APPN%\log"  (or pass --log-dir)
echo   2. check  : "%HERE%run.bat" --doctor
echo   3. plot   : "%HERE%run.bat"
goto :eof

:fail
echo.
echo INSTALL FAILED - see the message above.
endlocal
exit /b 1
