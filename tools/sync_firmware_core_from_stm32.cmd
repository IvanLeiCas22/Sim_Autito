@echo off
setlocal

rem Copies the portable firmware navigation core from the STM32 repository into this simulator repository.
rem Usage:
rem   tools\sync_firmware_core_from_stm32.cmd C:\Users\GAMING\Desktop\MICROCONTROLADORES\MICROCONTROLADORES-STM32

if "%~1"=="" (
    echo Usage: %~nx0 ^<STM32_REPO_ROOT^>
    echo Example: %~nx0 C:\Users\GAMING\Desktop\MICROCONTROLADORES\MICROCONTROLADORES-STM32
    exit /b 1
)

set "STM32_ROOT=%~1"
set "DEST=%~dp0..\firmware_core"

if not exist "%STM32_ROOT%\Core" (
    echo ERROR: STM32 repo root does not contain Core\
    echo Given: "%STM32_ROOT%"
    exit /b 1
)

if not exist "%DEST%" mkdir "%DEST%"

echo Copying portable firmware core from:
echo   %STM32_ROOT%
echo to:
echo   %DEST%
echo.

rem These files are expected after the STM32 project is refactored to expose a portable core.
rem Missing files are reported but do not stop the script, so the list can evolve incrementally.
call :copy_if_exists "Core\Src\app_nav.c"              "app_nav.c"
call :copy_if_exists "Core\Inc\app_nav.h"              "app_nav.h"
call :copy_if_exists "Core\Inc\app_nav_types.h"        "app_nav_types.h"
call :copy_if_exists "Core\Inc\app_nav_config.h"       "app_nav_config.h"
call :copy_if_exists "Core\Inc\app_nav_debug.h"        "app_nav_debug.h"
call :copy_if_exists "Core\Src\app_nav_supervisor.c"   "app_nav_supervisor.c"
call :copy_if_exists "Core\Inc\app_nav_supervisor.h"   "app_nav_supervisor.h"
call :copy_if_exists "Core\Src\app_maze.c"             "app_maze.c"
call :copy_if_exists "Core\Inc\app_maze.h"             "app_maze.h"
call :copy_if_exists "Core\Inc\app_maze_types.h"       "app_maze_types.h"
call :copy_if_exists "Core\Src\pid_controller.c"       "pid_controller.c"
call :copy_if_exists "Core\Inc\pid_controller.h"       "pid_controller.h"
call :copy_if_exists "Core\Src\app_find_cells_policy.c"       "app_find_cells_policy.c"
call :copy_if_exists "Core\Inc\app_find_cells_policy.h"       "app_find_cells_policy.h"

echo.
echo Done. Review git diff before committing.
exit /b 0

:copy_if_exists
set "SRC_REL=%~1"
set "DST_NAME=%~2"
if exist "%STM32_ROOT%\%SRC_REL%" (
    copy /Y "%STM32_ROOT%\%SRC_REL%" "%DEST%\%DST_NAME%" >nul
    echo copied: %SRC_REL% -^> firmware_core\%DST_NAME%
) else (
    echo missing: %SRC_REL%
)
exit /b 0
