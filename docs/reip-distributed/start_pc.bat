@echo off
REM Start REIP PC components
REM Run each command in a separate terminal window

echo Starting REIP Validation System...
echo.
echo Open 4 separate PowerShell/CMD windows and run:
echo.
echo Window 1 - Position Server:
echo   python pc\aruco_position_server.py
echo.
echo Window 2 - Visualizer:
echo   python pc\visualizer.py
echo.
echo Window 3 - Logger:
echo   python pc\logger.py my_experiment
echo.
echo Window 4 - Fault Injector:
echo   python pc\fault_inject.py
echo.
echo Then SSH to each robot:
echo   ssh pi@clanker-1.local
echo   python3 reip_node.py 1
echo.
pause
