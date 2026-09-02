@echo off
REM ===========================================================
REM  DroneTwin - khoi dong Python bridge
REM  Dat file nay canh mavlink_to_unreal.py
REM ===========================================================
setlocal
cd /d "%~dp0"

echo.
echo  === DroneTwin Bridge ===
echo.

REM --- Kiem tra Python ---
python --version >nul 2>&1
if errorlevel 1 (
    echo [X] Khong tim thay Python trong PATH.
    echo     Cai tu python.org va nho tick "Add Python to PATH".
    pause
    exit /b 1
)

REM --- Kiem tra pymavlink ---
python -c "import pymavlink" >nul 2>&1
if errorlevel 1 (
    echo [!] Chua co pymavlink. Dang cai...
    python -m pip install --upgrade pip
    python -m pip install pymavlink
    if errorlevel 1 (
        echo [X] Cai pymavlink that bai.
        pause
        exit /b 1
    )
)

REM --- Quet cong SITL ---
python mavlink_to_unreal.py --scan

echo.
echo  Chon che do:
echo    [1] Ket noi SITL that  (Mission Planner phai dang chay SIMULATION)
echo    [2] Demo bay vong tron (khong can Mission Planner - dung de test Unreal)
echo    [3] Thoat
echo.
set /p MODE="  Nhap 1, 2 hoac 3: "

if "%MODE%"=="1" goto REAL
if "%MODE%"=="2" goto DEMO
goto :eof

:REAL
echo.
python mavlink_to_unreal.py --source auto --unreal 127.0.0.1:14560
pause
goto :eof

:DEMO
echo.
python mavlink_to_unreal.py --demo --unreal 127.0.0.1:14560
pause
goto :eof
