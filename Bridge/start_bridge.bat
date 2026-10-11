@echo off
:: Lệnh bắt buộc để hiển thị tiếng Việt có dấu
chcp 65001 > nul
REM ===========================================================
REM  DroneTwin - Khởi động Python bridge
REM  File này cần được đặt cùng thư mục với mavlink_to_unreal.py
REM ===========================================================
setlocal
cd /d "%~dp0"

echo.
echo  === DroneTwin Bridge ===
echo.

REM --- Kiểm tra Python ---
python --version >nul 2>&1
if errorlevel 1 (
    echo [X] Không tìm thấy Python trong PATH.
    echo     Truy cập python.org để tải Python nếu chưa cài và nhớ chọn "Add Python to PATH".
    pause
    exit /b 1
)

REM --- Kiểm tra pymavlink ---
python -c "import pymavlink" >nul 2>&1
if errorlevel 1 (
    echo [!] Chưa có pymavlink. Đang cài...
    python -m pip install --upgrade pip
    python -m pip install pymavlink
    if errorlevel 1 (
        echo [X] Cài pymavlink thất bại.
        pause
        exit /b 1
    )
)

REM --- Quét cổng SITL ---
python mavlink_to_unreal.py --scan

echo.
echo  Chọn chế độ:
echo    [1] Demo bay vòng tròn quanh Bách Khoa (không cần Mission Planner - dùng để kiểm tra Unreal Engine)
echo    [2] Kết nối SITL thật  (Mission Planner phải đang chạy SIMULATION)
echo    [3] Thoát
echo.
set /p MODE="  Nhập 1, 2 hoặc 3: "

if "%MODE%"=="1" goto DEMO
if "%MODE%"=="2" goto REAL
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
