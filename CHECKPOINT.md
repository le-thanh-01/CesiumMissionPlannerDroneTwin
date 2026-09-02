# DroneTwin — CHECKPOINT 02
## DJI Neo 2 Digital Twin — điều khiển HAI CHIỀU

**Trạng thái: hoạt động đầy đủ.**
Lái drone từ Unreal bằng bàn phím/tay cầm, ArduPilot SITL tính vật lý bay,
Cesium dựng địa hình thật, Mission Planner giám sát song song.

Ngày: 2026-09-02 · Môi trường: 1 máy Windows, không dùng WSL

---

## 1. Kiến trúc

```
        Unreal Engine (bàn phím / tay cầm Xbox)
                  │
                  │  UDP 14561 — gói CONTROL 32 byte
                  ▼
          mavlink_to_unreal.py
                  │
                  │  RC_CHANNELS_OVERRIDE / ARM / SET_MODE
                  ▼
          ArduCopter SITL  ◄──── TCP 5760 ────►  Mission Planner
                  │                              (giám sát, không bắt buộc)
                  │  telemetry (TCP 5763)
                  ▼
          mavlink_to_unreal.py
                  │
                  │  UDP 14560 — gói STATE 64 byte
                  ▼
        Unreal — ADronePawn — Cesium World Terrain
```

**Nguyên tắc thiết kế:** Unreal đóng vai *tay điều khiển*, không phải flight
controller. ArduPilot vẫn là bộ não — EKF, ổn định, mixing động cơ, failsafe đều
là thật. Nhờ vậy đây là digital twin đúng nghĩa chứ không phải game bay drone.

### Bảng cổng — tất cả trên `127.0.0.1`

| Cổng | Giao thức | Ai giữ | Ai kết nối |
|---|---|---|---|
| 5760 | TCP | SITL SERIAL0 | Mission Planner |
| 5762 | TCP | SITL SERIAL1 | dự phòng |
| 5763 | TCP | SITL SERIAL2 | Python bridge |
| 14560 | UDP | Unreal DronePawn | bridge → Unreal (telemetry) |
| 14561 | UDP | Python bridge | Unreal → bridge (điều khiển) |

Loopback không bị Windows Firewall lọc nên **không cần tạo rule firewall nào**.

---

## 2. Dựng lại trên máy trắng

Phần này dành cho người chưa từng chạy dự án. Những mục đánh dấu **[CẤU HÌNH]**
là thứ *không* tự động có sau khi tải — phải làm tay, và là chỗ hay bị bỏ sót nhất.

### 2.1 Phần mềm cần cài

| Phần mềm | Phiên bản đã kiểm chứng | Ghi chú |
|---|---|---|
| Windows | 10 / 11 | |
| Visual Studio 2019 Community | toolchain 14.29.30133 | xem 2.2 |
| Unreal Engine | 4.27.2 | qua Epic Games Launcher |
| Cesium for Unreal | 1.24.0 | qua Marketplace |
| Mission Planner | 1.3.83 | có SITL tích hợp, không cần cài ArduPilot riêng |
| Python | 3.8 trở lên | nhớ tick "Add Python to PATH" khi cài |

### 2.2 **[CẤU HÌNH]** Visual Studio 2019

Cài VS thôi chưa đủ. Mở **Visual Studio Installer → Modify** và bật:

- Workload: **Game development with C++**
- Individual components:
  - **Unreal Engine installer**
  - **Windows 10 SDK** (10.0.18362 trở lên)
  - **MSVC v142 — VS 2019 C++ x64/x86 build tools**

Thiếu những cái này thì UE báo "Failed to compile" mà không nói rõ lý do.

### 2.3 **[CẤU HÌNH]** Cesium ion token

Cesium World Terrain là dịch vụ streaming, cần tài khoản riêng — token của người
khác **không** đi kèm project.

1. Đăng ký miễn phí tại https://ion.cesium.com
2. Trong Unreal: panel **Cesium → Token → Use existing token**, dán token vào
3. Cần internet khi chạy. Bay lần đầu qua vùng mới sẽ khựng lúc tải tile

### 2.4 **[CẤU HÌNH]** Tham số ArduPilot

Ba tham số này **quyết định dự án chạy hay không**, và chúng nằm trong
`eeprom.bin` của Mission Planner, không nằm trong repo.

Đặt qua **Mission Planner → CONFIG → Full Parameter List** → sửa → **Write Params**:

| Tham số | Giá trị | Vì sao |
|---|---|---|
| `SYSID_MYGCS` | `255` | **Quan trọng nhất.** ArduPilot chỉ nhận `RC_CHANNELS_OVERRIDE` từ đúng sysid này. Sai giá trị → lệnh bị vứt lặng lẽ, không báo lỗi, drone không nhúc nhích |
| `DISARM_DELAY` | `0` | Tắt auto-disarm sau 10 giây. Chỉ nên làm trong mô phỏng |
| `FS_GCS_ENABLE` | `0` | Tránh failsafe khi bridge tạm ngắt |

Bridge mặc định dùng sysid `255`. Muốn giá trị khác thì chạy
`python mavlink_to_unreal.py --sysid <N>` và đặt `SYSID_MYGCS` bằng đúng `<N>`.

### 2.5 **[CẤU HÌNH]** Cesium origin phải khớp home của SITL

Mission Planner truyền `--home` trên dòng lệnh khi khởi động SITL, **ghi đè**
tham số `SIM_OPOS_*`. Đã thử sửa param, không ăn.

Nên phải chỉnh chiều ngược lại: chọn `CesiumGeoreference` trong World Outliner,
đặt Origin cho khớp toạ độ mà bridge in ra.

Giá trị đang dùng (mặc định của Mission Planner, ở Colorado, Mỹ):

```
Origin Latitude  =  39.4009998
Origin Longitude = -105.2573699
Origin Height    =  2163
```

Lệch nhiều → drone ra ngoài vùng chính xác của float 32-bit trong Unreal →
giật loạn hoặc biến mất. Đây **không** phải bug code.

Chênh mốc cao độ: MAVLink báo độ cao theo mực nước biển (MSL), Cesium tính theo
ellipsoid WGS84, lệch khoảng 17 m ở Colorado. Nếu drone lơ lửng hoặc chìm nửa
người, chỉnh `Origin Height` lên xuống vài chục đơn vị cho tới khi chạm đất đúng.

### 2.6 **[CẤU HÌNH]** Editor Preferences

**Edit → Editor Preferences → Performance → tắt "Use Less CPU when in Background"**

Không tắt thì mỗi lần alt-tab sang terminal, Unreal ngừng gửi lệnh điều khiển,
bridge nhả RC override và drone tự rơi ga giữa chừng.

### 2.7 Import model drone

`dji_neo_2.fbx` → Content Browser → Import:

| Option | Giá trị |
|---|---|
| Skeletal Mesh | Off |
| Combine Meshes | **On** (đã chọn — dễ dùng hơn, đổi lại là cánh quạt không quay được) |
| Import Uniform Scale | `1.0` hoặc `100.0` (UE dùng cm, FBX từ Blender thường theo mét) |
| Generate Missing Collision | Off |
| Import Materials / Textures | On |

Mũi drone phải hướng theo trục **+X**. Không đúng thì chỉnh Rotation của
`BodyMesh`, hoặc bù bằng `YawOffsetDeg`.

### 2.8 Build C++

```
1. Chép Source\DroneTwin\*.h, *.cpp, *.Build.cs vào project
2. Chuột phải DroneTwin.uproject → Generate Visual Studio project files
3. Mở DroneTwin.sln → build Development Editor | x64
```

Build lỗi lạ → đóng UE và VS, xoá `Binaries\`, `Intermediate\`, `.vs\`,
`DroneTwin.sln`, rồi làm lại từ bước 2. **Không bao giờ xoá** `Content\`,
`Config\`, `Source\`.

### 2.9 **[CẤU HÌNH]** Blueprint `BP_DJINeo2`

Đây là thứ **không thể tái tạo từ code** — `.uasset` là file nhị phân. Nếu mất,
dựng lại như sau (khoảng 10 phút):

1. Content Browser → chuột phải → Blueprint Class → All Classes → `DronePawn`
2. Chọn `BodyMesh` → gán static mesh `dji_neo_2`
3. Add Component → **Spring Arm**, attach vào `RootScene`:
   - Target Arm Length `400`
   - Socket Offset Z `100`
   - **Do Collision Test: TẮT** (nếu bật, spring arm va vào terrain Cesium và camera giật liên tục)
   - Use Pawn Control Rotation: tắt
   - Inherit Pitch: tắt · Inherit Roll: tắt · Inherit Yaw: **bật**
   - Enable Camera Lag: bật, Camera Lag Speed `5`
4. Add Component → **Camera**, attach vào Spring Arm
5. Chọn node gốc (Self) → Details:
   - `Georeference` = kéo `CesiumGeoreference` từ World Outliner vào
   - Pawn → **Auto Possess Player = Player 0**
6. Compile, Save, kéo vào level

**Tham số Tuning đã chốt:**

```
Yaw Offset Deg            = ______   <-- ĐIỀN GIÁ TRỊ THỰC TẾ VÀO ĐÂY
Location Interp Speed     = 15
Rotation Interp Speed     = 12
Throttle Rate             = 1.0
Stick Rise Rate           = 3.0
Stick Return Rate         = 6.0
Enable Control            = true
Use Gamepad               = true
Show Debug Overlay        = true
Listen Port               = 14560
Bridge Host               = 127.0.0.1
Bridge Control Port       = 14561
Propellers                = rỗng (mesh đã combine — đúng, không phải lỗi)
```

Cũng nên xoá actor `Floor` mặc định của template Blank khỏi level — nó là mặt
phẳng ca-rô che mất terrain Cesium.

### 2.10 Python

```
cd Bridge
pip install -r requirements.txt
```

---

## 3. Quy trình chạy hằng ngày

### Khởi động — đúng thứ tự

**1. Mission Planner** → tab SIMULATION → Multirotor → Stable
Đợi HUD chạy và hiện `GPS: 3D Fix` (~30 giây).

**2. Bridge** — một trong hai cách, hoàn toàn tương đương:

```
Bridge\start_bridge.bat        rồi chọn 1
```
hoặc
```
cd Bridge
python mavlink_to_unreal.py
```

`start_bridge.bat` chỉ là vỏ bọc gọi chính `mavlink_to_unreal.py`, thêm việc
kiểm tra Python, tự cài pymavlink và quét cổng. Dùng lệnh trực tiếp khi debug
vì thông báo lỗi rõ hơn và cửa sổ không tự đóng.

Phải thấy đủ ba dòng:
```
Tu dong tim thay SITL tai tcp:127.0.0.1:5763  (sysid=255)
Nhan lenh dieu khien tu Unreal tren UDP 14561
Gui telemetry sang Unreal tai ('127.0.0.1', 14560) @ 50 Hz
```
Để cửa sổ này chạy suốt buổi.

**3. Unreal** → Play → bấm chuột trái vào viewport để chắc chắn nhận phím.
Overlay phải hiện `LINK: OK` màu xanh.

### Bay — làm hết trong Unreal

| Phím | Tác dụng |
|---|---|
| `W` / `S` | Throttle lên / xuống — **giữ nguyên khi thả phím**, đúng như cần ga thật |
| `A` / `D` | Yaw trái / phải |
| `↑` `↓` `←` `→` | Pitch và Roll — tự về giữa khi thả, như cần lò xo |
| `Enter` | ARM |
| `Backspace` | DISARM |
| `1` … `6` | STABILIZE · ALT_HOLD · LOITER · GUIDED · RTL · LAND |

Tay cầm Xbox cắm vào dùng được ngay: cần trái dọc = throttle (tuyệt đối),
cần trái ngang = yaw, cần phải = roll/pitch, Start = ARM, Back = DISARM.
Không cần cấu hình Input mapping — code đọc phím trực tiếp qua PlayerController.

**Trình tự cất cánh:**
1. `3` → vào LOITER (giữ vị trí bằng GPS, dễ lái nhất)
2. Xác nhận overlay hiện `THR 0.00` — bridge từ chối ARM nếu throttle chưa về 0
3. `Enter` → overlay chuyển sang `ARMED`
4. Giữ `W` cho throttle vượt `0.55` → cất cánh
5. Mũi tên và `A`/`D` để bay
6. `6` (LAND) để hạ, chạm đất rồi `Backspace`

### Tắt — thứ tự quan trọng

```
Dừng Play trong Unreal
   → Ctrl+C ở cửa sổ bridge   (ĐỪNG bấm dấu X)
   → Disconnect trong Mission Planner
   → đóng cửa sổ đen của SITL
```

Bridge khi thoát bằng `Ctrl+C` sẽ gửi lệnh nhả RC override. Đóng bằng dấu X thì
lệnh đó không kịp gửi, SITL kẹt ở giá trị cần cuối cùng và lần chạy sau drone có
thể tự vọt lên. Gặp vậy thì restart SITL là hết.

---

## 4. Chẩn đoán

### Nguyên tắc chia đôi

```
python mavlink_to_unreal.py --demo
```

Bay vòng tròn giả lập, không cần Mission Planner, không cần cả pymavlink.

- Demo chạy đúng → phía Unreal/Cesium/Blueprint ổn, lỗi nằm ở MAVLink/SITL
- Demo cũng sai → lỗi ở phía Unreal

Lưu ý demo chỉ kiểm tra chiều telemetry, không kiểm tra chiều điều khiển.

### Đọc `RCin` — công cụ quan trọng nhất cho chiều điều khiển

Dòng telemetry của bridge có `RCin[roll pitch throttle yaw]` — đó là PWM mà
flight controller **thực sự nhận được**. Giữ `W` và nhìn số thứ ba:

- bò lên `2000` → override tới nơi, vấn đề nằm chỗ khác
- đứng im ở `1000` → override bị vứt, gần như chắc chắn do `SYSID_MYGCS` (mục 2.4)

### Bảng lỗi

| Triệu chứng | Nguyên nhân | Cách sửa |
|---|---|---|
| Build lỗi `<optional> available only with C++17` | Cesium native header cần C++17, UE 4.27 mặc định C++14 | `CppStandard = CppStandardVersion.Cpp17;` + `bEnableExceptions = true;` trong Build.cs |
| Play xong điều khiển quả cầu trên sàn | GameMode sinh DefaultPawn | SpringArm + Camera trong BP, Auto Possess Player = Player 0 |
| Overlay `THR 1.00` nhưng drone đứng im, telemetry `thr=0.00` | `SYSID_MYGCS` không khớp sysid của bridge | Đặt `SYSID_MYGCS = 255` |
| Arm xong 10s tự Disarm | `DISARM_DELAY` mặc định | Đặt `DISARM_DELAY = 0` |
| Bridge báo `Mat lenh tu Unreal` khi alt-tab | Unreal giảm tốc khi mất focus | Tắt "Use Less CPU when in Background" |
| Bridge in `he thong 0 component 0` | Broadcast ID, vô hại | Bridge tự ép về `1`, bỏ qua |
| `Chua co GLOBAL_POSITION_INT` | Chưa có GPS fix | Đợi ~30s |
| Fly To Here không làm drone bay | Chưa arm / chưa cất cánh | Fly To Here chỉ đặt đích, phải Guided → Arm → Takeoff trước |
| MP mất kết nối khi bridge chạy | Bridge chiếm 5760 | `--source tcp:127.0.0.1:5763` |
| Drone chìm dưới đất / lơ lửng | Lệch mốc MSL vs ellipsoid | Chỉnh `Origin Height` |
| Drone bay ngang / lùi | Mesh không hướng +X hoặc `YawOffsetDeg` sai | Chỉnh `YawOffsetDeg` lúc đang Play |
| Drone ở tít xa / biến mất | Cesium origin lệch home SITL | Mục 2.5 |
| ARM bị từ chối | Đọc tab **Messages** của MP, hoặc dòng `[SITL]` trong terminal bridge | |

### Tuỳ chọn dòng lệnh

```
python mavlink_to_unreal.py --scan          # xem cổng SITL nào đang mở
python mavlink_to_unreal.py --demo          # bay vòng tròn giả
python mavlink_to_unreal.py --no-control    # chỉ xem, không nhận lệnh từ Unreal
python mavlink_to_unreal.py --sysid 254     # đổi sysid (nhớ đổi SYSID_MYGCS theo)
python mavlink_to_unreal.py --force-arm     # bỏ qua pre-arm check (chỉ trong sim)
python mavlink_to_unreal.py --quiet         # không in telemetry
```

---

## 5. Giao ước gói tin

Hai chiều dùng hai struct khác nhau, đều `#pragma pack(1)` và little-endian.
Sửa field nào phải sửa **đồng thời** ở `DronePawn.h` và `mavlink_to_unreal.py`,
nếu không `static_assert` bên C++ hoặc `assert` bên Python sẽ nổ ngay khi chạy.
Đây là chủ ý — thà nổ sớm còn hơn nhận dữ liệu rác âm thầm.

### STATE — bridge → Unreal (UDP 14560), 64 byte, magic `0x314A4A44`

| Offset | Kiểu | Tên | Đơn vị |
|---|---|---|---|
| 0 | uint32 | Magic | `0x314A4A44` ('DJI1') |
| 4 | uint32 | Seq | số thứ tự, để bỏ gói đến trễ |
| 8 | double | Lat | độ, WGS84 |
| 16 | double | Lon | độ, WGS84 |
| 24 | double | AltMSL | mét trên mực nước biển |
| 32 | float | Roll | độ, dương = nghiêng phải |
| 36 | float | Pitch | độ, dương = ngóc mũi lên |
| 40 | float | Yaw | độ, heading la bàn (0 = Bắc) |
| 44 | float | Vx | m/s, NED North |
| 48 | float | Vy | m/s, NED East |
| 52 | float | Vz | m/s, NED Down |
| 56 | float | Throttle | 0.0 – 1.0 |
| 60 | uint8 | Armed | 0 / 1 |
| 61 | uint8 | Mode | custom_mode của ArduCopter |
| 62 | uint16 | Pad | căn cho tròn 64 byte |

### CONTROL — Unreal → bridge (UDP 14561), 32 byte, magic `0x324A4A44`

| Offset | Kiểu | Tên | Đơn vị |
|---|---|---|---|
| 0 | uint32 | Magic | `0x324A4A44` ('DJI2') |
| 4 | uint32 | Seq | |
| 8 | float | Roll | -1.0 – 1.0, dương = nghiêng phải |
| 12 | float | Pitch | -1.0 – 1.0, dương = chúi mũi về trước |
| 16 | float | Yaw | -1.0 – 1.0, dương = xoay phải |
| 20 | float | Throttle | 0.0 – 1.0 |
| 24 | uint8 | ArmRequest | 0 không, 1 arm, 2 disarm |
| 25 | uint8 | ModeRequest | 255 = không đổi, còn lại là custom_mode |
| 26 | uint16 | Pad | |
| 28 | uint32 | Reserved | để dành cho gimbal / camera |

**Quy đổi sang PWM** (làm trong bridge): cần `-1..1` → `1000..2000`,
throttle `0..1` → `1000..2000`. RCMAP mặc định của ArduCopter là
CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw. **CH2 bị đảo dấu** vì trong
ArduCopter, CH2 thấp = chúi mũi về trước.

**RC override phải gửi liên tục ở 25 Hz**, không phải gửi một lần. Ngừng gửi thì
ArduPilot quay về dùng RC mô phỏng sau vài giây. Bridge tự nhả override khi mất
liên lạc với Unreal quá 1 giây, và nhả lần cuối lúc thoát.

---

## 6. Cấu trúc thư mục

```
D:\UnrealProjects\DroneTwin\
├── DroneTwin.uproject
├── CHECKPOINT.md                    <- file này
├── .gitignore
│
├── Config\                          (UE tự sinh — COMMIT)
│   ├── DefaultEngine.ini
│   ├── DefaultGame.ini
│   └── DefaultInput.ini
│
├── Content\                         (COMMIT — không tái tạo được từ code)
│   ├── BP_DJINeo2.uasset            <- Blueprint drone, chứa toàn bộ tuning
│   ├── dji_neo_2.uasset             <- static mesh đã combine
│   ├── tripo_material_*.uasset
│   └── Maps\
│       └── OriginMap.umap           <- level: CesiumGeoreference, CesiumSunSky,
│                                       Cesium World Terrain, Cesium OSM Buildings
│
├── Source\
│   ├── DroneTwin.Target.cs          (UE tự sinh)
│   ├── DroneTwinEditor.Target.cs    (UE tự sinh)
│   └── DroneTwin\
│       ├── DroneTwin.Build.cs       <- ĐÃ SỬA: C++17 + Cesium
│       ├── DroneTwin.cpp            (UE tự sinh)
│       ├── DroneTwin.h              (UE tự sinh)
│       ├── DroneTwinGameModeBase.*  (UE tự sinh, không dùng)
│       ├── DronePawn.h              <- TỰ VIẾT
│       └── DronePawn.cpp            <- TỰ VIẾT
│
├── Bridge\                          <- TỰ VIẾT, không thuộc UE
│   ├── mavlink_to_unreal.py
│   ├── start_bridge.bat
│   └── requirements.txt
│
├── Binaries\                        (build sinh ra — XOÁ ĐƯỢC)
├── Intermediate\                    (build sinh ra — XOÁ ĐƯỢC)
├── Saved\                           (log, autosave — XOÁ ĐƯỢC)
└── .vs\                             (Visual Studio — XOÁ ĐƯỢC)
```

### Nằm ngoài repo nhưng phải backup

```
%USERPROFILE%\Documents\Mission Planner\sitl\
├── ArduCopter.exe        (MP tự tải)
└── eeprom.bin            <- SYSID_MYGCS, DISARM_DELAY, FS_GCS_ENABLE
```

Nếu Mission Planner tải lại SITL, file này có thể bị ghi đè và bạn sẽ gặp lại
lỗi "drone không nhúc nhích" mà không hiểu vì sao. Xem lại mục 2.4.

Cesium ion token cũng lưu theo máy, không nằm trong repo — xem mục 2.3.

---

## 7. Toàn bộ source code

Bảy file dưới đây là tất cả những gì phải tự viết. Mọi thứ khác trong project
đều do Unreal sinh ra tự động.

### `Source/DroneTwin/DronePawn.h`

Struct gói STATE (64B) và CONTROL (32B), khai báo class ADronePawn.  
*237 dòng*

```cpp
// DronePawn.h — Digital Twin DJI Neo 2  (v2: dieu khien 2 chieu)
// Unreal Engine 4.27.2 + Cesium for Unreal 1.24.0

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Networking.h"
#include "Sockets.h"
#include "Common/UdpSocketReceiver.h"
#include "HAL/CriticalSection.h"
#include "DronePawn.generated.h"

class ACesiumGeoreference;

// ---------------------------------------------------------------------------
// Goi TELEMETRY: bridge -> Unreal (UDP 14560)
// ---------------------------------------------------------------------------
#define DRONE_STATE_MAGIC 0x314A4A44u   // 'DJI1'

#pragma pack(push, 1)
struct FDroneStatePacket
{
	uint32 Magic;
	uint32 Seq;
	double Lat;         // do (WGS84)
	double Lon;         // do (WGS84)
	double AltMSL;      // met tren muc nuoc bien
	float  Roll;        // do, duong = nghieng phai
	float  Pitch;       // do, duong = ngoc mui len
	float  Yaw;         // do, heading la ban (0 = Bac)
	float  Vx;          // m/s, NED North
	float  Vy;          // m/s, NED East
	float  Vz;          // m/s, NED Down
	float  Throttle;    // 0.0 .. 1.0
	uint8  Armed;
	uint8  Mode;
	uint16 Pad;
};
#pragma pack(pop)

static_assert(sizeof(FDroneStatePacket) == 64, "State packet phai dung 64 byte");

// ---------------------------------------------------------------------------
// Goi CONTROL: Unreal -> bridge (UDP 14561)
// ---------------------------------------------------------------------------
#define DRONE_CONTROL_MAGIC 0x324A4A44u   // 'DJI2'

// ArmRequest
#define DRONE_ARM_NONE    0
#define DRONE_ARM_ARM     1
#define DRONE_ARM_DISARM  2

// ModeRequest: 255 = khong doi. Con lai la custom_mode cua ArduCopter.
#define DRONE_MODE_NONE      255
#define DRONE_MODE_STABILIZE 0
#define DRONE_MODE_ALT_HOLD  2
#define DRONE_MODE_GUIDED    4
#define DRONE_MODE_LOITER    5
#define DRONE_MODE_RTL       6
#define DRONE_MODE_LAND      9

#pragma pack(push, 1)
struct FDroneControlPacket
{
	uint32 Magic;
	uint32 Seq;
	float  Roll;         // -1.0 .. 1.0  (duong = nghieng phai)
	float  Pitch;        // -1.0 .. 1.0  (duong = chui mui ve truoc)
	float  Yaw;          // -1.0 .. 1.0  (duong = xoay phai)
	float  Throttle;     //  0.0 .. 1.0
	uint8  ArmRequest;
	uint8  ModeRequest;
	uint16 Pad;
	uint32 Reserved;     // de danh cho gimbal / camera sau nay
};
#pragma pack(pop)

static_assert(sizeof(FDroneControlPacket) == 32, "Control packet phai dung 32 byte");


UCLASS()
class DRONETWIN_API ADronePawn : public APawn
{
	GENERATED_BODY()

public:
	ADronePawn();

	virtual void Tick(float DeltaSeconds) override;

	// ---- Components ----
	UPROPERTY(VisibleAnywhere, Category = "Drone")
	USceneComponent* RootScene;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
	UStaticMeshComponent* BodyMesh;

	// Bo trong neu mesh da combine. Vong lap se tu bo qua.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone")
	TArray<UStaticMeshComponent*> Propellers;

	// ---- Cesium ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cesium")
	ACesiumGeoreference* Georeference = nullptr;

	// ---- Network ----
	// Cong nhan telemetry tu bridge
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
	int32 ListenPort = 14560;

	// Cong gui lenh dieu khien sang bridge
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
	FString BridgeHost = TEXT("127.0.0.1");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
	int32 BridgeControlPort = 14561;

	// ---- Control ----
	// Tat cai nay neu chi muon xem, khong muon lai tu Unreal.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
	bool bEnableControl = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
	float ControlSendHz = 25.f;

	// Toc do can roll/pitch/yaw day toi het bien (don vi/giay)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
	float StickRiseRate = 3.0f;

	// Toc do can tu ve giua khi tha phim
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
	float StickReturnRate = 6.0f;

	// Toc do thay doi throttle (don vi/giay). Throttle GIU nguyen khi tha phim.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
	float ThrottleRate = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
	bool bUseGamepad = true;

	// Hien thong so len goc man hinh khi Play
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
	bool bShowDebugOverlay = true;

	// ---- Tuning ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tuning")
	float YawOffsetDeg = -90.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tuning")
	float LocationInterpSpeed = 15.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tuning")
	float RotationInterpSpeed = 12.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tuning")
	float MaxPropSpinDegPerSec = 4000.f;

	// ---- Telemetry doc duoc tu Blueprint ----
	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
	bool bIsArmed = false;

	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
	float GroundSpeed = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
	float AltitudeMSL = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
	bool bLinkAlive = false;

	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
	uint8 CurrentMode = 0;

	// ---- Trang thai can dieu khien, doc duoc tu Blueprint de ve HUD ----
	UPROPERTY(BlueprintReadOnly, Category = "Control")
	float StickRoll = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Control")
	float StickPitch = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Control")
	float StickYaw = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Control")
	float StickThrottle = 0.f;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	// --- nhan telemetry ---
	void StartListening();
	void StopListening();
	void OnUdpDataReceived(const FArrayReaderPtr& Data, const FIPv4Endpoint& Endpoint);
	void ApplyTelemetry(float DeltaSeconds);

	// --- gui dieu khien ---
	void StartSending();
	void StopSending();
	void PollInput(float DeltaSeconds);
	void SendControl();
	void DrawOverlay();

	FVector GeoToUnreal(double Lon, double Lat, double HeightMeters) const;

	// Socket nhan
	FSocket* RecvSocket = nullptr;
	FUdpSocketReceiver* Receiver = nullptr;

	// Socket gui
	FSocket* SendSocket = nullptr;
	TSharedPtr<FInternetAddr> BridgeAddr;

	mutable FCriticalSection StateMutex;
	FDroneStatePacket LatestPacket;
	bool bHasPacket = false;
	uint32 LastSeq = 0;
	double LastPacketTime = 0.0;

	float PropAngle = 0.f;

	// Trang thai lenh dang cho gui
	uint8 PendingArm = DRONE_ARM_NONE;
	uint8 PendingMode = DRONE_MODE_NONE;
	float PendingArmTimer = 0.f;
	float PendingModeTimer = 0.f;

	// Chong lap khi giu phim
	bool bArmKeyWasDown = false;
	bool bDisarmKeyWasDown = false;
	bool bModeKeyWasDown[6] = { false, false, false, false, false, false };

	uint32 ControlSeq = 0;
	float ControlSendAccum = 0.f;
};
```

---

### `Source/DroneTwin/DronePawn.cpp`

UDP receiver trên thread riêng, đọc bàn phím/tay cầm, gửi lệnh điều khiển, chuyển toạ độ Cesium, vẽ overlay.  
*450 dòng*

```cpp
// DronePawn.cpp  (v2: dieu khien 2 chieu)

#include "DronePawn.h"
#include "CesiumGeoreference.h"
#include "Kismet/GameplayStatics.h"
#include "SocketSubsystem.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"

ADronePawn::ADronePawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	RootScene = CreateDefaultSubobject<USceneComponent>(TEXT("RootScene"));
	SetRootComponent(RootScene);

	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	BodyMesh->SetupAttachment(RootScene);
	BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BodyMesh->SetMobility(EComponentMobility::Movable);

	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void ADronePawn::BeginPlay()
{
	Super::BeginPlay();

	if (!Georeference)
	{
		TArray<AActor*> Found;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ACesiumGeoreference::StaticClass(), Found);
		if (Found.Num() > 0)
		{
			Georeference = Cast<ACesiumGeoreference>(Found[0]);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("[DronePawn] Khong tim thay CesiumGeoreference trong level!"));
		}
	}

	StartListening();

	if (bEnableControl)
	{
		StartSending();
	}
}

void ADronePawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopListening();
	StopSending();
	Super::EndPlay(EndPlayReason);
}

// ===========================================================================
//  NHAN TELEMETRY
// ===========================================================================

void ADronePawn::StartListening()
{
	FIPv4Endpoint Endpoint(FIPv4Address::Any, static_cast<uint16>(ListenPort));

	RecvSocket = FUdpSocketBuilder(TEXT("DroneTwinRecv"))
		.AsNonBlocking()
		.AsReusable()
		.BoundToEndpoint(Endpoint)
		.WithReceiveBufferSize(1 * 1024 * 1024);

	if (!RecvSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("[DronePawn] Khong mo duoc UDP socket tren port %d"), ListenPort);
		return;
	}

	Receiver = new FUdpSocketReceiver(RecvSocket, FTimespan::FromMilliseconds(5), TEXT("DroneTwinRecvThread"));
	Receiver->OnDataReceived().BindUObject(this, &ADronePawn::OnUdpDataReceived);
	Receiver->Start();

	UE_LOG(LogTemp, Log, TEXT("[DronePawn] Dang lang nghe UDP tren port %d"), ListenPort);
}

void ADronePawn::StopListening()
{
	if (Receiver)
	{
		Receiver->Stop();
		delete Receiver;
		Receiver = nullptr;
	}

	if (RecvSocket)
	{
		RecvSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(RecvSocket);
		RecvSocket = nullptr;
	}
}

// LUU Y: chay tren thread rieng, khong duoc dung vao scene component o day.
void ADronePawn::OnUdpDataReceived(const FArrayReaderPtr& Data, const FIPv4Endpoint& Endpoint)
{
	if (!Data.IsValid() || Data->Num() < static_cast<int32>(sizeof(FDroneStatePacket)))
	{
		return;
	}

	FDroneStatePacket Packet;
	FMemory::Memcpy(&Packet, Data->GetData(), sizeof(FDroneStatePacket));

	if (Packet.Magic != DRONE_STATE_MAGIC)
	{
		return;
	}

	FScopeLock Lock(&StateMutex);

	const bool bNewer = (Packet.Seq > LastSeq) || (LastSeq - Packet.Seq > 1000);
	if (bHasPacket && !bNewer)
	{
		return;
	}

	LatestPacket = Packet;
	LastSeq = Packet.Seq;
	bHasPacket = true;
	LastPacketTime = FPlatformTime::Seconds();
}

// ===========================================================================
//  GUI DIEU KHIEN
// ===========================================================================

void ADronePawn::StartSending()
{
	SendSocket = FUdpSocketBuilder(TEXT("DroneTwinSend")).AsReusable();

	if (!SendSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("[DronePawn] Khong mo duoc socket gui dieu khien"));
		return;
	}

	ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	BridgeAddr = SS->CreateInternetAddr();

	bool bAddrValid = false;
	BridgeAddr->SetIp(*BridgeHost, bAddrValid);
	BridgeAddr->SetPort(BridgeControlPort);

	if (!bAddrValid)
	{
		UE_LOG(LogTemp, Error, TEXT("[DronePawn] Dia chi bridge khong hop le: %s"), *BridgeHost);
		StopSending();
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("[DronePawn] Gui dieu khien toi %s:%d"), *BridgeHost, BridgeControlPort);
}

void ADronePawn::StopSending()
{
	if (SendSocket)
	{
		SendSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SendSocket);
		SendSocket = nullptr;
	}
	BridgeAddr.Reset();
}

void ADronePawn::PollInput(float DeltaSeconds)
{
	APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC)
	{
		return;
	}

	// --- Doc ban phim thanh muc tieu -1..1 ---
	float TargetRoll = 0.f;
	float TargetPitch = 0.f;
	float TargetYaw = 0.f;

	if (PC->IsInputKeyDown(EKeys::Right)) TargetRoll += 1.f;
	if (PC->IsInputKeyDown(EKeys::Left))  TargetRoll -= 1.f;
	if (PC->IsInputKeyDown(EKeys::Up))    TargetPitch += 1.f;   // chui mui ve truoc
	if (PC->IsInputKeyDown(EKeys::Down))  TargetPitch -= 1.f;
	if (PC->IsInputKeyDown(EKeys::D))     TargetYaw += 1.f;
	if (PC->IsInputKeyDown(EKeys::A))     TargetYaw -= 1.f;

	// --- Tay cam (neu co) de len tren ban phim ---
	if (bUseGamepad)
	{
		float LX = 0.f, LY = 0.f, RX = 0.f, RY = 0.f;
		PC->GetInputAnalogStickState(EControllerAnalogStick::CAS_LeftStick, LX, LY);
		PC->GetInputAnalogStickState(EControllerAnalogStick::CAS_RightStick, RX, RY);

		const float Dead = 0.15f;
		if (FMath::Abs(RX) > Dead) TargetRoll = RX;
		if (FMath::Abs(RY) > Dead) TargetPitch = RY;
		if (FMath::Abs(LX) > Dead) TargetYaw = LX;
		if (FMath::Abs(LY) > Dead)
		{
			// Can trai doc = throttle tuyet doi, giong tay dieu khien that
			StickThrottle = FMath::Clamp((LY + 1.f) * 0.5f, 0.f, 1.f);
		}
	}

	// --- Can tu ve giua khi tha, giong lo xo cua tay dieu khien that ---
	auto MoveStick = [&](float Current, float Target) -> float
	{
		const float Rate = (FMath::Abs(Target) > KINDA_SMALL_NUMBER) ? StickRiseRate : StickReturnRate;
		return FMath::FInterpConstantTo(Current, Target, DeltaSeconds, Rate);
	};

	StickRoll  = MoveStick(StickRoll, TargetRoll);
	StickPitch = MoveStick(StickPitch, TargetPitch);
	StickYaw   = MoveStick(StickYaw, TargetYaw);

	// --- Throttle GIU nguyen khi tha phim (khong tu ve 0) ---
	if (PC->IsInputKeyDown(EKeys::W)) StickThrottle += ThrottleRate * DeltaSeconds;
	if (PC->IsInputKeyDown(EKeys::S)) StickThrottle -= ThrottleRate * DeltaSeconds;
	StickThrottle = FMath::Clamp(StickThrottle, 0.f, 1.f);

	// --- Arm / Disarm (chi kich hoat luc bam xuong, khong lap khi giu) ---
	const bool bArmDown = PC->IsInputKeyDown(EKeys::Enter) || PC->IsInputKeyDown(EKeys::Gamepad_Special_Right);
	if (bArmDown && !bArmKeyWasDown)
	{
		PendingArm = DRONE_ARM_ARM;
		PendingArmTimer = 0.5f;
		UE_LOG(LogTemp, Log, TEXT("[DronePawn] Yeu cau ARM"));
	}
	bArmKeyWasDown = bArmDown;

	const bool bDisarmDown = PC->IsInputKeyDown(EKeys::BackSpace) || PC->IsInputKeyDown(EKeys::Gamepad_Special_Left);
	if (bDisarmDown && !bDisarmKeyWasDown)
	{
		PendingArm = DRONE_ARM_DISARM;
		PendingArmTimer = 0.5f;
		UE_LOG(LogTemp, Log, TEXT("[DronePawn] Yeu cau DISARM"));
	}
	bDisarmKeyWasDown = bDisarmDown;

	// --- Doi mode bang phim so 1..6 ---
	static const FKey ModeKeys[6] = {
		EKeys::One, EKeys::Two, EKeys::Three, EKeys::Four, EKeys::Five, EKeys::Six
	};
	static const uint8 ModeValues[6] = {
		DRONE_MODE_STABILIZE, DRONE_MODE_ALT_HOLD, DRONE_MODE_LOITER,
		DRONE_MODE_GUIDED, DRONE_MODE_RTL, DRONE_MODE_LAND
	};

	for (int32 i = 0; i < 6; ++i)
	{
		const bool bDown = PC->IsInputKeyDown(ModeKeys[i]);
		if (bDown && !bModeKeyWasDown[i])
		{
			PendingMode = ModeValues[i];
			PendingModeTimer = 0.5f;
			UE_LOG(LogTemp, Log, TEXT("[DronePawn] Yeu cau doi mode -> %d"), PendingMode);
		}
		bModeKeyWasDown[i] = bDown;
	}

	// Lenh duoc giu trong 0.5s de chiu duoc mat goi UDP
	PendingArmTimer = FMath::Max(0.f, PendingArmTimer - DeltaSeconds);
	if (PendingArmTimer <= 0.f) PendingArm = DRONE_ARM_NONE;

	PendingModeTimer = FMath::Max(0.f, PendingModeTimer - DeltaSeconds);
	if (PendingModeTimer <= 0.f) PendingMode = DRONE_MODE_NONE;
}

void ADronePawn::SendControl()
{
	if (!SendSocket || !BridgeAddr.IsValid())
	{
		return;
	}

	FDroneControlPacket Packet;
	FMemory::Memzero(&Packet, sizeof(Packet));

	Packet.Magic = DRONE_CONTROL_MAGIC;
	Packet.Seq = ControlSeq++;
	Packet.Roll = StickRoll;
	Packet.Pitch = StickPitch;
	Packet.Yaw = StickYaw;
	Packet.Throttle = StickThrottle;
	Packet.ArmRequest = PendingArm;
	Packet.ModeRequest = PendingMode;

	int32 BytesSent = 0;
	SendSocket->SendTo(reinterpret_cast<const uint8*>(&Packet), sizeof(Packet), BytesSent, *BridgeAddr);
}

// ===========================================================================
//  AP DUNG TELEMETRY
// ===========================================================================

FVector ADronePawn::GeoToUnreal(double Lon, double Lat, double HeightMeters) const
{
	if (!Georeference)
	{
		return FVector::ZeroVector;
	}

	// Cesium for Unreal 1.24.0 — thu tu tham so la (Longitude, Latitude, Height).
	return Georeference->InaccurateTransformLongitudeLatitudeHeightToUnreal(
		FVector(Lon, Lat, HeightMeters));

	// Neu compile bao loi "no member named ...", ten ham da doi giua cac minor
	// version cua plugin. Mo CesiumGeoreference.h va tim ham co chu
	// "LongitudeLatitudeHeight...Unreal". Cac ten da tung dung:
	//   InaccurateTransformLongitudeLatitudeHeightToUe(const FVector&)
	//   TransformLongitudeLatitudeHeightPositionToUnreal(const FVector&)
	//   TransformLongitudeLatitudeHeightToUnreal(const glm::dvec3&)
}

void ADronePawn::ApplyTelemetry(float DeltaSeconds)
{
	FDroneStatePacket Packet;
	double PacketTime = 0.0;
	{
		FScopeLock Lock(&StateMutex);
		if (!bHasPacket)
		{
			bLinkAlive = false;
			return;
		}
		Packet = LatestPacket;
		PacketTime = LastPacketTime;
	}

	bLinkAlive = (FPlatformTime::Seconds() - PacketTime) < 2.0;

	const FVector TargetLocation = GeoToUnreal(Packet.Lon, Packet.Lat, Packet.AltMSL);
	const FRotator TargetRotation(Packet.Pitch, Packet.Yaw + YawOffsetDeg, Packet.Roll);

	if (LocationInterpSpeed > KINDA_SMALL_NUMBER)
	{
		SetActorLocation(FMath::VInterpTo(GetActorLocation(), TargetLocation, DeltaSeconds, LocationInterpSpeed));
	}
	else
	{
		SetActorLocation(TargetLocation);
	}

	if (RotationInterpSpeed > KINDA_SMALL_NUMBER)
	{
		SetActorRotation(FMath::RInterpTo(GetActorRotation(), TargetRotation, DeltaSeconds, RotationInterpSpeed));
	}
	else
	{
		SetActorRotation(TargetRotation);
	}

	// Canh quat (bo qua neu mesh da combine, mang rong)
	const float Spin = FMath::Clamp(Packet.Throttle, 0.f, 1.f) * MaxPropSpinDegPerSec;
	PropAngle = FMath::Fmod(PropAngle + Spin * DeltaSeconds, 360.f);

	for (int32 i = 0; i < Propellers.Num(); ++i)
	{
		if (!Propellers[i]) continue;
		const float Dir = (i % 2 == 0) ? 1.f : -1.f;
		Propellers[i]->SetRelativeRotation(FRotator(0.f, PropAngle * Dir, 0.f));
	}

	bIsArmed = (Packet.Armed != 0);
	AltitudeMSL = static_cast<float>(Packet.AltMSL);
	GroundSpeed = FMath::Sqrt(Packet.Vx * Packet.Vx + Packet.Vy * Packet.Vy);
	CurrentMode = Packet.Mode;
}

// ===========================================================================
//  OVERLAY
// ===========================================================================

static FString ModeName(uint8 Mode)
{
	switch (Mode)
	{
	case DRONE_MODE_STABILIZE: return TEXT("STABILIZE");
	case DRONE_MODE_ALT_HOLD:  return TEXT("ALT_HOLD");
	case DRONE_MODE_GUIDED:    return TEXT("GUIDED");
	case DRONE_MODE_LOITER:    return TEXT("LOITER");
	case DRONE_MODE_RTL:       return TEXT("RTL");
	case DRONE_MODE_LAND:      return TEXT("LAND");
	default:                   return FString::Printf(TEXT("MODE %d"), Mode);
	}
}

void ADronePawn::DrawOverlay()
{
	if (!GEngine)
	{
		return;
	}

	const FColor LinkColor = bLinkAlive ? FColor::Green : FColor::Red;
	const FColor ArmColor = bIsArmed ? FColor::Green : FColor::Yellow;

	GEngine->AddOnScreenDebugMessage(101, 0.f, LinkColor,
		FString::Printf(TEXT("LINK: %s   MODE: %s   %s"),
			bLinkAlive ? TEXT("OK") : TEXT("MAT KET NOI"),
			*ModeName(CurrentMode),
			bIsArmed ? TEXT("ARMED") : TEXT("DISARMED")));

	GEngine->AddOnScreenDebugMessage(102, 0.f, FColor::Cyan,
		FString::Printf(TEXT("ALT: %.1f m    GS: %.1f m/s"), AltitudeMSL, GroundSpeed));

	GEngine->AddOnScreenDebugMessage(103, 0.f, FColor::White,
		FString::Printf(TEXT("THR %.2f  |  ROLL %+.2f  PITCH %+.2f  YAW %+.2f"),
			StickThrottle, StickRoll, StickPitch, StickYaw));

	GEngine->AddOnScreenDebugMessage(104, 0.f, ArmColor,
		TEXT("W/S throttle | A/D yaw | Mui ten roll+pitch | Enter=ARM | Backspace=DISARM | 1..6 doi mode"));
}

// ===========================================================================
//  TICK
// ===========================================================================

void ADronePawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (bEnableControl)
	{
		PollInput(DeltaSeconds);

		ControlSendAccum += DeltaSeconds;
		const float Interval = 1.f / FMath::Max(1.f, ControlSendHz);
		if (ControlSendAccum >= Interval)
		{
			ControlSendAccum = 0.f;
			SendControl();
		}
	}

	ApplyTelemetry(DeltaSeconds);

	if (bShowDebugOverlay)
	{
		DrawOverlay();
	}
}
```

---

### `Source/DroneTwin/DroneTwin.Build.cs`

Bật C++17 và exceptions cho Cesium, khai báo Networking/Sockets/CesiumRuntime.  
*39 dòng*

```csharp
// DroneTwin.Build.cs
// Doi "DroneTwin" thanh ten project that cua ban o ca ten file lan ten class.

using UnrealBuildTool;

public class DroneTwin : ModuleRules
{
	public DroneTwin(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		// ------------------------------------------------------------------
		// BAT BUOC khi include header cua Cesium.
		// Cesium native (CesiumGeospatial/Ellipsoid.h ...) dung std::optional,
		// von la C++17. UE 4.27 mac dinh bien dich module game bang C++14 nen
		// se bao: "The contents of <optional> are available only with C++17".
		// Hai dong duoi day mirror dung cau hinh cua module CesiumRuntime.
		// ------------------------------------------------------------------
		CppStandard = CppStandardVersion.Cpp17;
		bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",

			// Can cho FSocket / FUdpSocketBuilder / FUdpSocketReceiver
			"Networking",
			"Sockets",

			// Can cho ACesiumGeoreference
			"CesiumRuntime",
		});

		PrivateDependencyModuleNames.AddRange(new string[] { });
	}
}
```

---

### `Bridge/mavlink_to_unreal.py`

Cầu nối hai chiều. Telemetry MAVLink → Unreal, và lệnh Unreal → RC_CHANNELS_OVERRIDE/ARM/SET_MODE. Có --demo và --scan.  
*513 dòng*

```python
#!/usr/bin/env python3
"""
mavlink_to_unreal.py — cau noi HAI CHIEU giua MAVLink va Unreal Engine
Thiet ke cho: 1 may Windows duy nhat, khong can WSL/Linux.

    Unreal (ban phim / tay cam)
          |  UDP 14561  (goi CONTROL, 32 byte)
          v
      script nay  --> RC_CHANNELS_OVERRIDE / ARM / SET_MODE --> TCP 5763 --> SITL
          ^                                                                    |
          |  UDP 14560  (goi STATE, 64 byte)                                   |
          +-------------------- telemetry <------------------------------------+

Mission Planner van cam song song tren TCP 5760 de giam sat.

Cai dat:
    pip install pymavlink

Chay:
    python mavlink_to_unreal.py                 # hai chieu, tu do cong SITL
    python mavlink_to_unreal.py --no-control    # chi xem, khong nhan lenh tu Unreal
    python mavlink_to_unreal.py --demo          # bay vong tron gia, khong can MP
    python mavlink_to_unreal.py --scan          # xem cong SITL nao dang mo
"""

import argparse
import math
import socket
import struct
import sys
import time

# pymavlink chi can cho che do that. --demo va --scan chay duoc ma khong co no.
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


def require_pymavlink():
    if mavutil is None:
        print("Thieu pymavlink. Chay:  pip install pymavlink")
        sys.exit(1)


# ---------------------------------------------------------------------------
# Goi STATE: bridge -> Unreal. Khop voi FDroneStatePacket trong DronePawn.h
# ---------------------------------------------------------------------------
STATE_MAGIC = 0x314A4A44                # 'DJI1'
STATE_FMT = "<IIdddfffffffBBH"
STATE_SIZE = struct.calcsize(STATE_FMT)
assert STATE_SIZE == 64, f"State packet phai 64 byte, dang la {STATE_SIZE}"

# ---------------------------------------------------------------------------
# Goi CONTROL: Unreal -> bridge. Khop voi FDroneControlPacket trong DronePawn.h
# ---------------------------------------------------------------------------
CONTROL_MAGIC = 0x324A4A44              # 'DJI2'
CONTROL_FMT = "<IIffffBBHI"
CONTROL_SIZE = struct.calcsize(CONTROL_FMT)
assert CONTROL_SIZE == 32, f"Control packet phai 32 byte, dang la {CONTROL_SIZE}"

ARM_NONE, ARM_ARM, ARM_DISARM = 0, 1, 2
MODE_NONE = 255

MODE_NAMES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
              5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD"}

SEND_HZ = 50.0          # telemetry -> Unreal
RC_HZ = 25.0            # RC override -> SITL
CONTROL_TIMEOUT = 1.0   # khong nhan lenh qua ngan nay giay thi nha override

SITL_TCP_PORTS = [5763, 5762, 5760]
FALLBACK_UDP = "udpin:0.0.0.0:14551"
DEFAULT_HOME = (21.0278, 105.8342, 20.0)


class DroneState:
    """Gom telemetry tu nhieu message MAVLink vao mot trang thai duy nhat."""

    def __init__(self):
        self.lat = 0.0
        self.lon = 0.0
        self.alt_msl = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.throttle = 0.0
        self.armed = 0
        self.mode = 0
        self.got_position = False
        # PWM ma flight controller THUC SU nhan duoc — dung de kiem tra
        # RC override co den noi khong.
        self.rc_in = [0, 0, 0, 0]

    def update(self, msg):
        t = msg.get_type()

        if t == "GLOBAL_POSITION_INT":
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            self.alt_msl = msg.alt / 1000.0
            self.vx = msg.vx / 100.0
            self.vy = msg.vy / 100.0
            self.vz = msg.vz / 100.0
            self.got_position = True

        elif t == "ATTITUDE":
            self.roll = math.degrees(msg.roll)
            self.pitch = math.degrees(msg.pitch)
            self.yaw = math.degrees(msg.yaw) % 360.0

        elif t == "VFR_HUD":
            self.throttle = max(0.0, min(1.0, msg.throttle / 100.0))

        elif t in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
            self.rc_in = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]

        elif t == "HEARTBEAT":
            if msg.get_srcComponent() in (0, 1):
                self.armed = 1 if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else 0
                self.mode = msg.custom_mode & 0xFF

    def pack(self, seq):
        return struct.pack(
            STATE_FMT,
            STATE_MAGIC, seq & 0xFFFFFFFF,
            self.lat, self.lon, self.alt_msl,
            self.roll, self.pitch, self.yaw,
            self.vx, self.vy, self.vz,
            self.throttle,
            self.armed, self.mode, 0,
        )

    def line(self):
        rc = f"RCin[{self.rc_in[0]} {self.rc_in[1]} {self.rc_in[2]} {self.rc_in[3]}]"
        return (f"alt={self.alt_msl:7.2f}m r={self.roll:5.1f} p={self.pitch:5.1f} "
                f"y={self.yaw:5.1f} thr={self.throttle:.2f} {rc} "
                f"{MODE_NAMES.get(self.mode, self.mode)} "
                f"{'ARMED' if self.armed else 'disarmed'}")


class ControlState:
    """Lenh dieu khien nhan tu Unreal."""

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0
        self.arm_request = ARM_NONE
        self.mode_request = MODE_NONE
        self.last_rx = 0.0

    def alive(self):
        return (time.time() - self.last_rx) < CONTROL_TIMEOUT


def stick_to_pwm(v):
    """-1.0 .. 1.0  ->  1000 .. 2000"""
    v = max(-1.0, min(1.0, v))
    return int(round(1500 + v * 500))


def throttle_to_pwm(v):
    """0.0 .. 1.0  ->  1000 .. 2000"""
    v = max(0.0, min(1.0, v))
    return int(round(1000 + v * 1000))


# ---------------------------------------------------------------------------
# Do cong
# ---------------------------------------------------------------------------
def port_is_open(port, host="127.0.0.1", timeout=0.4):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        return s.connect_ex((host, port)) == 0
    finally:
        s.close()


def scan():
    print("Quet cong MAVLink tren 127.0.0.1:\n")
    labels = {5760: "SERIAL0 (Mission Planner thuong dung)",
              5762: "SERIAL1",
              5763: "SERIAL2 (nen dung cho bridge)"}
    found = False
    for p in (5760, 5762, 5763):
        ok = port_is_open(p)
        print(f"  TCP {p:<6} {'DANG MO ' if ok else 'dong    '}  {labels[p]}")
        found = found or ok
    if not found:
        print("\n  Khong thay cong nao. SITL chua chay?")
        print("  Mo Mission Planner -> tab SIMULATION -> Multirotor -> Stable")
    print()


def autodetect():
    for p in SITL_TCP_PORTS:
        if port_is_open(p):
            conn = f"tcp:127.0.0.1:{p}"
            print(f"[bridge] Tu dong tim thay SITL tai {conn}")
            if p == 5760:
                print("[bridge] CANH BAO: 5760 la cong Mission Planner hay dung. "
                      "Neu MP bi ngat ket noi, hay dung 5763.")
            return conn
    return None


# ---------------------------------------------------------------------------
# Che do demo
# ---------------------------------------------------------------------------
def run_demo(ue_addr, home, radius=60.0, alt=40.0, period=30.0):
    lat0, lon0, _ = home
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state = DroneState()
    state.got_position = True
    state.armed = 1
    state.throttle = 0.55

    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))

    print(f"[demo] Bay vong tron ban kinh {radius:.0f}m, cao {alt:.0f}m, chu ky {period:.0f}s")
    print(f"[demo] Gui sang Unreal tai {ue_addr}. Ctrl+C de dung.\n")

    seq = 0
    t0 = time.time()
    interval = 1.0 / SEND_HZ

    try:
        while True:
            t = time.time() - t0
            ang = 2.0 * math.pi * (t / period)

            north = radius * math.cos(ang)
            east = radius * math.sin(ang)

            state.lat = lat0 + north / m_per_deg_lat
            state.lon = lon0 + east / m_per_deg_lon
            state.alt_msl = home[2] + alt

            speed = 2.0 * math.pi * radius / period
            state.vx = -speed * math.sin(ang)
            state.vy = speed * math.cos(ang)
            state.vz = 0.0

            state.yaw = math.degrees(math.atan2(state.vy, state.vx)) % 360.0
            state.roll = math.degrees(math.atan2(speed ** 2 / radius, 9.81))
            state.pitch = -3.0

            sock.sendto(state.pack(seq), ue_addr)
            seq += 1

            if seq % int(SEND_HZ * 2) == 0:
                print(state.line())

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n[demo] Dung.")
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# Che do that, hai chieu
# ---------------------------------------------------------------------------
def run_bridge(source, ue_addr, control_port, enable_control, verbose, force_arm, sysid):
    require_pymavlink()

    print(f"[bridge] Ket noi MAVLink: {source}  (sysid={sysid})")
    master = mavutil.mavlink_connection(source, source_system=sysid, source_component=190)

    print("[bridge] Cho HEARTBEAT tu autopilot...")
    master.wait_heartbeat()
    print(f"[bridge] Da thay he thong {master.target_system} component {master.target_component}")

    # Target system 0 nghia la broadcast — lenh se khong den dung dich.
    # ArduPilot that luon la 1, nen ep ve 1 khi gap 0.
    tgt_sys = master.target_system if master.target_system != 0 else 1
    tgt_comp = 1

    ue_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    ctrl_sock = None
    ctrl = ControlState()
    if enable_control:
        ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ctrl_sock.setblocking(False)
        ctrl_sock.bind(("0.0.0.0", control_port))
        print(f"[bridge] Nhan lenh dieu khien tu Unreal tren UDP {control_port}")

    state = DroneState()

    seq = 0
    now = time.time()
    next_send = now
    next_rc = now
    next_keepalive = now
    send_interval = 1.0 / SEND_HZ
    rc_interval = 1.0 / RC_HZ

    last_arm_request = ARM_NONE
    last_mode_request = MODE_NONE
    override_active = False
    warned_no_control = False

    print(f"[bridge] Gui telemetry sang Unreal tai {ue_addr} @ {SEND_HZ:.0f} Hz")
    print("[bridge] Ctrl+C de dung.\n")

    try:
        while True:
            # ---------- doc MAVLink ----------
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() != "BAD_DATA":
                    state.update(msg)
                    if verbose and msg.get_type() == "STATUSTEXT":
                        try:
                            print(f"  [SITL] {msg.text}")
                        except Exception:
                            pass

            # ---------- doc lenh tu Unreal ----------
            if ctrl_sock is not None:
                while True:
                    try:
                        data, _ = ctrl_sock.recvfrom(256)
                    except BlockingIOError:
                        break
                    except OSError:
                        break
                    if len(data) < CONTROL_SIZE:
                        continue
                    fields = struct.unpack(CONTROL_FMT, data[:CONTROL_SIZE])
                    if fields[0] != CONTROL_MAGIC:
                        continue
                    (_, _, ctrl.roll, ctrl.pitch, ctrl.yaw, ctrl.throttle,
                     ctrl.arm_request, ctrl.mode_request, _, _) = fields
                    ctrl.last_rx = time.time()

            now = time.time()

            # ---------- keepalive ----------
            if now >= next_keepalive:
                next_keepalive = now + 2.0
                try:
                    master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    master.mav.request_data_stream_send(
                        tgt_sys, tgt_comp,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL, int(SEND_HZ), 1)
                except Exception as e:
                    print(f"[bridge] Loi gui keepalive: {e}")

            # ---------- xu ly lenh ARM / MODE ----------
            if enable_control and ctrl.alive():
                # Chi ban lenh khi trang thai CHUYEN tu none sang co yeu cau,
                # nen giu phim khong lam lenh bi ban lien tuc.
                if ctrl.arm_request != last_arm_request:
                    if ctrl.arm_request == ARM_ARM:
                        if ctrl.throttle > 0.05:
                            print(f"[bridge] TU CHOI ARM: throttle dang o {ctrl.throttle:.2f}, "
                                  "phai ve 0 truoc khi arm (bam S giu cho ve 0)")
                        else:
                            p2 = 21196 if force_arm else 0
                            master.mav.command_long_send(
                                tgt_sys, tgt_comp,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, p2, 0, 0, 0, 0, 0)
                            print("[bridge] --> Gui lenh ARM")
                    elif ctrl.arm_request == ARM_DISARM:
                        p2 = 21196 if force_arm else 0
                        master.mav.command_long_send(
                            tgt_sys, tgt_comp,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 0, p2, 0, 0, 0, 0, 0)
                        print("[bridge] --> Gui lenh DISARM")
                    last_arm_request = ctrl.arm_request

                if ctrl.mode_request != last_mode_request:
                    if ctrl.mode_request != MODE_NONE:
                        master.mav.set_mode_send(
                            tgt_sys,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            ctrl.mode_request)
                        name = MODE_NAMES.get(ctrl.mode_request, ctrl.mode_request)
                        print(f"[bridge] --> Doi mode sang {name}")
                    last_mode_request = ctrl.mode_request

            # ---------- RC override ----------
            if enable_control and now >= next_rc:
                next_rc += rc_interval
                if next_rc < now:
                    next_rc = now + rc_interval

                if ctrl.alive():
                    # RCMAP mac dinh cua ArduCopter:
                    #   CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw
                    # CH2 thap = chui mui ve truoc, nen phai dao dau pitch.
                    ch1 = stick_to_pwm(ctrl.roll)
                    ch2 = stick_to_pwm(-ctrl.pitch)
                    ch3 = throttle_to_pwm(ctrl.throttle)
                    ch4 = stick_to_pwm(ctrl.yaw)
                    master.mav.rc_channels_override_send(
                        tgt_sys, tgt_comp, ch1, ch2, ch3, ch4, 0, 0, 0, 0)
                    override_active = True
                    warned_no_control = False

                elif override_active:
                    # Mat lien lac voi Unreal — nha override de SITL khong bi ket
                    master.mav.rc_channels_override_send(
                        tgt_sys, tgt_comp, 0, 0, 0, 0, 0, 0, 0, 0)
                    override_active = False
                    print("[bridge] Mat lenh tu Unreal, da nha RC override")

                elif not warned_no_control and verbose:
                    warned_no_control = True
                    print("[bridge] Dang cho lenh tu Unreal (bam Play va bat bEnableControl)")

            # ---------- gui telemetry sang Unreal ----------
            if now >= next_send:
                next_send += send_interval
                if next_send < now:
                    next_send = now + send_interval

                if state.got_position:
                    ue_sock.sendto(state.pack(seq), ue_addr)
                    seq += 1
                    if verbose and seq % int(SEND_HZ) == 0:
                        print(state.line())

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n[bridge] Dung.")
    finally:
        # Luon nha override truoc khi thoat
        if enable_control and override_active:
            try:
                master.mav.rc_channels_override_send(
                    tgt_sys, tgt_comp, 0, 0, 0, 0, 0, 0, 0, 0)
                print("[bridge] Da nha RC override.")
            except Exception:
                pass
        ue_sock.close()
        if ctrl_sock:
            ctrl_sock.close()


def main():
    p = argparse.ArgumentParser(description="Cau noi MAVLink <-> Unreal (Windows, mot may)")
    p.add_argument("--source", default="auto",
                   help="Nguon MAVLink. 'auto' de tu do. VD: tcp:127.0.0.1:5763")
    p.add_argument("--unreal", default="127.0.0.1:14560",
                   help="Dia chi Unreal, khop ListenPort cua DronePawn")
    p.add_argument("--control-port", type=int, default=14561,
                   help="Cong UDP nhan lenh tu Unreal")
    p.add_argument("--no-control", action="store_true",
                   help="Chi xem, khong nhan lenh dieu khien tu Unreal")
    p.add_argument("--force-arm", action="store_true",
                   help="Bo qua kiem tra truoc khi arm (chi dung trong sim)")
    p.add_argument("--sysid", type=int, default=255,
                   help="System ID cua bridge. PHAI khop tham so SYSID_MYGCS "
                        "cua ArduPilot, neu khong RC override se bi bo qua. Mac dinh 255.")
    p.add_argument("--demo", action="store_true",
                   help="Bay vong tron gia lap, khong can Mission Planner")
    p.add_argument("--home", default=None, help="Home cho che do demo: lat,lon,alt")
    p.add_argument("--scan", action="store_true", help="Chi quet cong roi thoat")
    p.add_argument("--quiet", action="store_true", help="Khong in telemetry")
    args = p.parse_args()

    if args.scan:
        scan()
        return 0

    host, port = args.unreal.split(":")
    ue_addr = (host, int(port))

    if args.demo:
        home = DEFAULT_HOME
        if args.home:
            parts = [float(x) for x in args.home.split(",")]
            home = (parts[0], parts[1], parts[2] if len(parts) > 2 else 0.0)
        run_demo(ue_addr, home)
        return 0

    source = args.source
    if source == "auto":
        source = autodetect()
        if source is None:
            print("[bridge] Khong tim thay SITL dang chay.\n")
            scan()
            print("Neu ban dung MAVLink forwarding cua Mission Planner, chay:")
            print(f"    python mavlink_to_unreal.py --source {FALLBACK_UDP}\n")
            print("Muon kiem tra phia Unreal truoc, chay:")
            print("    python mavlink_to_unreal.py --demo")
            return 1

    run_bridge(source, ue_addr, args.control_port,
               not args.no_control, not args.quiet, args.force_arm, args.sysid)
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

---

### `Bridge/start_bridge.bat`

Vỏ bọc Windows gọi mavlink_to_unreal.py: kiểm tra Python, tự cài pymavlink, quét cổng, cho chọn chế độ.  
*60 dòng*

```bat
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
```

---

### `Bridge/requirements.txt`

Dependency Python.  
*4 dòng*

```text
# DroneTwin Bridge — Python dependencies
# Cai bang:  pip install -r requirements.txt

pymavlink>=2.4.37
```

---

### `.gitignore`

Loại file sinh ra khi build; giữ lại Content/ vì .uasset không tái tạo được.  
*27 dòng*

```text
# ===== Unreal Engine =====
Binaries/
DerivedDataCache/
Intermediate/
Saved/
.vscode/
.vs/
*.VC.db
*.opensdf
*.opendb
*.sdf
*.sln
*.suo
*.xcodeproj
*.xcworkspace

# ===== Python =====
__pycache__/
*.py[cod]
.venv/
venv/

# ===== KHONG ignore nhung thu nay =====
# Content/      <- .uasset cua Blueprint, BAT BUOC phai commit
# Config/       <- cau hinh project
# Source/       <- code C++
# Bridge/       <- code Python
```

---

## 8. Chưa làm

1. **HUD bằng UMG** — overlay hiện tại vẽ bằng `AddOnScreenDebugMessage`, đủ dùng
   để debug nhưng xấu. Các biến `bIsArmed`, `AltitudeMSL`, `GroundSpeed`,
   `bLinkAlive`, `CurrentMode`, `StickRoll/Pitch/Yaw/Throttle` đã expose sang
   Blueprint, chỉ cần vẽ widget, không phải đụng C++.
2. **Camera FPV / gimbal** — SceneCaptureComponent2D → RenderTarget → widget góc
   màn hình. Field `Reserved` trong gói CONTROL đã để dành cho điều khiển gimbal.
3. **Vẽ đường bay** — spline hoặc trail theo lịch sử vị trí.
4. **Ép home về Hà Nội** (21.0278, 105.8342) cho bản demo cuối. Cần chạy
   `ArduCopter.exe` trực tiếp với cờ `--home` thay vì dùng nút SIMULATION của MP.
5. **Cánh quạt quay** — mesh đã combine nên không quay được. Cách đúng là dùng
   mesh đĩa mờ bán trong suốt, không phải quay mesh cánh thật (ở 60 fps mắt chỉ
   thấy nhấp nháy do aliasing).
6. **Unreal tự tính physics** — chỉ cần nếu muốn mô phỏng va chạm/gió trong
   Unreal. Lúc đó bridge gửi `HIL_GPS` / `HIL_STATE_QUATERNION` vào SITL ở chế độ
   HIL. Đánh đổi: mất EKF và failsafe thật, phải tự viết bộ điều khiển bay.
