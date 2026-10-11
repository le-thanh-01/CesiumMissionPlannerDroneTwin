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
    python mavlink_to_unreal.py --demo          # bay kich ban demo co dinh, khong can MP
    python mavlink_to_unreal.py --scan          # xem cong SITL nao dang mo
"""

import argparse
import bisect
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
        print("Thiếu pymavlink. Chạy:  pip install pymavlink")
        sys.exit(1)


# ---------------------------------------------------------------------------
# Goi STATE: bridge -> Unreal. Khop voi FDroneStatePacket trong DronePawn.h
# ---------------------------------------------------------------------------
STATE_MAGIC = 0x314A4A44                # 'DJI1'
STATE_FMT = "<IIdddfffffffBBH4H"
STATE_SIZE = struct.calcsize(STATE_FMT)
assert STATE_SIZE == 72, f"State packet phai 72 byte, dang la {STATE_SIZE}"

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
# Kich ban demo: (lat, lon). Cac diem cua --demo; --home se dich ca lo trinh theo diem xuat phat moi.
DEMO_START = (21.00501642683876, 105.84396099404114)    # cat canh tai day
DEMO_FIRST = (21.005066506302764, 105.84563469242508)   # diem 1: bay thang den day o do cao DEMO_CLIMB_ALT
DEMO_ARC = [                                            # tu diem 1 bay vong cung qua lan luot cac diem nay
    (21.006333511150274, 105.8453664715302),
    (21.007405198551503, 105.8438268835937),
    (21.007395182817017, 105.84168648085272),
    (21.005402038278028, 105.84182595571804),
    (21.004270240834327, 105.84165965876323),
    (21.003899650425293, 105.84339773016193),
    (21.00415505742738, 105.8449855978595),
    (21.005036458626385, 105.84564542126085),
]
DEMO_CLIMB_ALT = 70.0       # m, do cao sau khi cat canh
DEMO_CRUISE_ALT = 300.0     # m, do cao o diem cuoi cua duong vong cung
DEMO_SPEED = 10.0           # m/s, toc do bay ngang
DEMO_CLIMB_RATE = 2.5       # m/s, toc do len luc cat canh
DEMO_DESCENT_RATE = 5.0     # m/s, toc do ha canh
DEMO_SPIN_SEC = 5.0         # s, quay tai cho dung 1 vong
DEMO_HOLD_SEC = 5.0         # s, dung yen truoc khi ha canh
DEMO_PITCH_MAX = 15.0       # do, goc chui mui toi da khi bay toi o toc do DEMO_SPEED (am = mui chui xuong, theo MAVLink)
DEMO_YAW_RATE = 90.0        # deg/s, toc do quay yaw toi da khi doi huong
DEMO_TAIL_SEC = 1.5         # s, van gui trang thai da ha canh them mot luc roi thoat
DEFAULT_HOME = (DEMO_START[0], DEMO_START[1], 2.0)     # lat, lon, do cao mat dat (MSL)


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
        self.motor_pwm = [0, 0, 0, 0]

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

        elif t == "SERVO_OUTPUT_RAW":
            self.motor_pwm = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]

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
            *self.motor_pwm,
        )

    def line(self):
        rc = f"RCin[{self.rc_in[0]} {self.rc_in[1]} {self.rc_in[2]} {self.rc_in[3]}]"
        mot = f"mot[{self.motor_pwm[0]} {self.motor_pwm[1]} {self.motor_pwm[2]} {self.motor_pwm[3]}]"
        return (f"alt={self.alt_msl:7.2f}m r={self.roll:5.1f} p={self.pitch:5.1f} "
                f"y={self.yaw:5.1f} thr={self.throttle:.2f} {rc} {mot} "
                f"{MODE_NAMES.get(self.mode, self.mode)} "
                f"{'ARMED' if self.armed else 'disarmed'}"
                )


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
    print("Quét cổng MAVLink trên 127.0.0.1:\n")
    labels = {5760: "SERIAL0 (Mission Planner thường dùng)",
              5762: "SERIAL1",
              5763: "SERIAL2 (nên dùng cho bridge)"}
    found = False
    for p in (5760, 5762, 5763):
        ok = port_is_open(p)
        print(f"  TCP {p:<6} {'ĐANG MỞ ' if ok else 'đóng    '}  {labels[p]}")
        found = found or ok
    if not found:
        print("\n  Không thấy cổng nào. SITL chưa chạy?")
        print("  Mở Mission Planner -> tab SIMULATION -> Multirotor -> Stable")
    print()


def autodetect():
    for p in SITL_TCP_PORTS:
        if port_is_open(p):
            conn = f"tcp:127.0.0.1:{p}"
            print(f"[bridge] Tự động tìm thấy SITL tại {conn}")
            if p == 5760:
                print("[bridge] CẢNH BÁO: 5760 là cổng Mission Planner hay dùng. "
                      "Nếu MP bị ngắt kết nối, hãy dùng 5763.")
            return conn
    return None


# ---------------------------------------------------------------------------
# Che do demo: bay theo kich ban co dinh
# ---------------------------------------------------------------------------
def _to_local(lat, lon, lat0, lon0):
    """(lat, lon) -> (north, east) tinh bang met, goc la (lat0, lon0)."""
    m_lat = 111320.0
    m_lon = 111320.0 * math.cos(math.radians(lat0))
    return ((lat - lat0) * m_lat, (lon - lon0) * m_lon)


def _catmull_rom(p0, p1, p2, p3, n, alpha=0.5):
    """Duong cong Catmull-Rom (centripetal) di qua p1 -> p2, lay n mau (khong gom p2).
    p0 va p3 chi dinh huong cong o hai dau."""
    def tj(ti, pi, pj):
        return ti + max(math.hypot(pj[0] - pi[0], pj[1] - pi[1]), 1e-6) ** alpha

    def mix(a, b, wa, wb):
        return (a[0] * wa + b[0] * wb, a[1] * wa + b[1] * wb)

    t0 = 0.0
    t1 = tj(t0, p0, p1)
    t2 = tj(t1, p1, p2)
    t3 = tj(t2, p2, p3)

    out = []
    for k in range(n):
        t = t1 + (t2 - t1) * k / n
        a1 = mix(p0, p1, (t1 - t) / (t1 - t0), (t - t0) / (t1 - t0))
        a2 = mix(p1, p2, (t2 - t) / (t2 - t1), (t - t1) / (t2 - t1))
        a3 = mix(p2, p3, (t3 - t) / (t3 - t2), (t - t2) / (t3 - t2))
        b1 = mix(a1, a2, (t2 - t) / (t2 - t0), (t - t0) / (t2 - t0))
        b2 = mix(a2, a3, (t3 - t) / (t3 - t1), (t - t1) / (t3 - t1))
        out.append(mix(b1, b2, (t2 - t) / (t2 - t1), (t - t1) / (t2 - t1)))
    return out


class _Polyline:
    """Duong gap khuc (north, east) co do dai cung, de lay vi tri theo quang duong da di."""

    def __init__(self, pts):
        self.pts = pts
        self.cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.cum[-1]

    def at(self, s):
        s = max(0.0, min(self.length, s))
        i = min(bisect.bisect_right(self.cum, s) - 1, len(self.pts) - 2)
        seg = self.cum[i + 1] - self.cum[i]
        u = (s - self.cum[i]) / seg if seg > 0 else 0.0
        a, b = self.pts[i], self.pts[i + 1]
        return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u)

    def heading(self, s, d=2.0):
        """Huong bay (do, 0 = Bac, theo chieu kim dong ho) tai quang duong s."""
        a = self.at(s - d)
        b = self.at(s + d)
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360.0


def build_demo_mission():
    """Tra ve (danh sach giai doan, tong thoi gian).
    Moi giai doan = (ten, thoi_luong, ham(tau) -> (north, east, do_cao_so_voi_mat_dat, yaw))
    voi north/east tinh bang met so voi diem xuat phat."""
    lat_s, lon_s = DEMO_START

    def loc(p):
        return _to_local(p[0], p[1], lat_s, lon_s)

    start = (0.0, 0.0)
    first = loc(DEMO_FIRST)
    arc_pts = [loc(p) for p in DEMO_ARC]

    # Duong vong cung: Catmull-Rom di qua lan luot cac diem, dau/cuoi noi voi diem xuat phat
    ctrl = [start, first] + arc_pts + [start]
    dense = []
    for i in range(1, len(ctrl) - 2):
        dense += _catmull_rom(ctrl[i - 1], ctrl[i], ctrl[i + 1], ctrl[i + 2], 60)
    dense.append(ctrl[-2])
    arc = _Polyline(dense)

    out_leg = _Polyline([start, first])
    back_leg = _Polyline([arc_pts[-1], start])
    yaw_out = out_leg.heading(0.0)
    yaw_back = back_leg.heading(0.0)

    v = DEMO_SPEED
    hi = DEMO_CRUISE_ALT
    lo = DEMO_CLIMB_ALT

    def climb(tau):
        return (0.0, 0.0, DEMO_CLIMB_RATE * tau, yaw_out)

    def go_first(tau):
        n, e = out_leg.at(v * tau)
        return (n, e, lo, yaw_out)

    def go_arc(tau):
        s = v * tau
        n, e = arc.at(s)
        agl = lo + (hi - lo) * min(1.0, s / arc.length)
        return (n, e, agl, arc.heading(s))

    def go_back(tau):
        n, e = back_leg.at(v * tau)
        return (n, e, hi, yaw_back)

    def spin(tau):
        return (0.0, 0.0, hi, yaw_back + 360.0 * tau / DEMO_SPIN_SEC)

    def hold(tau):
        return (0.0, 0.0, hi, yaw_back)

    def land(tau):
        return (0.0, 0.0, max(0.0, hi - DEMO_DESCENT_RATE * tau), yaw_back)

    phases = [
        ("Cất cánh", lo / DEMO_CLIMB_RATE, climb),
        ("Bay thẳng đến điểm 1", out_leg.length / v, go_first),
        ("Bay vòng cung", arc.length / v, go_arc),
        ("Bay thẳng về điểm xuất phát", back_leg.length / v, go_back),
        ("Quay tại chỗ", DEMO_SPIN_SEC, spin),
        ("Đứng yên", DEMO_HOLD_SEC, hold),
        ("Hạ cánh", hi / DEMO_DESCENT_RATE, land),
    ]
    return phases, sum(p[1] for p in phases)


def mission_at(phases, t):
    for name, dur, fn in phases:
        if t < dur:
            return name, fn(t)
        t -= dur
    name, dur, fn = phases[-1]
    return name, fn(dur)


def run_demo(ue_addr, home):
    lat0, lon0, ground_alt = home
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))

    phases, total = build_demo_mission()

    state = DroneState()
    state.got_position = True
    state.armed = 1
    state.yaw = mission_at(phases, 0.0)[1][3]
    state.motor_pwm = [1500, 1500, 1500, 1500]
    print(f"[demo] Kịch bản: cất cánh lên {DEMO_CLIMB_ALT:.0f}m -> bay thẳng đến điểm 1 -> "
          f"bay vòng cung qua {len(DEMO_ARC)} điểm (độ cao tăng dần lên {DEMO_CRUISE_ALT:.0f}m) -> "
          f"bay thẳng về điểm xuất phát -> quay tại chỗ {DEMO_SPIN_SEC:.0f}s -> "
          f"đứng yên {DEMO_HOLD_SEC:.0f}s -> hạ cánh")
    print(f"[demo] Tốc độ {DEMO_SPEED:.0f} m/s, tổng thời gian dự kiến {total:.0f}s ({total / 60:.1f} phút)")
    print(f"[demo] Gửi sang Unreal tại {ue_addr}. Ctrl+C để dừng.\n")

    seq = 0
    t0 = time.time()
    interval = 1.0 / SEND_HZ
    prev_t = 0.0
    prev_phase = None
    last_print = -999.0
    h = 0.05    # buoc lay vi phan de tinh van toc

    try:
        while True:
            t = time.time() - t0
            if t > total + DEMO_TAIL_SEC:
                break
            tc = min(t, total)
            phase, (n, e, agl, yaw_tgt) = mission_at(phases, tc)

            if phase != prev_phase:
                print(f"[demo] t={t:6.1f}s  Giai đoạn: {phase}")
                prev_phase = phase

            # van toc = dao ham vi tri (vz theo MAVLink: duong la di xuong)
            ta, tb = max(0.0, tc - h), min(total, tc + h)
            _, pa = mission_at(phases, ta)
            _, pb = mission_at(phases, tb)
            span = max(tb - ta, 1e-6)
            vn = (pb[0] - pa[0]) / span
            ve = (pb[1] - pa[1]) / span
            vup = (pb[2] - pa[2]) / span

            # yaw quay muot, gioi han toc do quay
            dt = max(1e-3, t - prev_t)
            prev_t = t
            diff = ((yaw_tgt - state.yaw + 180.0) % 360.0) - 180.0
            max_step = DEMO_YAW_RATE * dt
            step = max(-max_step, min(max_step, diff))
            state.yaw = (state.yaw + step) % 360.0
            yaw_rate = step / dt

            # pitch: chui mui ve phia truoc, ty le theo toc do bay ngang (pitch am = mui chui xuong)
            # roll: nghieng theo luc ly tam khi vao cua
            v_h = math.hypot(vn, ve)
            pitch_t = -DEMO_PITCH_MAX * min(1.0, v_h / DEMO_SPEED)
            if v_h > 0.5:
                roll_t = math.degrees(math.atan2(v_h * math.radians(yaw_rate), 9.81))
                roll_t = max(-30.0, min(30.0, roll_t))
            else:
                roll_t = 0.0
            state.roll += (roll_t - state.roll) * 0.15
            state.pitch += (pitch_t - state.pitch) * 0.15

            landed = t >= total
            state.armed = 0 if landed else 1
            state.throttle = 0.0 if landed else max(0.2, min(0.8, 0.5 + 0.05 * vup))

            state.lat = lat0 + n / m_per_deg_lat
            state.lon = lon0 + e / m_per_deg_lon
            state.alt_msl = ground_alt + agl
            state.vx = vn
            state.vy = ve
            state.vz = -vup

            sock.sendto(state.pack(seq), ue_addr)
            seq += 1

            if t - last_print >= 2.0:
                last_print = t
                print(f"[demo] {state.line()}")

            time.sleep(interval)

        print(f"[demo] Hoàn thành kịch bản bay, thiết bị đã hạ cánh (tổng {total:.0f}s).")

    except KeyboardInterrupt:
        print("\n[demo] Dừng.")
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# Che do that, hai chieu
# ---------------------------------------------------------------------------
def run_bridge(source, ue_addr, control_port, enable_control, verbose, force_arm, sysid):
    require_pymavlink()

    print(f"[bridge] Kết nối MAVLink: {source}  (sysid={sysid})")
    master = mavutil.mavlink_connection(source, source_system=sysid, source_component=190)

    print("[bridge] Chờ HEARTBEAT từ autopilot...")
    master.wait_heartbeat()
    print(f"[bridge] Đã thấy hệ thống {master.target_system} component {master.target_component}")

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
        print(f"[bridge] Nhận lệnh điều khiển từ Unreal trên UDP {control_port}")

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

    print(f"[bridge] Gửi telemetry sang Unreal tại {ue_addr} @ {SEND_HZ:.0f} Hz")
    print("[bridge] Ctrl+C để dừng.\n")

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
                    print(f"[bridge] Lỗi gửi keepalive: {e}")

            # ---------- xu ly lenh ARM / MODE ----------
            if enable_control and ctrl.alive():
                # Chi ban lenh khi trang thai CHUYEN tu none sang co yeu cau,
                # nen giu phim khong lam lenh bi ban lien tuc.
                if ctrl.arm_request != last_arm_request:
                    if ctrl.arm_request == ARM_ARM:
                        if ctrl.throttle > 0.05:
                            print(f"[bridge] TỪ CHỐI ARM: throttle đang ở {ctrl.throttle:.2f}, "
                                  "phải về 0 trước khi arm (bấm S giữ cho về 0)")
                        else:
                            p2 = 21196 if force_arm else 0
                            master.mav.command_long_send(
                                tgt_sys, tgt_comp,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, p2, 0, 0, 0, 0, 0)
                            print("[bridge] --> Gửi lệnh ARM")
                    elif ctrl.arm_request == ARM_DISARM:
                        p2 = 21196 if force_arm else 0
                        master.mav.command_long_send(
                            tgt_sys, tgt_comp,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 0, p2, 0, 0, 0, 0, 0)
                        print("[bridge] --> Gửi lệnh DISARM")
                    last_arm_request = ctrl.arm_request

                if ctrl.mode_request != last_mode_request:
                    if ctrl.mode_request != MODE_NONE:
                        master.mav.set_mode_send(
                            tgt_sys,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            ctrl.mode_request)
                        name = MODE_NAMES.get(ctrl.mode_request, ctrl.mode_request)
                        print(f"[bridge] --> Đổi mode sang {name}")
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
                    print("[bridge] Mất lệnh từ Unreal, đã nhả RC override")

                elif not warned_no_control and verbose:
                    warned_no_control = True
                    print("[bridge] Đang chờ lệnh từ Unreal (bấm Play và bật bEnableControl)")

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
        print("\n[bridge] Dừng.")
    finally:
        # Luon nha override truoc khi thoat
        if enable_control and override_active:
            try:
                master.mav.rc_channels_override_send(
                    tgt_sys, tgt_comp, 0, 0, 0, 0, 0, 0, 0, 0)
                print("[bridge] Đã nhả RC override.")
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
                   help="Bay kich ban demo co dinh (cat canh, vong cung, ha canh), khong can Mission Planner")
    p.add_argument("--home", default=None, help="Diem xuat phat cho che do demo: lat,lon,alt (ca lo trinh dich theo diem nay)")
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
            print("[bridge] Không tìm thấy SITL đang chạy.\n")
            scan()
            print("Nếu bạn dùng MAVLink forwarding của Mission Planner, chạy:")
            print(f"    python mavlink_to_unreal.py --source {FALLBACK_UDP}\n")
            print("Muốn kiểm tra phía Unreal trước, chạy:")
            print("    python mavlink_to_unreal.py --demo")
            return 1

    run_bridge(source, ue_addr, args.control_port,
               not args.no_control, not args.quiet, args.force_arm, args.sysid)
    return 0


if __name__ == "__main__":
    sys.exit(main())