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
