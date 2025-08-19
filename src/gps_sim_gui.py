#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UBX + NMEA (v4.11) GPS Simulator GUI (Tkinter)

Key features:
- Multi-thread: UI thread + worker simulator thread
- Serial write if port opens; otherwise runs offline (still generates data & updates UI)
- Port initially blank; Baud combobox enabled only after choosing a port
- Start enabled only when Port & Baudrate are set and at least one message is selected
- Apply-to-running resets sim and applies changes (including update rate); button lights only when settings changed
- Profiles with linear speed ramp; NO_FIX and GPS_DISABLE special modes
- Active profile shown inside Profile settings; 60fps smoothed progress bar (gray in NO_FIX/GPS_DISABLE)
- Status area with fixed columns; Speed standout line; FIX text as:
    * "3D FIX" (green) for fix=3, "2D FIX" (green) for fix=2, "NO FIX" (red) otherwise
    * "GPS DISABLED" (gray) in GPS_DISABLE profile
    * In NO_FIX or GPS_DISABLE, numeric metrics show as "______"
    * In GPS_DISABLE the entire status area turns gray
- Message settings:
    * Update rate (Hz) (applies on Apply-to-running)
    * Master checkboxes (Enable UBX / Enable NMEA) aligned with child checkboxes (3-column layout)
    * Actual send rate per message (UBX + NMEA)
- TX Monitor lists latest UBX (hex, full) and NMEA (text) per enabled message; right-click to copy
- Auto-resize window height on actions (not on timer) to fit TX rows; min size stable to avoid jumpiness
"""

import threading
import queue
import time
import math
import random
import struct
from datetime import datetime, timezone
from typing import Optional, Dict, Tuple, List

import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None  # UI can open; starting requires pyserial for real serial I/O


# =========================
# Math / UBX helpers
# =========================

def ubx_checksum(body: bytes):
    ck_a, ck_b = 0, 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def to_ubx(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    length = struct.pack('<H', len(payload))
    body = bytes([msg_class, msg_id]) + length + payload
    ck_a, ck_b = ubx_checksum(body)
    return b'\xB5\x62' + body + bytes([ck_a, ck_b])


def gps_week_and_tow_ms(now_utc: datetime):
    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    delta = now_utc - gps_epoch
    total_ms = int(delta.total_seconds() * 1000)
    week_ms = 7 * 24 * 3600 * 1000
    week = total_ms // week_ms
    itow = total_ms % week_ms
    return int(week), int(itow)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, h_m: float):
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1 - e2) + h_m) * sin_lat
    return x, y, z


def ned_to_ecef(lat_deg: float, lon_deg: float, vn: float, ve: float, vd: float):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    r11 = -sin_lat * cos_lon
    r12 = -sin_lat * sin_lon
    r13 = cos_lat
    r21 = -sin_lon
    r22 =  cos_lon
    r23 =  0.0
    r31 = -cos_lat * cos_lon
    r32 = -cos_lat * sin_lon
    r33 = -sin_lat
    vx = r11 * vn + r12 * ve + r13 * vd
    vy = r21 * vn + r22 * ve + r23 * vd
    vz = r31 * vn + r32 * ve + r33 * vd
    return vx, vy, vz


# =========================
# NMEA helpers (v4.11)
# =========================

def nmea_checksum(sentence_wo_dollar: str) -> str:
    cs = 0
    for ch in sentence_wo_dollar:
        cs ^= ord(ch)
    return f"{cs:02X}"


def deg_to_nmea_lat(lat: float) -> Tuple[str, str]:
    hemi = 'N' if lat >= 0 else 'S'
    lat = abs(lat)
    deg = int(lat)
    minutes = (lat - deg) * 60.0
    return f"{deg:02d}{minutes:07.4f}", hemi


def deg_to_nmea_lon(lon: float) -> Tuple[str, str]:
    hemi = 'E' if lon >= 0 else 'W'
    lon = abs(lon)
    deg = int(lon)
    minutes = (lon - deg) * 60.0
    return f"{deg:03d}{minutes:07.4f}", hemi


def knots_from_kmh(kmh: float) -> float:
    return kmh * 0.539956803


# =========================
# Simulator worker
# =========================

class UbxNmeaSimulator:
    """
    Control queue (from UI):
      - {"cmd": "reset", "cfg": <new_config_dict>}

    Callbacks (to UI):
      - rate_cb(dict): actual rates per key
      - profile_cb(dict): {"name": str, "progress": float, "is_lost": bool}
      - stats_cb(dict): {"utc","local","speed","heading","sats","fix","lat","lon"}
      - tx_cb(dict_nmea, dict_ubx): latest payloads keyed by message name
        * UBX keys: "pvt","velned","sol" (values: hex string)
        * NMEA keys: "gga","gll","gsa","gsv","rmc","vtg" (values: sentence string)
    """
    def __init__(
        self,
        config: dict,
        ctrl_q: queue.Queue,
        rate_cb=None,
        profile_cb=None,
        stats_cb=None,
        tx_cb=None,
        stop_event: Optional[threading.Event] = None,
    ):
        self.cfg = config
        self.ctrl_q = ctrl_q
        self.rate_cb = rate_cb or (lambda r: None)
        self.profile_cb = profile_cb or (lambda p: None)
        self.stats_cb = stats_cb or (lambda s: None)
        self.tx_cb = tx_cb or (lambda n, u: None)
        self.stop_event = stop_event or threading.Event()

        # Open serial if available; run offline if not
        self.ser = None
        if serial is not None:
            self._open_serial(self.cfg["port"], self.cfg["baudrate"])

        self._apply_config(self.cfg, reset_all=True)

    def _open_serial(self, port: str, baud: int):
        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0)
            try:
                self.ser.set_buffer_size(rx_size=8192, tx_size=8192)
            except Exception:
                pass
            try:
                self.ser.reset_output_buffer()
            except Exception:
                pass
        except Exception:
            # Offline mode (no serial). Keep running without raising.
            self.ser = None

    def _apply_config(self, cfg: dict, reset_all: bool):
        self.cfg = cfg
        self.update_rate_hz = max(0.01, float(cfg["update_rate_hz"]))
        self.send_interval = 1.0 / self.update_rate_hz

        self.ubx_en = dict(cfg["ubx_enable"])
        self.nmea_en = dict(cfg["nmea_enable"])

        self.PROFILES = dict(cfg["profiles"])
        self.order = [
            "STATIONARY",
            "WALKING",
            "CITY_DRIVING",
            "HIGHWAY_DRIVING",
            "HIGH_SPEED",
            "NO_FIX",
            "GPS_DISABLE",
        ]
        self.cycle = [p for p in self.order if self.PROFILES.get(p, {}).get("enabled", False)]
        if not self.cycle:
            self.PROFILES["STATIONARY"] = {"enabled": True, "min": 0, "max": 0, "duration": 5.0}
            self.cycle = ["STATIONARY"]
        self.current_profile = self.cycle[0]
        self.profile_start = time.perf_counter()

        if reset_all:
            self.lat, self.lon, self.h_m = 51.5074, -0.1278, 15.0
            self.speed_kmh, self.heading_deg, self.itow_ms = 0.0, 0.0, 0
            self.fix_type, self.sats = 3, 12
            self.hacc_mm, self.sacc_mms = 2000, 300

        self._sent_counts = {
            "pvt": 0, "velned": 0, "sol": 0,
            "gga": 0, "gll": 0, "gsa": 0, "gsv": 0, "rmc": 0, "vtg": 0,
        }
        self._actual_rates = {k: 0.0 for k in self._sent_counts.keys()}
        self._last_rate_time = time.perf_counter()
        self.rate_cb(dict(self._actual_rates))

    # ----- UBX builders (system time) -----
    def build_nav_pvt(self) -> bytes:
        now = datetime.now(timezone.utc)
        payload = struct.pack('<L', int(self.itow_ms))
        payload += struct.pack('<HBBBBBB', now.year, now.month, now.day,
                               now.hour, now.minute, now.second, 0x07)
        payload += struct.pack('<L', 50000)  # tAcc ns
        payload += struct.pack('<l', 0)      # nano
        flags = 0x01 if self.fix_type >= 3 else 0x00
        payload += struct.pack('<BBB', self.fix_type, flags, 0x00)
        payload += struct.pack('<B', self.sats)
        payload += struct.pack('<l', int(self.lon * 1e7))
        payload += struct.pack('<l', int(self.lat * 1e7))
        nowt = time.perf_counter()
        payload += struct.pack('<l', int((self.h_m * 1000.0) + math.sin(nowt) * 5000))
        payload += struct.pack('<l', int((self.h_m * 1000.0) - 2000 + math.sin(nowt) * 5000))
        payload += struct.pack('<L', self.hacc_mm)
        payload += struct.pack('<L', 5000)
        speed_mms = self.speed_kmh * 1000.0 / 3.6
        heading_rad = math.radians(self.heading_deg)
        vel_n = int(math.cos(heading_rad) * speed_mms)
        vel_e = int(math.sin(heading_rad) * speed_mms)
        vel_d = random.randint(-100, 100)
        payload += struct.pack('<lll', vel_n, vel_e, vel_d)
        payload += struct.pack('<l', int(abs(speed_mms)))               # gSpeed
        payload += struct.pack('<l', int(self.heading_deg * 1e5))       # headMot
        payload += struct.pack('<L', self.sacc_mms)                      # sAcc
        payload += struct.pack('<L', 1_000_000)                          # headAcc
        payload += struct.pack('<H', 120)                                # pDOP
        payload += b'\x00' * 6
        payload += struct.pack('<l', int(self.heading_deg * 1e5))        # headVeh
        payload += struct.pack('<hH', 500, 1000)                         # magDec, magAcc
        return to_ubx(0x01, 0x07, payload)

    def build_nav_velned(self) -> bytes:
        speed_mms = self.speed_kmh * 1000.0 / 3.6
        speed_cms = int(abs(speed_mms) / 10.0)
        heading = int(self.heading_deg * 1e5)
        sacc_cms = max(1, int(self.sacc_mms / 10.0))
        cacc = 1_000_000
        heading_rad = math.radians(self.heading_deg)
        velN_cms = int((math.cos(heading_rad) * speed_mms) / 10.0)
        velE_cms = int((math.sin(heading_rad) * speed_mms) / 10.0)
        velD_cms = int(random.randint(-100, 100) / 10.0)
        payload = struct.pack('<L', int(self.itow_ms))
        payload += struct.pack('<lll', velN_cms, velE_cms, velD_cms)
        payload += struct.pack('<L', speed_cms)
        payload += struct.pack('<L', speed_cms)
        payload += struct.pack('<l', heading)
        payload += struct.pack('<LL', sacc_cms, cacc)
        return to_ubx(0x01, 0x12, payload)

    def build_nav_sol(self) -> bytes:
        now = datetime.now(timezone.utc)
        week, itow_ms = gps_week_and_tow_ms(now)
        x_m, y_m, z_m = geodetic_to_ecef(self.lat, self.lon, self.h_m)
        x_cm = int(x_m * 100.0); y_cm = int(y_m * 100.0); z_cm = int(z_m * 100.0)
        speed_mms = self.speed_kmh * 1000.0 / 3.6
        vn = math.cos(math.radians(self.heading_deg)) * (speed_mms / 1000.0)
        ve = math.sin(math.radians(self.heading_deg)) * (speed_mms / 1000.0)
        vd = 0.0
        vx_mps, vy_mps, vz_mps = ned_to_ecef(self.lat, self.lon, vn, ve, vd)
        vx_cms = int(vx_mps * 100.0); vy_cms = int(vy_mps * 100.0); vz_cms = int(vz_mps * 100.0)
        pacc_cm = max(1, int(self.hacc_mm / 10.0))
        sacc_cms = max(1, int(self.sacc_mms / 10.0))
        gpsFix = 3 if self.fix_type >= 3 else (2 if self.fix_type == 2 else 0)
        flags = 0x01 if gpsFix >= 2 else 0x00
        payload = struct.pack('<L', itow_ms)
        payload += struct.pack('<l', 0)               # fTOW
        payload += struct.pack('<h', week)
        payload += struct.pack('<B', gpsFix)
        payload += struct.pack('<B', flags)
        payload += struct.pack('<lll', x_cm, y_cm, z_cm)
        payload += struct.pack('<L', pacc_cm)
        payload += struct.pack('<lll', vx_cms, vy_cms, vz_cms)
        payload += struct.pack('<L', sacc_cms)
        payload += struct.pack('<H', 120)             # pDOP
        payload += struct.pack('<B', 0)
        payload += struct.pack('<B', self.sats)
        payload += struct.pack('<L', 0)
        return to_ubx(0x01, 0x06, payload)

    # ----- NMEA builders -----
    def _utc_hhmmss_frac(self) -> str:
        now = datetime.now(timezone.utc)
        return f"{now:%H%M%S}.{int(now.microsecond/1000):03d}"

    def _utc_date_ddmmyy(self) -> str:
        now = datetime.now(timezone.utc)
        return f"{now:%d%m%y}"

    def build_nmea_gga(self) -> str:
        t = self._utc_hhmmss_frac()
        lat, ns = deg_to_nmea_lat(self.lat)
        lon, ew = deg_to_nmea_lon(self.lon)
        fixq = '1' if self.fix_type >= 2 else '0'
        nums = f"{max(0, min(12, self.sats)):02d}"
        hdop = "1.2"
        alt = f"{self.h_m:.1f}"
        geoid = "0.0"
        parts = ["GNGGA", t, lat, ns, lon, ew, fixq, nums, hdop, alt, "M", geoid, "M", "", ""]
        core = ",".join(parts)
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_gll(self) -> str:
        t = self._utc_hhmmss_frac()
        lat, ns = deg_to_nmea_lat(self.lat)
        lon, ew = deg_to_nmea_lon(self.lon)
        status = 'A' if self.fix_type >= 2 else 'V'
        mode = 'A' if self.fix_type >= 3 else ('D' if self.fix_type == 2 else 'N')
        parts = ["GNGLL", lat, ns, lon, ew, t, status, mode]
        core = ",".join(parts)
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_gsa(self) -> str:
        mode1 = 'A'
        mode2 = '3' if self.fix_type >= 3 else ('2' if self.fix_type == 2 else '1')
        prns = [f"{i:02d}" for i in range(1, 13)]
        pdop, hdop, vdop = "1.8", "1.0", "1.5"
        parts = ["GNGSA", mode1, mode2] + prns + [pdop, hdop, vdop]
        core = ",".join(parts)
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_gsv(self) -> str:
        total = 1
        msg_no = 1
        in_view = max(0, self.sats)
        sats = []
        for i in range(1, min(4, in_view) + 1):
            sats += [
                f"{i:02d}",
                f"{random.randint(10, 80):02d}",
                f"{random.randint(0, 359):03d}",
                f"{random.randint(20, 45):02d}",
            ]
        while len(sats) < 16:
            sats.append("")
        parts = ["GNGSV", str(total), str(msg_no), f"{in_view:02d}"] + sats[:16]
        core = ",".join(parts)
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_rmc(self) -> str:
        t = self._utc_hhmmss_frac()
        status = 'A' if self.fix_type >= 2 else 'V'
        lat, ns = deg_to_nmea_lat(self.lat)
        lon, ew = deg_to_nmea_lon(self.lon)
        spd_kn = f"{knots_from_kmh(self.speed_kmh):.1f}"
        cog = f"{self.heading_deg:.1f}"
        date = self._utc_date_ddmmyy()
        magv = ""
        mode = 'A' if self.fix_type >= 3 else ('D' if self.fix_type == 2 else 'N')
        parts = ["GNRMC", t, status, lat, ns, lon, ew, spd_kn, cog, date, magv, "", mode]
        core = ",".join(parts)
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_vtg(self) -> str:
        cog_t = f"{self.heading_deg:.1f}"
        cog_m = ""
        spd_kn = f"{knots_from_kmh(self.speed_kmh):.1f}"
        spd_kmh = f"{self.speed_kmh:.1f}"
        mode = 'A' if self.fix_type >= 3 else ('D' if self.fix_type == 2 else 'N')
        parts = ["GNVTG", cog_t, "T", cog_m, "M", spd_kn, "N", spd_kmh, "K", mode]
        core = ",".join(parts)
        return f"${core}*{nmea_checksum(core)}\r\n"

    # ----- profile helpers -----
    def _profiles_cycle(self) -> List[str]:
        return [p for p in self.order if self.PROFILES.get(p, {}).get("enabled", False)]

    def _advance_profile_if_needed(self, now: float):
        info = self.PROFILES.get(self.current_profile, {"duration": 5.0})
        dur = max(0.1, float(info.get("duration", 5.0)))
        if now - self.profile_start > dur:
            cyc = self._profiles_cycle()
            if not cyc:
                return
            if self.current_profile not in cyc:
                self.current_profile = cyc[0]
            else:
                idx = cyc.index(self.current_profile)
                self.current_profile = cyc[(idx + 1) % len(cyc)]
            self.profile_start = now

    def _target_speed(self, now: float) -> float:
        info = self.PROFILES.get(self.current_profile, {"min": 0, "max": 0, "duration": 5.0})
        mn, mx = float(info["min"]), float(info["max"])
        if mn < 0:  # NO_FIX / GPS_DISABLE markers
            return mn
        if mn == 0 and mx == 0:
            return 0.0
        el = now - self.profile_start
        dur = max(0.1, float(info["duration"]))
        progress = min(1.0, el / dur)
        return max(0.0, mn + (mx - mn) * progress + random.uniform(-2.0, 2.0))

    def _emit_profile(self, now: float):
        info = self.PROFILES.get(self.current_profile, {"duration": 5.0})
        dur = max(0.1, float(info.get("duration", 5.0)))
        progress = min(1.0, max(0.0, (now - self.profile_start) / dur))
        is_lost = self.current_profile in ("NO_FIX", "GPS_DISABLE")  # gray progressbar
        self.profile_cb({"name": self.current_profile, "progress": progress, "is_lost": is_lost})

    def _emit_stats(self):
        now_utc = datetime.now(timezone.utc)
        now_loc = datetime.now()
        self.stats_cb(
            {
                "utc": now_utc.strftime("%Y-%m-%d %H:%M:%S UTC"),
                "local": now_loc.strftime("%Y-%m-%d %H:%M:%S"),
                "speed": float(self.speed_kmh),
                "heading": float(self.heading_deg),
                "sats": int(self.sats),
                "fix": int(self.fix_type),
                "lat": float(self.lat),
                "lon": float(self.lon),
            }
        )

    # ----- main loop -----
    def run(self):
        next_send = time.perf_counter() + self.send_interval
        next_rate = time.perf_counter() + 1.0
        try:
            while not self.stop_event.is_set():
                # control
                try:
                    while True:
                        cmd = self.ctrl_q.get_nowait()
                        if isinstance(cmd, dict) and cmd.get("cmd") == "reset":
                            new_cfg = cmd.get("cfg", {})
                            if serial is not None and (
                                new_cfg.get("port") != self.cfg.get("port")
                                or new_cfg.get("baudrate") != self.cfg.get("baudrate")
                            ):
                                self._open_serial(new_cfg["port"], new_cfg["baudrate"])
                            self._apply_config(new_cfg, reset_all=True)
                            now = time.perf_counter()
                            next_send = now + self.send_interval
                            next_rate = now + 1.0
                except queue.Empty:
                    pass

                now = time.perf_counter()
                self._advance_profile_if_needed(now)
                self._emit_profile(now)

                # quality & modes
                active_send = True
                if self.current_profile == "GPS_DISABLE":
                    active_send = False
                    self.fix_type = 0
                    self.sats = random.randint(0, 2)
                elif self.current_profile == "NO_FIX":
                    self.fix_type = 0
                    self.sats = random.randint(0, 4)
                    self.hacc_mm = random.randint(20000, 50000)
                    self.sacc_mms = random.randint(1500, 5000)
                else:
                    if random.random() < 0.05:
                        self.fix_type = 2
                        self.sats = random.randint(4, 8)
                        self.hacc_mm = random.randint(5000, 15000)
                        self.sacc_mms = random.randint(500, 2000)
                    else:
                        self.fix_type = 3
                        self.sats = random.randint(8, 16)
                        self.hacc_mm = random.randint(1000, 5000)
                        self.sacc_mms = random.randint(200, 800)
                    if random.random() < 0.002:
                        self.fix_type = 0
                        self.sats = random.randint(0, 4)

                tgt = self._target_speed(now)
                if tgt >= 0:
                    delta = tgt - self.speed_kmh
                    self.speed_kmh += (delta * 0.1) if abs(delta) > 0.1 else delta
                    self.speed_kmh = max(0.0, self.speed_kmh)
                    self.heading_deg = (self.heading_deg + random.uniform(-1, 1)) % 360
                    if self.speed_kmh > 0.1:
                        factor = self.speed_kmh / 100.0
                        self.lat += random.uniform(-0.0001, 0.0001) * factor
                        self.lon += random.uniform(-0.0001, 0.0001) * factor

                self.itow_ms = (self.itow_ms + int(self.send_interval * 1000)) % 604_800_000
                self._emit_stats()

                # Send tick
                nmea_map: Dict[str, str] = {}
                ubx_map: Dict[str, str] = {}

                if active_send and now >= next_send:
                    # UBX
                    if self.ubx_en.get("pvt", False):
                        pkt = self.build_nav_pvt()
                        try:
                            if self.ser:
                                self.ser.write(pkt)
                        except Exception:
                            pass
                        self._sent_counts["pvt"] += 1
                        ubx_map["pvt"] = pkt.hex().upper()
                    if self.ubx_en.get("velned", False):
                        pkt = self.build_nav_velned()
                        try:
                            if self.ser:
                                self.ser.write(pkt)
                        except Exception:
                            pass
                        self._sent_counts["velned"] += 1
                        ubx_map["velned"] = pkt.hex().upper()
                    if self.ubx_en.get("sol", False):
                        pkt = self.build_nav_sol()
                        try:
                            if self.ser:
                                self.ser.write(pkt)
                        except Exception:
                            pass
                        self._sent_counts["sol"] += 1
                        ubx_map["sol"] = pkt.hex().upper()

                    # NMEA
                    if self.nmea_en.get("gga", False):
                        s = self.build_nmea_gga()
                        try:
                            if self.ser:
                                self.ser.write(s.encode("ascii"))
                        except Exception:
                            pass
                        self._sent_counts["gga"] += 1
                        nmea_map["gga"] = s.strip()
                    if self.nmea_en.get("gll", False):
                        s = self.build_nmea_gll()
                        try:
                            if self.ser:
                                self.ser.write(s.encode("ascii"))
                        except Exception:
                            pass
                        self._sent_counts["gll"] += 1
                        nmea_map["gll"] = s.strip()
                    if self.nmea_en.get("gsa", False):
                        s = self.build_nmea_gsa()
                        try:
                            if self.ser:
                                self.ser.write(s.encode("ascii"))
                        except Exception:
                            pass
                        self._sent_counts["gsa"] += 1
                        nmea_map["gsa"] = s.strip()
                    if self.nmea_en.get("gsv", False):
                        s = self.build_nmea_gsv()
                        try:
                            if self.ser:
                                self.ser.write(s.encode("ascii"))
                        except Exception:
                            pass
                        self._sent_counts["gsv"] += 1
                        nmea_map["gsv"] = s.strip()
                    if self.nmea_en.get("rmc", False):
                        s = self.build_nmea_rmc()
                        try:
                            if self.ser:
                                self.ser.write(s.encode("ascii"))
                        except Exception:
                            pass
                        self._sent_counts["rmc"] += 1
                        nmea_map["rmc"] = s.strip()
                    if self.nmea_en.get("vtg", False):
                        s = self.build_nmea_vtg()
                        try:
                            if self.ser:
                                self.ser.write(s.encode("ascii"))
                        except Exception:
                            pass
                        self._sent_counts["vtg"] += 1
                        nmea_map["vtg"] = s.strip()

                    if nmea_map or ubx_map:
                        self.tx_cb(nmea_map, ubx_map)

                    skips = max(1, int((now - next_send) // self.send_interval) + 1)
                    next_send += skips * self.send_interval

                if now >= next_rate:
                    elapsed = max(1e-6, now - self._last_rate_time)
                    for k in self._sent_counts:
                        self._actual_rates[k] = self._sent_counts[k] / elapsed
                        self._sent_counts[k] = 0
                    self._last_rate_time = now
                    self.rate_cb(dict(self._actual_rates))
                    steps = max(1, int((now - next_rate) // 1.0) + 1)
                    next_rate += steps * 1.0

                self._emit_profile(time.perf_counter())
                time.sleep(0.01)
        finally:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass


# =========================
# Tkinter GUI
# =========================

class SimulatorGUI:
    UND = "______"  # placeholder for lost/disabled

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("UBX + NMEA GPS Simulator")
        self.root.geometry("1180x900")
        self.root.minsize(1180, 900)
        self.root.grid_columnconfigure(0, weight=1)

        # Fonts
        self.mono_font = ("Consolas", 10)
        self.big_speed_font = ("Segoe UI", 16, "bold")
        self.active_prof_font = ("Segoe UI", 14, "bold")

        # Colors
        self.col_green = "#2e7d32"
        self.col_red   = "#c62828"
        self.col_gray  = "#9e9e9e"
        self.col_text  = "#000000"   # default text color for status area

        # Queues
        self.rate_queue = queue.Queue()
        self.profile_queue = queue.Queue()
        self.stats_queue = queue.Queue()
        self.tx_queue = queue.Queue()
        self.ctrl_queue = queue.Queue()

        # Thread control
        self.sim_thread = None
        self.stop_event = threading.Event()
        self.sim_running = False

        # Dirty tracking
        self._last_applied_config = None
        self._dirty = False

        # Profile tracking
        self._current_profile_name = "—"
        self._current_is_lost = False

        # Progressbar smoothing (60fps)
        self._prog_value = 0.0      # current 0..1
        self._prog_target = 0.0     # target 0..1
        self._prog_last_t = time.perf_counter()
        self._prog_alpha_per_s = 6.0  # higher = snappier
        self.root.after(16, self._animate_progress)  # start 60fps animator

        # --- Top bar: Port + Baud ---
        top = ttk.Frame(root, padding=8)
        top.grid(row=0, column=0, sticky="ew")
        for c, w in enumerate([60, 260, 90, 80, 240]):
            top.grid_columnconfigure(c, minsize=w)

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w", padx=(0, 6))
        self.port_var = tk.StringVar(value="")
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=20, state="readonly")
        self.port_cb.grid(row=0, column=1, sticky="w")
        self.port_cb.bind("<<ComboboxSelected>>", self._on_port_selected)

        ttk.Button(top, text="Refresh", command=self.refresh_ports, width=10).grid(
            row=0, column=2, padx=(6, 12), sticky="w"
        )

        ttk.Label(top, text="Baudrate:").grid(row=0, column=3, sticky="e")
        self.baud_var = tk.StringVar(value="115200")
        self.baud_cb = ttk.Combobox(
            top,
            textvariable=self.baud_var,
            width=12,
            state="disabled",  # enabled after port selected
            values=["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"],
        )
        self.baud_cb.grid(row=0, column=4, sticky="w", padx=(6, 0))
        self.baud_cb.bind("<<ComboboxSelected>>", self._on_any_setting_changed)

        # --- Message settings (masters aligned with children) ---
        msg = ttk.LabelFrame(root, text="Message settings", padding=8)
        msg.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        for c in range(6):
            msg.grid_columnconfigure(c, minsize=180)

        ttk.Label(msg, text="Update rate (Hz):").grid(row=0, column=0, sticky="w")
        self.hz_var = tk.StringVar(value="10.0")
        self.hz_entry = ttk.Entry(msg, textvariable=self.hz_var, width=10)
        self.hz_entry.grid(row=0, column=1, sticky="w", padx=(6, 18))
        self.hz_entry.bind("<KeyRelease>", self._on_any_setting_changed)

        # === UBX group: master + children in the same grid ===
        self.ubx_master_var = tk.BooleanVar(value=True)
        self.ubx_group = ttk.Frame(msg)
        self.ubx_group.grid(row=1, column=0, columnspan=6, sticky="ew", pady=(6, 4))
        for c in range(6):
            self.ubx_group.grid_columnconfigure(c, minsize=180)

        self.ubx_master_chk = ttk.Checkbutton(
            self.ubx_group, text="Enable UBX messages",
            variable=self.ubx_master_var, command=self._on_master_toggled
        )
        self.ubx_master_chk.grid(row=0, column=0, sticky="w")

        ttk.Separator(self.ubx_group, orient="horizontal").grid(
            row=1, column=0, columnspan=6, sticky="ew", pady=(4, 6)
        )

        self.pvt_var = tk.BooleanVar(value=True)
        self.vel_var = tk.BooleanVar(value=False)
        self.sol_var = tk.BooleanVar(value=False)

        def add_ubx_cell(parent, row: int, col_pair: int, text: str, var: tk.BooleanVar):
            def on_child():
                if not self.ubx_master_var.get():
                    self.ubx_master_var.set(True)
                self._on_any_setting_changed()
            chk = ttk.Checkbutton(parent, text=text, variable=var, command=on_child)
            lbl = ttk.Label(parent, text="", font=self.mono_font, width=16)
            c = col_pair * 2
            chk.grid(row=row, column=c, sticky="w")
            lbl.grid(row=row, column=c + 1, sticky="w")
            return lbl

        self.pvt_rate = add_ubx_cell(self.ubx_group, 2, 0, "UBX-NAV-PVT", self.pvt_var)
        self.vel_rate = add_ubx_cell(self.ubx_group, 2, 1, "UBX-NAV-VELNED", self.vel_var)
        self.sol_rate = add_ubx_cell(self.ubx_group, 2, 2, "UBX-NAV-SOL", self.sol_var)

        # Separator between UBX and NMEA groups
        ttk.Separator(msg, orient="horizontal").grid(
            row=2, column=0, columnspan=6, sticky="ew", pady=(6, 6)
        )

        # === NMEA group: master + children in the same grid ===
        self.nmea_master_var = tk.BooleanVar(value=False)
        self.nmea_group = ttk.Frame(msg)
        self.nmea_group.grid(row=3, column=0, columnspan=6, sticky="ew", pady=(0, 2))
        for c in range(6):
            self.nmea_group.grid_columnconfigure(c, minsize=180)

        self.nmea_master_chk = ttk.Checkbutton(
            self.nmea_group, text="Enable NMEA messages",
            variable=self.nmea_master_var, command=self._on_master_toggled
        )
        self.nmea_master_chk.grid(row=0, column=0, sticky="w")

        ttk.Separator(self.nmea_group, orient="horizontal").grid(
            row=1, column=0, columnspan=6, sticky="ew", pady=(4, 6)
        )

        self.nmea_vars: Dict[str, tk.BooleanVar] = {}
        self.nmea_rate_labels: Dict[str, ttk.Label] = {}

        def add_nmea_cell(parent, row: int, col_pair: int, name: str, text: str):
            var = tk.BooleanVar(value=False)
            def on_child():
                if not self.nmea_master_var.get():
                    self.nmea_master_var.set(True)
                self._on_any_setting_changed()
            chk = ttk.Checkbutton(parent, text=text, variable=var, command=on_child)
            lbl = ttk.Label(parent, text="", font=self.mono_font, width=16)
            c = col_pair * 2
            chk.grid(row=row, column=c, sticky="w")
            lbl.grid(row=row, column=c + 1, sticky="w")
            self.nmea_vars[name] = var
            self.nmea_rate_labels[name] = lbl

        add_nmea_cell(self.nmea_group, 2, 0, "gga", "NMEA GGA")
        add_nmea_cell(self.nmea_group, 2, 1, "gll", "NMEA GLL")
        add_nmea_cell(self.nmea_group, 2, 2, "gsa", "NMEA GSA")
        add_nmea_cell(self.nmea_group, 3, 0, "gsv", "NMEA GSV")
        add_nmea_cell(self.nmea_group, 3, 1, "rmc", "NMEA RMC")
        add_nmea_cell(self.nmea_group, 3, 2, "vtg", "NMEA VTG")

        # --- Profile settings (table + active profile) ---
        prof_set = ttk.LabelFrame(root, text="Profile settings", padding=8)
        prof_set.grid(row=2, column=0, sticky="ew", padx=8, pady=(0, 8))
        headers = ["Enable", "Profile", "Min (km/h)", "Max (km/h)", "Duration (s)"]
        for i, h in enumerate(headers):
            ttk.Label(prof_set, text=h).grid(row=0, column=i, sticky="w", padx=(0, 8))
        for c, w in enumerate([80, 220, 140, 140, 160]):
            prof_set.grid_columnconfigure(c, minsize=w)

        self.profile_rows = {}

        def add_profile_row(r, name, enabled, mn, mx, dur, lock_minmax=False):
            en_var = tk.BooleanVar(value=enabled)
            chk = ttk.Checkbutton(prof_set, variable=en_var, command=self._on_any_setting_changed)
            chk.grid(row=r, column=0, sticky="w")
            ttk.Label(prof_set, text=name).grid(row=r, column=1, sticky="w")

            mn_var = tk.StringVar(value=str(mn))
            mx_var = tk.StringVar(value=str(mx))
            du_var = tk.StringVar(value=str(dur))

            e1 = ttk.Entry(prof_set, textvariable=mn_var, width=12, state=("disabled" if lock_minmax else "normal"))
            e2 = ttk.Entry(prof_set, textvariable=mx_var, width=12, state=("disabled" if lock_minmax else "normal"))
            e3 = ttk.Entry(prof_set, textvariable=du_var, width=14)

            e1.grid(row=r, column=2, sticky="w")
            e2.grid(row=r, column=3, sticky="w")
            e3.grid(row=r, column=4, sticky="w")
            e3.bind("<KeyRelease>", self._on_any_setting_changed)
            if not lock_minmax:
                e1.bind("<KeyRelease>", self._on_any_setting_changed)
                e2.bind("<KeyRelease>", self._on_any_setting_changed)
            self.profile_rows[name] = (en_var, mn_var, mx_var, du_var)

        rows = [
            ("STATIONARY", True, 0, 0, 5.0, True),
            ("WALKING", True, 1, 8, 5.0, False),
            ("CITY_DRIVING", True, 10, 60, 5.0, False),
            ("HIGHWAY_DRIVING", True, 60, 100, 5.0, False),
            ("HIGH_SPEED", True, 100, 399, 5.0, False),
            ("NO_FIX", True, -1, -1, 5.0, True),
            ("GPS_DISABLE", True, -2, -2, 10.0, True),
        ]
        for i, (n, e, mn, mx, du, lock) in enumerate(rows, start=1):
            add_profile_row(i, n, e, mn, mx, du, lock_minmax=lock)

        # Active profile inside Profile settings
        style = ttk.Style()
        style.configure("Profile.Normal.Horizontal.TProgressbar", troughcolor="#f0f0f0")
        style.configure("Profile.Lost.Horizontal.TProgressbar", troughcolor="#f0f0f0", background="#9e9e9e")

        prof_set.grid_rowconfigure(len(rows) + 1, minsize=12)
        self.active_prof_label = ttk.Label(prof_set, text="Active profile:", font=self.active_prof_font, width=16, anchor="w")
        self.active_prof_value = ttk.Label(prof_set, text="—", font=self.active_prof_font, width=20, anchor="w")
        self.active_prog = ttk.Progressbar(
            prof_set, orient="horizontal", mode="determinate",
            maximum=1000, value=0,
            style="Profile.Normal.Horizontal.TProgressbar",
        )
        base_row = len(rows) + 2
        self.active_prof_label.grid(row=base_row, column=0, sticky="w", pady=(6, 0))
        self.active_prof_value.grid(row=base_row, column=1, sticky="w", pady=(6, 0))
        self.active_prog.grid(row=base_row, column=2, columnspan=3, sticky="ew", pady=(6, 0))

        # --- Current status (fixed columns) ---
        stat = ttk.LabelFrame(root, text="Current status", padding=8)
        stat.grid(row=3, column=0, sticky="ew", padx=8, pady=(0, 8))
        for i in range(6):
            stat.grid_columnconfigure(i, minsize=190)

        self.lbl_speed_big = ttk.Label(stat, text="Speed: —", font=self.big_speed_font)
        self.lbl_speed_big.grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 6))

        def mono_label(parent, txt, col, row):
            lbl = ttk.Label(parent, text=txt, font=self.mono_font, anchor="w")
            lbl.grid(row=row, column=col, sticky="w")
            return lbl

        self.lbl_time_utc = mono_label(stat, "Time (UTC): —", 0, 1)
        self.lbl_time_loc = mono_label(stat, "Time (Local): —", 1, 1)
        self.lbl_heading  = mono_label(stat, "Heading: —°", 2, 1)
        self.lbl_sats     = mono_label(stat, "Sats: —", 3, 1)

        # FIX text label (color-coded & descriptive)
        self.lbl_fix_status = ttk.Label(stat, text="FIX: —", font=("Segoe UI", 11, "bold"), foreground=self.col_gray)
        self.lbl_fix_status.grid(row=1, column=4, sticky="w")

        self.lbl_lat      = mono_label(stat, "Lat: —", 0, 2)
        self.lbl_lon      = mono_label(stat, "Lon: —", 1, 2)

        # --- Buttons row ---
        btns = ttk.Frame(root, padding=(8, 0, 8, 8))
        btns.grid(row=4, column=0, sticky="ew")
        for c, w in enumerate([160, 200, 360]):
            btns.grid_columnconfigure(c, minsize=w)

        self.start_btn = ttk.Button(btns, text="Start", command=self.start_sim, state="disabled")
        self.stop_btn = ttk.Button(btns, text="Stop", command=self.stop_sim, state="disabled")
        self.apply_btn = ttk.Button(btns, text="Apply to running (reset)", command=self.apply_to_running, state="disabled")
        self.start_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.stop_btn.grid(row=0, column=1, sticky="ew", padx=(4, 4))
        self.apply_btn.grid(row=0, column=2, sticky="ew", padx=(4, 0))

        # --- TX Monitor (auto-height) ---
        monitor = ttk.LabelFrame(root, text="TX Monitor (latest)", padding=8)
        monitor.grid(row=5, column=0, sticky="ew", padx=8, pady=(0, 8))
        monitor.grid_columnconfigure(0, weight=1)

        ttk.Label(monitor, text="UBX (hex):").grid(row=0, column=0, sticky="w")
        ubx_frame = ttk.Frame(monitor)
        ubx_frame.grid(row=1, column=0, sticky="ew")
        ubx_frame.grid_columnconfigure(0, weight=1)
        self.ubx_tree = ttk.Treeview(ubx_frame, columns=("msg", "len", "hex"), show="headings", height=1)
        self.ubx_tree.heading("msg", text="Message")
        self.ubx_tree.heading("len", text="Bytes")
        self.ubx_tree.heading("hex", text="Hex (no spaces)")
        self.ubx_tree.column("msg", width=160, anchor="w", stretch=False)
        self.ubx_tree.column("len", width=70, anchor="center", stretch=False)
        self.ubx_tree.column("hex", width=800, anchor="w", stretch=True)
        self.ubx_tree.grid(row=0, column=0, sticky="ew")
        ubx_vsb = ttk.Scrollbar(ubx_frame, orient="vertical", command=self.ubx_tree.yview)
        ubx_hsb = ttk.Scrollbar(ubx_frame, orient="horizontal", command=self.ubx_tree.xview)
        self.ubx_tree.configure(yscroll=ubx_vsb.set, xscroll=ubx_hsb.set)
        ubx_vsb.grid(row=0, column=1, sticky="ns")
        ubx_hsb.grid(row=1, column=0, sticky="ew")

        ttk.Separator(monitor, orient="horizontal").grid(row=2, column=0, sticky="ew", pady=(4, 8))

        ttk.Label(monitor, text="NMEA (text):").grid(row=3, column=0, sticky="w")
        nmea_frame = ttk.Frame(monitor)
        nmea_frame.grid(row=4, column=0, sticky="ew")
        nmea_frame.grid_columnconfigure(0, weight=1)
        self.nmea_tree = ttk.Treeview(nmea_frame, columns=("msg", "text"), show="headings", height=1)
        self.nmea_tree.heading("msg", text="Message")
        self.nmea_tree.heading("text", text="Sentence")
        self.nmea_tree.column("msg", width=160, anchor="w", stretch=False)
        self.nmea_tree.column("text", width=970, anchor="w", stretch=True)
        self.nmea_tree.grid(row=0, column=0, sticky="ew")
        nmea_vsb = ttk.Scrollbar(nmea_frame, orient="vertical", command=self.nmea_tree.yview)
        nmea_hsb = ttk.Scrollbar(nmea_frame, orient="horizontal", command=self.nmea_tree.xview)
        self.nmea_tree.configure(yscroll=nmea_vsb.set, xscroll=nmea_hsb.set)
        nmea_vsb.grid(row=0, column=1, sticky="ns")
        nmea_hsb.grid(row=1, column=0, sticky="ew")

        # Context menus (copy)
        self._build_context_menus()

        # Init ports (do not auto-select)
        self.refresh_ports()
        self._update_start_btn_state()
        self._rebuild_tx_tables()

        # Timers to drain queues (profile at ~60fps)
        self.root.after(120, self._drain_rate_queue)
        self.root.after(16, self._drain_profile_queue)
        self.root.after(120, self._drain_stats_queue)
        self.root.after(80, self._drain_tx_queue)

        # Ensure trees render immediately
        self.root.after(50, self._force_refresh_treeviews)

    # ----- context menus -----
    def _build_context_menus(self):
        self.ubx_menu = tk.Menu(self.root, tearoff=0)
        self.ubx_menu.add_command(label="Copy Payload", command=self._copy_selected_ubx)
        self.ubx_menu.add_command(label="Copy Row", command=self._copy_selected_ubx_row)
        self.ubx_menu.add_separator()
        self.ubx_menu.add_command(label="Copy All", command=self._copy_all_ubx)
        self.ubx_tree.bind("<Button-3>", self._popup_ubx_menu)
        self.ubx_tree.bind("<Control-c>", lambda e: self._copy_selected_ubx())
        self.ubx_tree.bind("<Double-1>", lambda e: self._copy_selected_ubx())

        self.nmea_menu = tk.Menu(self.root, tearoff=0)
        self.nmea_menu.add_command(label="Copy Payload", command=self._copy_selected_nmea)
        self.nmea_menu.add_command(label="Copy Row", command=self._copy_selected_nmea_row)
        self.nmea_menu.add_separator()
        self.nmea_menu.add_command(label="Copy All", command=self._copy_all_nmea)
        self.nmea_tree.bind("<Button-3>", self._popup_nmea_menu)
        self.nmea_tree.bind("<Control-c>", lambda e: self._copy_selected_nmea())
        self.nmea_tree.bind("<Double-1>", lambda e: self._copy_selected_nmea())

    def _popup_ubx_menu(self, event):
        iid = self.ubx_tree.identify_row(event.y)
        if iid:
            self.ubx_tree.selection_set(iid)
        self.ubx_menu.tk_popup(event.x_root, event.y_root)

    def _popup_nmea_menu(self, event):
        iid = self.nmea_tree.identify_row(event.y)
        if iid:
            self.nmea_tree.selection_set(iid)
        self.nmea_menu.tk_popup(event.x_root, event.y_root)

    # ----- copy helpers -----
    def _copy_selected_ubx(self):
        sel = self.ubx_tree.selection()
        if not sel:
            return
        iid = sel[0]
        vals = self.ubx_tree.item(iid, "values")
        payload = vals[2] if len(vals) >= 3 else ""
        self._to_clipboard(payload)

    def _copy_selected_ubx_row(self):
        sel = self.ubx_tree.selection()
        if not sel:
            return
        iid = sel[0]
        vals = self.ubx_tree.item(iid, "values")
        self._to_clipboard(",".join(vals))

    def _copy_all_ubx(self):
        lines = []
        for iid in self.ubx_tree.get_children():
            lines.append(",".join(self.ubx_tree.item(iid, "values")))
        self._to_clipboard("\n".join(lines))

    def _copy_selected_nmea(self):
        sel = self.nmea_tree.selection()
        if not sel:
            return
        iid = sel[0]
        vals = self.nmea_tree.item(iid, "values")
        payload = vals[1] if len(vals) >= 2 else ""
        self._to_clipboard(payload)

    def _copy_selected_nmea_row(self):
        sel = self.nmea_tree.selection()
        if not sel:
            return
        iid = sel[0]
        vals = self.nmea_tree.item(iid, "values")
        self._to_clipboard(",".join(vals))

    def _copy_all_nmea(self):
        lines = []
        for iid in self.nmea_tree.get_children():
            lines.append(",".join(self.nmea_tree.item(iid, "values")))
        self._to_clipboard("\n".join(lines))

    def _to_clipboard(self, text: str):
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
        except Exception:
            pass

    # ----- utilities -----
    def _force_refresh_treeviews(self):
        self.ubx_tree.update_idletasks()
        self.nmea_tree.update_idletasks()

    def _set_tx_heights(self):
        ubx_count = sum([
            self.ubx_master_var.get() and self.pvt_var.get(),
            self.ubx_master_var.get() and self.vel_var.get(),
            self.ubx_master_var.get() and self.sol_var.get(),
        ])
        nmea_count = sum(1 for v in self.nmea_vars.values() if self.nmea_master_var.get() and v.get())
        self.ubx_tree.configure(height=max(1, ubx_count))
        self.nmea_tree.configure(height=max(1, nmea_count))

    def _rebuild_tx_tables(self):
        for tree in (self.ubx_tree, self.nmea_tree):
            for iid in tree.get_children():
                tree.delete(iid)

        if self.ubx_master_var.get():
            if self.pvt_var.get():
                self.ubx_tree.insert("", "end", iid="pvt", values=("UBX-NAV-PVT", "", ""))
            if self.vel_var.get():
                self.ubx_tree.insert("", "end", iid="velned", values=("UBX-NAV-VELNED", "", ""))
            if self.sol_var.get():
                self.ubx_tree.insert("", "end", iid="sol", values=("UBX-NAV-SOL", "", ""))

        if self.nmea_master_var.get():
            for key, label in (
                ("gga", "NMEA GGA"), ("gll", "NMEA GLL"), ("gsa", "NMEA GSA"),
                ("gsv", "NMEA GSV"), ("rmc", "NMEA RMC"), ("vtg", "NMEA VTG"),
            ):
                if self.nmea_vars.get(key, tk.BooleanVar()).get():
                    self.nmea_tree.insert("", "end", iid=key, values=(label, ""))

        self._set_tx_heights()
        self._force_refresh_treeviews()
        self._auto_resize_to_fit()  # action-triggered auto-resize

    def _clear_status_and_tx(self):
        # reset colors & progress animation
        self._set_status_area_color(self.col_text, include_fix=True)
        self._prog_value = 0.0
        self._prog_target = 0.0
        self._prog_last_t = time.perf_counter()

        self.lbl_speed_big.config(text="Speed: —")
        self.lbl_time_utc.config(text="Time (UTC): —")
        self.lbl_time_loc.config(text="Time (Local): —")
        self.lbl_heading.config(text="Heading: —°")
        self.lbl_sats.config(text="Sats: —")
        self._set_fix_text("FIX: —", self.col_gray)
        self.lbl_lat.config(text="Lat: —")
        self.lbl_lon.config(text="Lon: —")
        self.active_prof_value.config(text="—")
        self.active_prog["value"] = 0

        for iid in self.ubx_tree.get_children():
            vals = list(self.ubx_tree.item(iid, "values"))
            vals[1] = ""  # len
            vals[2] = ""  # hex
            self.ubx_tree.item(iid, values=vals)
        for iid in self.nmea_tree.get_children():
            vals = list(self.nmea_tree.item(iid, "values"))
            vals[1] = ""  # sentence
            self.nmea_tree.item(iid, values=vals)
        self._force_refresh_treeviews()

    # ----- status area coloring helpers -----
    def _set_status_area_color(self, color: str, include_fix: bool = False):
        labels = [
            self.lbl_speed_big, self.lbl_time_utc, self.lbl_time_loc,
            self.lbl_heading, self.lbl_sats, self.lbl_lat, self.lbl_lon
        ]
        for lbl in labels:
            try:
                lbl.configure(foreground=color)
            except Exception:
                pass
        if include_fix:
            try:
                self.lbl_fix_status.configure(foreground=color)
            except Exception:
                pass

    def _reset_status_area_basecolor(self):
        self._set_status_area_color(self.col_text, include_fix=False)

    # ----- auto-resize window ON ACTION -----
    def _auto_resize_to_fit(self):
        try:
            ubx_rows = max(1, sum([
                self.ubx_master_var.get() and self.pvt_var.get(),
                self.ubx_master_var.get() and self.vel_var.get(),
                self.ubx_master_var.get() and self.sol_var.get(),
            ]))
            nmea_rows = max(1, sum(1 for v in self.nmea_vars.values() if self.nmea_master_var.get() and v.get()))
            per_row = 22
            base = 820  # base layout height without extra TX rows
            desired = base + (ubx_rows - 1) * per_row + (nmea_rows - 1) * per_row
            screen_h = self.root.winfo_screenheight()
            desired = min(desired, screen_h - 80)
            cur_w = self.root.winfo_width() or 1180
            self.root.geometry(f"{cur_w}x{int(max(self.root.winfo_height(), desired))}")
        except Exception:
            pass

    # ----- dirty / master / start enable -----
    def _mark_dirty(self, dirty=True):
        self._dirty = bool(dirty)
        self.apply_btn.configure(state=("normal" if (self.sim_running and self._dirty) else "disabled"))

    def _on_any_setting_changed(self, *_):
        self._mark_dirty(True)
        self._update_start_btn_state()
        self._rebuild_tx_tables()

    def _on_master_toggled(self):
        if not self.ubx_master_var.get():
            self.pvt_var.set(False)
            self.vel_var.set(False)
            self.sol_var.set(False)
        if not self.nmea_master_var.get():
            for v in self.nmea_vars.values():
                v.set(False)
        self._mark_dirty(True)
        self._update_start_btn_state()
        self._rebuild_tx_tables()

    def _on_port_selected(self, *_):
        has_port = bool(self.port_var.get().strip())
        self.baud_cb.configure(state=("readonly" if has_port else "disabled"))
        self._mark_dirty(True)
        self._update_start_btn_state()

    def _has_any_message_selected(self) -> bool:
        any_ubx = self.ubx_master_var.get() and (self.pvt_var.get() or self.vel_var.get() or self.sol_var.get())
        any_nmea = self.nmea_master_var.get() and any(v.get() for v in self.nmea_vars.values())
        return any_ubx or any_nmea

    def _update_start_btn_state(self):
        if self.sim_running:
            self.start_btn.configure(state="disabled")
            return
        cfg_ok = self._current_config(validate_messages=False) is not None
        has_msgs = self._has_any_message_selected()
        self.start_btn.configure(state=("normal" if (cfg_ok and has_msgs) else "disabled"))

    # ----- config -----
    def refresh_ports(self):
        if serial is None:
            self.port_cb["values"] = []
            self.port_var.set("")
            self.baud_cb.configure(state="disabled")
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        # Keep blank until user picks one
        if self.port_var.get() not in ports:
            self.port_var.set("")
            self.baud_cb.configure(state="disabled")

    def _current_config(self, validate_messages: bool = True) -> Optional[dict]:
        port = self.port_var.get().strip()
        if not port:
            return None
        try:
            baudrate = int(self.baud_var.get())
        except ValueError:
            return None
        try:
            hz = float(self.hz_var.get())
            if hz <= 0:
                raise ValueError
        except ValueError:
            return None

        ubx_enable = {
            "pvt": bool(self.pvt_var.get()) if self.ubx_master_var.get() else False,
            "velned": bool(self.vel_var.get()) if self.ubx_master_var.get() else False,
            "sol": bool(self.sol_var.get()) if self.ubx_master_var.get() else False,
        }
        nmea_enable = {k: (bool(v.get()) if self.nmea_master_var.get() else False) for k, v in self.nmea_vars.items()}

        if validate_messages and not any(ubx_enable.values()) and not any(nmea_enable.values()):
            return None

        profiles = {}
        for name, (en_v, mn_v, mx_v, du_v) in self.profile_rows.items():
            try:
                profiles[name] = {
                    "enabled": bool(en_v.get()),
                    "min": float(mn_v.get()),
                    "max": float(mx_v.get()),
                    "duration": max(0.1, float(du_v.get())),
                }
            except ValueError:
                messagebox.showerror("Error", f"Invalid numbers in profile '{name}'.")
                return None

        return {
            "port": port,
            "baudrate": baudrate,
            "update_rate_hz": hz,
            "ubx_enable": ubx_enable,
            "nmea_enable": nmea_enable,
            "profiles": profiles,
        }

    # ----- queue draining -----
    def _drain_rate_queue(self):
        try:
            while True:
                rates = self.rate_queue.get_nowait()
                if self.sim_running:
                    self.pvt_rate.config(text=(f"act: {rates.get('pvt', 0.0):.2f} Hz" if self.pvt_var.get() and self.ubx_master_var.get() else ""))
                    self.vel_rate.config(text=(f"act: {rates.get('velned', 0.0):.2f} Hz" if self.vel_var.get() and self.ubx_master_var.get() else ""))
                    self.sol_rate.config(text=(f"act: {rates.get('sol', 0.0):.2f} Hz" if self.sol_var.get() and self.ubx_master_var.get() else ""))
                    for key in ("gga", "gll", "gsa", "gsv", "rmc", "vtg"):
                        lbl = self.nmea_rate_labels.get(key)
                        var = self.nmea_vars.get(key)
                        if lbl is not None and var is not None:
                            lbl.config(text=(f"act: {rates.get(key, 0.0):.2f} Hz" if var.get() and self.nmea_master_var.get() else ""))
        except queue.Empty:
            pass
        self.root.after(120, self._drain_rate_queue)

    def _drain_profile_queue(self):
        try:
            while True:
                info = self.profile_queue.get_nowait()
                if self.sim_running:
                    self._current_is_lost = bool(info.get("is_lost", False))
                    self._current_profile_name = info.get("name", "—")
                    self.active_prof_value.config(text=self._current_profile_name)

                    # target progress 0..1 for animator
                    t = float(info.get("progress", 0.0))
                    self._prog_target = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)

                    # style applied by animator; keep in sync here too
                    self.active_prog.configure(
                        style=("Profile.Lost.Horizontal.TProgressbar" if self._current_is_lost
                               else "Profile.Normal.Horizontal.TProgressbar")
                    )
        except queue.Empty:
            pass
        self.root.after(16, self._drain_profile_queue)  # ~60fps feed

    def _set_fix_text(self, text: str, color: str):
        self.lbl_fix_status.configure(text=text, foreground=color)

    def _drain_stats_queue(self):
        try:
            while True:
                s = self.stats_queue.get_nowait()
                if self.sim_running:
                    prof = self._current_profile_name
                    if prof == "GPS_DISABLE":
                        und = self.UND
                        self.lbl_speed_big.config(text=f"Speed: {und}")
                        self.lbl_time_utc.config(text=f"Time (UTC): {und}")
                        self.lbl_time_loc.config(text=f"Time (Local): {und}")
                        self.lbl_heading.config(text=f"Heading: {und}")
                        self.lbl_sats.config(text=f"Sats: {und}")
                        self.lbl_lat.config(text=f"Lat: {und}")
                        self.lbl_lon.config(text=f"Lon: {und}")
                        self._set_fix_text("FIX: GPS DISABLED", self.col_gray)
                        self._set_status_area_color(self.col_gray, include_fix=True)
                    elif prof == "NO_FIX":
                        und = self.UND
                        self.lbl_speed_big.config(text=f"Speed: {und}")
                        self.lbl_time_utc.config(text=f"Time (UTC): {und}")
                        self.lbl_time_loc.config(text=f"Time (Local): {und}")
                        self.lbl_heading.config(text=f"Heading: {und}")
                        self.lbl_sats.config(text=f"Sats: {int(s.get('sats', 0))}")
                        self.lbl_lat.config(text=f"Lat: {und}")
                        self.lbl_lon.config(text=f"Lon: {und}")
                        self._reset_status_area_basecolor()
                        self._set_fix_text("FIX: NO FIX", self.col_red)
                    else:
                        self._reset_status_area_basecolor()
                        spd = s.get("speed", 0.0)
                        fx = int(s.get("fix", 0))
                        self.lbl_speed_big.config(text=f"Speed: {spd:.1f} km/h")
                        self.lbl_time_utc.config(text=f"Time (UTC): {s.get('utc', '—')}")
                        self.lbl_time_loc.config(text=f"Time (Local): {s.get('local', '—')}")
                        self.lbl_heading.config(text=f"Heading: {s.get('heading', 0.0):.1f}°")
                        self.lbl_sats.config(text=f"Sats: {int(s.get('sats', 0))}")
                        self.lbl_lat.config(text=f"Lat: {s.get('lat', 0.0):.6f}")
                        self.lbl_lon.config(text=f"Lon: {s.get('lon', 0.0):.6f}")
                        if fx == 3:
                            self._set_fix_text("FIX: 3D FIX", self.col_green)
                        elif fx == 2:
                            self._set_fix_text("FIX: 2D FIX", self.col_green)
                        else:
                            self._set_fix_text("FIX: NO FIX", self.col_red)
        except queue.Empty:
            pass
        self.root.after(120, self._drain_stats_queue)

    def _drain_tx_queue(self):
        try:
            while True:
                nmea_map, ubx_map = self.tx_queue.get_nowait()
                if self.sim_running:
                    for key, label in (("pvt", "UBX-NAV-PVT"), ("velned", "UBX-NAV-VELNED"), ("sol", "UBX-NAV-SOL")):
                        if key in ubx_map and key in self.ubx_tree.get_children():
                            hexstr = ubx_map[key]
                            bytelen = len(hexstr) // 2
                            self.ubx_tree.item(key, values=(label, str(bytelen), hexstr))
                    for key, label in (
                        ("gga", "NMEA GGA"), ("gll", "NMEA GLL"), ("gsa", "NMEA GSA"),
                        ("gsv", "NMEA GSV"), ("rmc", "NMEA RMC"), ("vtg", "NMEA VTG"),
                    ):
                        if key in nmea_map and key in self.nmea_tree.get_children():
                            self.nmea_tree.item(key, values=(label, nmea_map[key]))
        except queue.Empty:
            pass
        self.root.after(80, self._drain_tx_queue)

    # ----- 60fps animator for progressbar -----
    def _animate_progress(self):
        """60fps animator: ease progress value toward target using time-based smoothing."""
        try:
            now = time.perf_counter()
            dt = max(0.0, min(0.1, now - self._prog_last_t))
            self._prog_last_t = now

            alpha = 1.0 - math.exp(-self._prog_alpha_per_s * dt)
            self._prog_value += (self._prog_target - self._prog_value) * alpha

            if abs(self._prog_target - self._prog_value) < 0.001:
                self._prog_value = self._prog_target

            self.active_prog["value"] = int(self._prog_value * 1000)
            self.active_prog.configure(
                style=("Profile.Lost.Horizontal.TProgressbar" if self._current_is_lost
                       else "Profile.Normal.Horizontal.TProgressbar")
            )
        except Exception:
            pass
        finally:
            self.root.after(16, self._animate_progress)

    # ----- start / stop / apply -----
    def start_sim(self):
        if self.sim_running:
            return
        cfg = self._current_config(validate_messages=True)
        if not cfg:
            messagebox.showerror("Error", "Invalid configuration or no messages selected.")
            return

        self.stop_event = threading.Event()
        self._rebuild_tx_tables()
        self._clear_status_and_tx()

        def rate_cb(r):   self.rate_queue.put(r)
        def prof_cb(p):   self.profile_queue.put(p)
        def stats_cb(s):  self.stats_queue.put(s)
        def tx_cb(n, u):  self.tx_queue.put((n, u))

        def worker():
            try:
                sim = UbxNmeaSimulator(
                    cfg,
                    ctrl_q=self.ctrl_queue,
                    rate_cb=rate_cb,
                    profile_cb=prof_cb,
                    stats_cb=stats_cb,
                    tx_cb=tx_cb,
                    stop_event=self.stop_event,
                )
                sim.run()
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Error", str(e)))
            finally:
                self.root.after(0, lambda: self._toggle_inputs(False))

        self.sim_thread = threading.Thread(target=worker, daemon=True)
        self.sim_thread.start()
        self._last_applied_config = cfg
        self._mark_dirty(False)
        self._toggle_inputs(True)
        self._auto_resize_to_fit()

    def stop_sim(self):
        if not self.sim_running:
            return
        self.stop_event.set()
        # Keep last status & TX snapshot visible

    def apply_to_running(self):
        if not self.sim_running or not self._dirty:
            return
        cfg = self._current_config(validate_messages=True)
        if not cfg:
            messagebox.showerror("Error", "Invalid configuration or no messages selected.")
            return
        self._rebuild_tx_tables()
        try:
            self.ctrl_queue.put_nowait({"cmd": "reset", "cfg": cfg})
        except Exception:
            messagebox.showerror("Error", "Failed to queue apply-to-running request.")
            return
        self._last_applied_config = cfg
        self._mark_dirty(False)
        self._auto_resize_to_fit()

    def _toggle_inputs(self, running: bool):
        self.sim_running = running
        self.start_btn.configure(state=("disabled" if running else ("normal" if self._has_any_message_selected() and self._current_config(validate_messages=False) else "disabled")))
        self.stop_btn.configure(state=("normal" if running else "disabled"))
        self.apply_btn.configure(state=("normal" if (running and self._dirty) else "disabled"))
        self.port_cb.configure(state=("disabled" if running else "readonly"))
        self.baud_cb.configure(state=("disabled" if running else ("readonly" if self.port_var.get() else "disabled")))

    def on_close(self):
        try:
            if self.sim_running:
                self.stop_event.set()
                for _ in range(60):
                    if self.sim_thread and not self.sim_thread.is_alive():
                        break
                    time.sleep(0.05)
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    gui = SimulatorGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
