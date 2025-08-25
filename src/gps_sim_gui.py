#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UBX + NMEA (v4.11) GPS Simulator GUI (Tkinter)

Highlights:
- UI thread (Tkinter) + worker thread (simulator).
- Start only when: Port chosen (initially blank), Baud chosen, ≥1 message enabled.
- "Apply to running" resets the sim & applies current settings (including update rate); button enabled only when settings changed.
- Message settings: master checkboxes (UBX/NMEA) + 3-column children, actual rate per message (UBX & NMEA).
- Profiles: STATIONARY, WALKING, CITY_DRIVING, HIGHWAY_DRIVING, HIGH_SPEED, NO_FIX, GPS_DISABLE.
- Profile durations are integer seconds via Spinbox; 0s => disabled, >0 => enabled (checkbox mirrors duration).
- Progress bar is data-driven (no 60fps animator). Worker emits a final 100% tick before switching profiles (Approach A).
- Status area: big bold Speed, FIX colored (3D FIX/2D FIX green; NO FIX red; GPS DISABLE gray).
- NO_FIX → numeric fields show "______"; GPS_DISABLE → entire status area gray.
- TX Monitor: UBX hex (full, with byte length), NMEA text; rows equal to enabled messages; right-click to copy.
- Window min size stable; auto-resizes once when TX rows count changes (no jitter).
- All comments and labels in English.
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
    serial = None  # UI can run even if pyserial isn't installed


# =========================
# UBX helpers
# =========================


def ubx_checksum(body: bytes):
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def to_ubx(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    length = struct.pack("<H", len(payload))
    body = bytes([msg_class, msg_id]) + length + payload
    ck_a, ck_b = ubx_checksum(body)
    return b"\xb5\x62" + body + bytes([ck_a, ck_b])


def gps_week_and_tow_ms(now_utc: datetime):
    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    delta = now_utc - gps_epoch
    total_ms = int(delta.total_seconds() * 1000)
    week_ms = 7 * 24 * 3600 * 1000
    return total_ms // week_ms, total_ms % week_ms


def geodetic_to_ecef(lat_deg: float, lon_deg: float, h_m: float):
    # WGS-84
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1 - e2) + h_m) * sin_lat
    return x, y, z


def ned_to_ecef(lat_deg: float, lon_deg: float, vn: float, ve: float, vd: float):
    # NED -> ECEF rotation applied to velocity
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    r11 = -sin_lat * cos_lon
    r12 = -sin_lat * sin_lon
    r13 = cos_lat
    r21 = -sin_lon
    r22 = cos_lon
    r23 = 0.0
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
    hemi = "N" if lat >= 0 else "S"
    lat = abs(lat)
    deg = int(lat)
    minutes = (lat - deg) * 60.0
    return f"{deg:02d}{minutes:07.4f}", hemi


def deg_to_nmea_lon(lon: float) -> Tuple[str, str]:
    hemi = "E" if lon >= 0 else "W"
    lon = abs(lon)
    deg = int(lon)
    minutes = (lon - deg) * 60.0
    return f"{deg:03d}{minutes:07.4f}", hemi


def knots_from_kmh(kmh: float) -> float:
    return kmh * 0.539956803


# =========================
# Simulator worker (Approach A implemented)
# =========================


class UbxNmeaSimulator:
    """
    Worker thread that generates UBX & NMEA messages.
    Communicates with UI via callbacks and a command queue.

    Control queue (from UI):
      - {"cmd": "reset", "cfg": <new_config_dict>}

    Callbacks (to UI):
      - rate_cb(dict): actual rates per message key
      - profile_cb(dict): {"name", "progress", "is_lost", "seq"}  # seq increments on profile change
      - stats_cb(dict): {"utc","local","speed","heading","sats","fix","lat","lon"}
      - tx_cb(nmea_map, ubx_map): latest payloads keyed by message name
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
        self.paused = False
        self._total_counts = {
            k: 0
            for k in ["pvt", "velned", "sol", "gga", "gll", "gsa", "gsv", "rmc", "vtg"]
        }

        # Serial open (offline if pyserial missing or open fails)
        self.ser = None
        if serial is not None:
            self._open_serial(self.cfg["port"], self.cfg["baudrate"])

        self._apply_config(self.cfg, reset_all=True)

    # ---- serial ----
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
            self.ser = None  # run offline

    # ---- config & profile state ----
    def _apply_config(self, cfg: dict, reset_all: bool):
        self.cfg = dict(cfg)  # shallow copy
        self.update_rate_hz = max(0.01, cfg["update_rate_hz"])
        self.send_interval = 1.0 / self.update_rate_hz

        self.ubx_en = dict(cfg["ubx_enable"])
        self.nmea_en = dict(cfg["nmea_enable"])

        # Use durations from user settings; enabled means duration > 0
        self.PROFILES = {k: dict(v) for k, v in cfg["profiles"].items()}
        order = [
            "STATIONARY",
            "WALKING",
            "CITY_DRIVING",
            "HIGHWAY_DRIVING",
            "HIGH_SPEED",
            "NO_FIX",
            "GPS_DISABLE",
        ]
        # Effective cycle only includes profiles with duration > 0
        self.cycle = [
            p for p in order if float(self.PROFILES.get(p, {}).get("duration", 0)) > 0.0
        ]
        if not self.cycle:
            # Fallback: STATIONARY 5s if user disabled all via 0s
            self.PROFILES["STATIONARY"] = {"min": 0, "max": 0, "duration": 5}
            self.cycle = ["STATIONARY"]

        self.current_profile = self.cycle[0]
        self.profile_start = time.perf_counter()
        # Approach A tracking:
        self.profile_seq = 0  # increments each time we actually switch to next profile
        self._sent_final_prog = (
            False  # set True after we emit 100% tick; next loop will switch
        )

        if reset_all:
            # Initial state (London)
            self.lat, self.lon, self.h_m = 51.5074, -0.1278, 15.0
            self.speed_kmh, self.heading_deg, self.itow_ms = 0.0, 0.0, 0
            self.fix_type, self.sats = 3, 12
            self.hacc_mm, self.sacc_mms = 2000, 100

        # Actual-rate tracking per message
        keys = ["pvt", "velned", "sol", "gga", "gll", "gsa", "gsv", "rmc", "vtg"]
        self._sent_counts = {k: 0 for k in keys}
        self._actual_rates = {k: 0.0 for k in keys}
        self._last_rate_time = time.perf_counter()
        self.rate_cb(dict(self._actual_rates))

    # ---- UBX builders (time from system clock) ----
    def build_nav_pvt(self) -> bytes:
        now = datetime.now(timezone.utc)
        payload = struct.pack("<L", int(self.itow_ms))
        payload += struct.pack(
            "<HBBBBBB",
            now.year,
            now.month,
            now.day,
            now.hour,
            now.minute,
            now.second,
            0x07,
        )
        payload += struct.pack("<L", 50000)  # tAcc ns
        payload += struct.pack("<l", 0)  # nano
        flags = 0x01 if self.fix_type >= 3 else 0x00
        payload += struct.pack("<BBB", self.fix_type, flags, 0x00)
        payload += struct.pack("<B", self.sats)
        payload += struct.pack("<l", int(self.lon * 1e7))
        payload += struct.pack("<l", int(self.lat * 1e7))
        nowt = time.perf_counter()
        payload += struct.pack("<l", int((self.h_m * 1000.0) + math.sin(nowt) * 5000))
        payload += struct.pack(
            "<l", int((self.h_m * 1000.0) - 2000 + math.sin(nowt) * 5000)
        )
        payload += struct.pack("<L", self.hacc_mm)
        payload += struct.pack("<L", 5000)
        speed_mms = self.speed_kmh * 1000.0 / 3.6
        heading_rad = math.radians(self.heading_deg)
        vel_n = int(math.cos(heading_rad) * speed_mms)
        vel_e = int(math.sin(heading_rad) * speed_mms)
        vel_d = random.randint(-100, 100)
        payload += struct.pack("<lll", vel_n, vel_e, vel_d)
        payload += struct.pack("<l", int(abs(speed_mms)))  # gSpeed
        payload += struct.pack("<l", int(self.heading_deg * 1e5))  # headMot
        payload += struct.pack("<L", self.sacc_mms)  # sAcc
        payload += struct.pack("<L", 1_000_000)  # headAcc
        payload += struct.pack("<H", 120)  # pDOP
        payload += b"\x00" * 6
        payload += struct.pack("<l", int(self.heading_deg * 1e5))  # headVeh
        payload += struct.pack("<hH", 500, 1000)  # magDec, magAcc
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
        payload = struct.pack("<L", int(self.itow_ms))
        payload += struct.pack("<lll", velN_cms, velE_cms, velD_cms)
        payload += struct.pack("<L", speed_cms)
        payload += struct.pack("<L", speed_cms)
        payload += struct.pack("<l", heading)
        payload += struct.pack("<LL", sacc_cms, cacc)
        return to_ubx(0x01, 0x12, payload)

    def build_nav_sol(self) -> bytes:
        now = datetime.now(timezone.utc)
        week, itow_ms = gps_week_and_tow_ms(now)
        x_m, y_m, z_m = geodetic_to_ecef(self.lat, self.lon, self.h_m)
        x_cm, y_cm, z_cm = int(x_m * 100.0), int(y_m * 100.0), int(z_m * 100.0)
        speed_mms = self.speed_kmh * 1000.0 / 3.6
        vn = math.cos(math.radians(self.heading_deg)) * (speed_mms / 1000.0)
        ve = math.sin(math.radians(self.heading_deg)) * (speed_mms / 1000.0)
        vd = 0.0
        vx_mps, vy_mps, vz_mps = ned_to_ecef(self.lat, self.lon, vn, ve, vd)
        vx_cms, vy_cms, vz_cms = (
            int(vx_mps * 100.0),
            int(vy_mps * 100.0),
            int(vz_mps * 100.0),
        )
        pacc_cm = max(1, int(self.hacc_mm / 10.0))
        sacc_cms = max(1, int(self.sacc_mms / 10.0))
        gpsFix = 3 if self.fix_type >= 3 else (2 if self.fix_type == 2 else 0)
        flags = 0x01 if gpsFix >= 2 else 0x00
        payload = struct.pack("<L", itow_ms)
        payload += struct.pack("<l", 0)  # fTOW
        payload += struct.pack("<h", week)
        payload += struct.pack("<B", gpsFix)
        payload += struct.pack("<B", flags)
        payload += struct.pack("<lll", x_cm, y_cm, z_cm)
        payload += struct.pack("<L", pacc_cm)
        payload += struct.pack("<lll", vx_cms, vy_cms, vz_cms)
        payload += struct.pack("<L", sacc_cms)
        payload += struct.pack("<H", 120)  # pDOP
        payload += struct.pack("<B", 0)
        payload += struct.pack("<B", self.sats)
        payload += struct.pack("<L", 0)
        return to_ubx(0x01, 0x06, payload)

    # ---- NMEA builders (time from system clock) ----
    def _utc_hhmmss_frac(self) -> str:
        now = datetime.now(timezone.utc)
        return f"{now:%H%M%S}.{int(now.microsecond / 1000):03d}"

    def _utc_date_ddmmyy(self) -> str:
        now = datetime.now(timezone.utc)
        return f"{now:%d%m%y}"

    def build_nmea_gga(self) -> str:
        t = self._utc_hhmmss_frac()
        lat, ns = deg_to_nmea_lat(self.lat)
        lon, ew = deg_to_nmea_lon(self.lon)
        fixq = "1" if self.fix_type >= 2 else "0"
        nums = f"{max(0, min(12, self.sats)):02d}"
        hdop = "1.2"
        alt = f"{self.h_m:.1f}"
        geoid = "0.0"
        parts = [
            "GNGGA",
            t,
            lat,
            ns,
            lon,
            ew,
            fixq,
            nums,
            hdop,
            alt,
            "M",
            geoid,
            "M",
            "",
            "",
        ]
        core = ",".join(parts)
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_gll(self) -> str:
        t = self._utc_hhmmss_frac()
        lat, ns = deg_to_nmea_lat(self.lat)
        lon, ew = deg_to_nmea_lon(self.lon)
        status = "A" if self.fix_type >= 2 else "V"
        mode = "A" if self.fix_type >= 3 else ("D" if self.fix_type == 2 else "N")
        core = ",".join(["GNGLL", lat, ns, lon, ew, t, status, mode])
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_gsa(self) -> str:
        mode1 = "A"
        mode2 = "3" if self.fix_type >= 3 else ("2" if self.fix_type == 2 else "1")
        prns = [f"{i:02d}" for i in range(1, 13)]
        core = ",".join(["GNGSA", mode1, mode2] + prns + ["1.8", "1.0", "1.5"])
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_gsv(self) -> str:
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
        core = ",".join(["GNGSV", "1", "1", f"{in_view:02d}"] + sats[:16])
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_rmc(self) -> str:
        t = self._utc_hhmmss_frac()
        date = self._utc_date_ddmmyy()
        status = "A" if self.fix_type >= 2 else "V"
        lat, ns = deg_to_nmea_lat(self.lat)
        lon, ew = deg_to_nmea_lon(self.lon)
        spd_kn = f"{knots_from_kmh(self.speed_kmh):.1f}"
        cog = f"{self.heading_deg:.1f}"
        mode = "A" if self.fix_type >= 3 else ("D" if self.fix_type == 2 else "N")
        core = ",".join(
            ["GNRMC", t, status, lat, ns, lon, ew, spd_kn, cog, date, "", "", mode]
        )
        return f"${core}*{nmea_checksum(core)}\r\n"

    def build_nmea_vtg(self) -> str:
        cog_t = f"{self.heading_deg:.1f}"
        spd_kn = f"{knots_from_kmh(self.speed_kmh):.1f}"
        spd_kmh = f"{self.speed_kmh:.1f}"
        mode = "A" if self.fix_type >= 3 else ("D" if self.fix_type == 2 else "N")
        core = ",".join(["GNVTG", cog_t, "T", "", "M", spd_kn, "N", spd_kmh, "K", mode])
        return f"${core}*{nmea_checksum(core)}\r\n"

    # ---- profile helpers (Approach A) ----
    def _profiles_cycle(self) -> List[str]:
        # Only those with duration > 0
        return [
            p
            for p in self.cycle
            if float(self.PROFILES.get(p, {}).get("duration", 0)) > 0.0
        ]

    def _advance_profile_if_needed(self, now: float):
        """Approach A: emit one final 100% progress tick, then switch on next loop."""
        info = self.PROFILES.get(self.current_profile, {"duration": 5})
        dur = max(0.0, float(info.get("duration", 5)))  # 0 handled by filtering cycle

        if dur == 0.0:
            # if ever happens mid-run (shouldn't), skip to next
            cyc = self._profiles_cycle()
            if cyc:
                idx = (
                    cyc.index(self.current_profile)
                    if self.current_profile in cyc
                    else -1
                )
                self.current_profile = cyc[(idx + 1) % len(cyc)] if idx >= 0 else cyc[0]
                self.profile_start = now
                self.profile_seq += 1
                self._sent_final_prog = False
            return

        if now - self.profile_start >= dur:
            if not self._sent_final_prog:
                # Send final tick at 100% for current profile
                is_lost = self.current_profile in ("NO_FIX", "GPS_DISABLE")
                self.profile_cb(
                    {
                        "name": self.current_profile,
                        "progress": 1.0,
                        "is_lost": is_lost,
                        "seq": self.profile_seq,
                    }
                )
                self._sent_final_prog = True
                return  # Defer switch to next loop to ensure UI can render 100%

            # Now actually switch profile (respect user-enabled via duration>0)
            cyc = self._profiles_cycle()
            if cyc:
                idx = (
                    cyc.index(self.current_profile)
                    if self.current_profile in cyc
                    else -1
                )
                self.current_profile = cyc[(idx + 1) % len(cyc)] if idx >= 0 else cyc[0]
            self.profile_start = now
            self.profile_seq += 1
            self._sent_final_prog = False

    def _emit_profile(self, now: float):
        """Continuous profile progress updates (progress=1.0 if final tick already sent)."""
        info = self.PROFILES.get(self.current_profile, {"duration": 5})
        dur = max(0.0, float(info.get("duration", 5)))
        progress = 0.0
        if dur > 0.0:
            progress = (
                1.0
                if self._sent_final_prog
                else min(1.0, max(0.0, (now - self.profile_start) / dur))
            )
        is_lost = self.current_profile in ("NO_FIX", "GPS_DISABLE")
        self.profile_cb(
            {
                "name": self.current_profile,
                "progress": progress,
                "is_lost": is_lost,
                "seq": self.profile_seq,
            }
        )

    # ---- status emit ----
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

    # ---- movement & quality ----
    def _target_speed(self, now: float) -> float:
        p = self.PROFILES.get(self.current_profile, {"min": 0, "max": 0, "duration": 5})
        mn, mx = float(p.get("min", 0)), float(p.get("max", 0))
        if (
            mn < 0
        ):  # NO_FIX/GPS_DISABLE markers (kept for backward compat, not used for enable)
            return mn
        if mn == 0 and mx == 0:
            return 0.0
        el = now - self.profile_start
        dur = max(0.1, float(p.get("duration", 5)))
        progress = min(1.0, el / dur)
        return max(0.0, mn + (mx - mn) * progress + random.uniform(-2.0, 2.0))

    # ---- main loop ----
    def run(self):
        next_send = time.perf_counter() + self.send_interval
        next_rate = time.perf_counter() + 1.0

        try:
            while not self.stop_event.is_set():
                # Handle control messages
                try:
                    while True:
                        cmd = self.ctrl_q.get_nowait()
                        if isinstance(cmd, dict):
                            if cmd.get("cmd") == "reset":
                                new_cfg = cmd.get("cfg", {})
                                if serial is not None and (
                                    new_cfg.get("port") != self.cfg.get("port")
                                    or new_cfg.get("baudrate")
                                    != self.cfg.get("baudrate")
                                ):
                                    self._open_serial(
                                        new_cfg["port"], new_cfg["baudrate"]
                                    )
                                self._apply_config(new_cfg, reset_all=True)
                                now = time.perf_counter()
                                next_send, next_rate = (
                                    now + self.send_interval,
                                    now + 1.0,
                                )

                            # NEW: pause/resume command
                            elif cmd.get("cmd") == "pause":
                                self.paused = bool(cmd.get("value", False))
                except queue.Empty:
                    pass

                if self.paused:
                    time.sleep(0.05)
                    continue
                now = time.perf_counter()
                self._advance_profile_if_needed(now)
                self._emit_profile(now)

                # Quality depending on profile
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
                    self.sats = random.randint(0, 2)
                else:
                    if random.random() < 0.05:
                        self.fix_type = 2
                        self.sats = random.randint(4, 8)
                        self.hacc_mm = random.randint(5000, 15000)
                        self.sacc_mms = random.randint(300, 5000)
                        # self.sacc_mms = 0
                    else:  # 95% of time
                        self.fix_type = 3
                        self.sats = random.randint(8, 16)
                        self.hacc_mm = random.randint(1000, 5000)
                        self.sacc_mms = random.randint(120, 500)
                        # self.sacc_mms = 0
                    if random.random() < 0.002:
                        self.fix_type = 0
                        self.sats = random.randint(0, 2)

                # Motion
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

                # Time-of-week advances with send interval
                self.itow_ms = (
                    self.itow_ms + int(self.send_interval * 1000)
                ) % 604_800_000
                self._emit_stats()

                # Send messages on schedule
                nmea_map: Dict[str, str] = {}
                ubx_map: Dict[str, str] = {}

                if active_send and now >= next_send:
                    # UBX
                    if self.ubx_en.get("pvt"):
                        pkt = self.build_nav_pvt()
                        try:
                            if self.ser:
                                self.ser.write(pkt)
                        except Exception:
                            pass
                        self._sent_counts["pvt"] += 1
                        self._total_counts["pvt"] += 1
                        ubx_map["pvt"] = pkt.hex().upper()
                    if self.ubx_en.get("velned"):
                        pkt = self.build_nav_velned()
                        try:
                            if self.ser:
                                self.ser.write(pkt)
                        except Exception:
                            pass
                        self._sent_counts["velned"] += 1
                        self._total_counts["velned"] += 1
                        ubx_map["velned"] = pkt.hex().upper()
                    if self.ubx_en.get("sol"):
                        pkt = self.build_nav_sol()
                        try:
                            if self.ser:
                                self.ser.write(pkt)
                        except Exception:
                            pass
                        self._sent_counts["sol"] += 1
                        self._total_counts["sol"] += 1
                        ubx_map["sol"] = pkt.hex().upper()

                    # NMEA
                    for key, builder in (
                        ("gga", self.build_nmea_gga),
                        ("gll", self.build_nmea_gll),
                        ("gsa", self.build_nmea_gsa),
                        ("gsv", self.build_nmea_gsv),
                        ("rmc", self.build_nmea_rmc),
                        ("vtg", self.build_nmea_vtg),
                    ):
                        if self.nmea_en.get(key):
                            s = builder()
                            try:
                                if self.ser:
                                    self.ser.write(s.encode("ascii"))
                            except Exception:
                                pass
                            self._sent_counts[key] += 1
                            self._total_counts[key] += 1
                            nmea_map[key] = s.strip()

                    if nmea_map or ubx_map:
                        self.tx_cb(nmea_map, ubx_map, dict(self._total_counts))

                    # Catch up in case of lag
                    skips = max(1, int((now - next_send) // self.send_interval) + 1)
                    next_send += skips * self.send_interval

                # Actual-rate update every 1s
                if now >= next_rate:
                    elapsed = max(1e-6, now - self._last_rate_time)
                    for k in self._sent_counts:
                        self._actual_rates[k] = self._sent_counts[k] / elapsed
                        self._sent_counts[k] = 0
                    self._last_rate_time = now
                    self.rate_cb(dict(self._actual_rates))
                    steps = max(1, int((now - next_rate) // 1.0) + 1)
                    next_rate += steps * 1.0

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
    UND = "______"  # placeholder for NO_FIX / GPS_DISABLE
    DEFAULT_DUR = {
        "STATIONARY": 0,
        "WALKING": 0,
        "CITY_DRIVING": 5,
        "HIGHWAY_DRIVING": 5,
        "HIGH_SPEED": 5,
        "NO_FIX": 0,
        "GPS_DISABLE": 0,
    }

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
        self.last3_font = (
            "Segoe UI",
            14,
            "bold",
        )  # NEW: slightly larger font for "last3"

        # Colors
        self.col_green = "#2e7d32"
        self.col_red = "#c62828"
        self.col_gray = "#9e9e9e"
        self.col_text = "#000000"

        # Queues for UI updates
        self.rate_queue = queue.Queue()
        self.profile_queue = queue.Queue()
        self.stats_queue = queue.Queue()
        self.tx_queue = queue.Queue()
        self.ctrl_queue = queue.Queue()

        # UI label for packet totals (added in Current status)
        # Row/column chosen to avoid overlap with existing labels.
        self.lbl_pkt_totals = None
        # A blank template used on Stop/clear:
        self._totals_blank_text = "Packets total: —"

        # Worker control
        self.sim_thread = None
        self.stop_event = threading.Event()
        self.sim_running = False
        # Pause/Resume state
        self.paused = False
        self._pause_label_cache = ("Pause", "Resume")

        # Dirty tracking (for "Apply to running")
        self._last_applied_config = None
        self._dirty = False

        # Profile direct data update (no animator)
        self._current_profile_name = "—"
        self._current_is_lost = False
        self._last_profile_seq = None  # detects profile switch to reset bar

        # ==== Top: Port & Baud ====
        top = ttk.Frame(root, padding=8)
        top.grid(row=0, column=0, sticky="ew")
        for c, w in enumerate([60, 260, 90, 80, 240]):
            top.grid_columnconfigure(c, minsize=w)

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w", padx=(0, 6))
        self.port_var = tk.StringVar(value="")
        self.port_cb = ttk.Combobox(
            top, textvariable=self.port_var, width=20, state="readonly"
        )
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
            state="disabled",
            values=[
                "9600",
                "19200",
                "38400",
                "57600",
                "115200",
                "230400",
                "460800",
                "921600",
            ],
        )
        self.baud_cb.grid(row=0, column=4, sticky="w", padx=(6, 0))
        self.baud_cb.bind("<<ComboboxSelected>>", self._on_any_setting_changed)

        # ==== Message settings (Update rate + UBX/NMEA masters & children in 3 columns) ====
        msg = ttk.LabelFrame(root, text="Message settings", padding=8)
        msg.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        for c in range(6):
            msg.grid_columnconfigure(c, minsize=180)

        ttk.Label(msg, text="Update rate (Hz):").grid(row=0, column=0, sticky="w")
        self.hz_var = tk.StringVar(value="10.0")
        self.hz_entry = ttk.Entry(msg, textvariable=self.hz_var, width=10)
        self.hz_entry.grid(row=0, column=1, sticky="w", padx=(6, 18))
        self.hz_entry.bind("<KeyRelease>", self._on_any_setting_changed)

        # -- UBX master + 3-column children --
        self.ubx_master_var = tk.BooleanVar(value=True)
        self.ubx_group = ttk.Frame(msg)
        self.ubx_group.grid(row=1, column=0, columnspan=6, sticky="ew", pady=(6, 4))
        for c in range(6):
            self.ubx_group.grid_columnconfigure(c, minsize=180)
        self.ubx_master_chk = ttk.Checkbutton(
            self.ubx_group,
            text="Enable UBX messages",
            variable=self.ubx_master_var,
            command=self._on_master_toggled,
        )
        self.ubx_master_chk.grid(row=0, column=0, sticky="w")
        ttk.Separator(self.ubx_group, orient="horizontal").grid(
            row=1, column=0, columnspan=6, sticky="ew", pady=(4, 6)
        )

        self.pvt_var = tk.BooleanVar(value=True)
        self.vel_var = tk.BooleanVar(value=False)
        self.sol_var = tk.BooleanVar(value=False)

        def add_ubx_cell(
            parent, row: int, col_pair: int, text: str, var: tk.BooleanVar
        ):
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
        self.vel_rate = add_ubx_cell(
            self.ubx_group, 2, 1, "UBX-NAV-VELNED", self.vel_var
        )
        self.sol_rate = add_ubx_cell(self.ubx_group, 2, 2, "UBX-NAV-SOL", self.sol_var)

        # Separator between UBX & NMEA children
        ttk.Separator(msg, orient="horizontal").grid(
            row=2, column=0, columnspan=6, sticky="ew", pady=(6, 6)
        )

        # -- NMEA master + 3-column children --
        self.nmea_master_var = tk.BooleanVar(value=False)
        self.nmea_group = ttk.Frame(msg)
        self.nmea_group.grid(row=3, column=0, columnspan=6, sticky="ew", pady=(0, 2))
        for c in range(6):
            self.nmea_group.grid_columnconfigure(c, minsize=180)
        self.nmea_master_chk = ttk.Checkbutton(
            self.nmea_group,
            text="Enable NMEA messages",
            variable=self.nmea_master_var,
            command=self._on_master_toggled,
        )
        self.nmea_master_chk.grid(row=0, column=0, sticky="w")
        ttk.Separator(self.nmea_group, orient="horizontal").grid(
            row=1, column=0, columnspan=6, sticky="ew", pady=(4, 6)
        )

        self.nmea_vars: Dict[str, tk.BooleanVar] = {}
        self.nmea_rate_labels: Dict[str, ttk.Label] = {}

        def add_nmea_cell(parent, row: int, col_pair: int, key: str, label: str):
            var = tk.BooleanVar(value=False)

            def on_child():
                if not self.nmea_master_var.get():
                    self.nmea_master_var.set(True)
                self._on_any_setting_changed()

            chk = ttk.Checkbutton(parent, text=label, variable=var, command=on_child)
            lbl = ttk.Label(parent, text="", font=self.mono_font, width=16)
            c = col_pair * 2
            chk.grid(row=row, column=c, sticky="w")
            lbl.grid(row=row, column=c + 1, sticky="w")
            self.nmea_vars[key] = var
            self.nmea_rate_labels[key] = lbl

        add_nmea_cell(self.nmea_group, 2, 0, "gga", "NMEA GGA")
        add_nmea_cell(self.nmea_group, 2, 1, "gll", "NMEA GLL")
        add_nmea_cell(self.nmea_group, 2, 2, "gsa", "NMEA GSA")
        add_nmea_cell(self.nmea_group, 3, 0, "gsv", "NMEA GSV")
        add_nmea_cell(self.nmea_group, 3, 1, "rmc", "NMEA RMC")
        add_nmea_cell(self.nmea_group, 3, 2, "vtg", "NMEA VTG")

        # ==== Profile settings (table) + Active profile (with progress bar) ====
        prof_set = ttk.LabelFrame(root, text="Profile settings", padding=8)
        prof_set.grid(row=2, column=0, sticky="ew", padx=8, pady=(0, 8))
        headers = ["Enable", "Profile", "Min (km/h)", "Max (km/h)", "Duration (s)"]
        for i, h in enumerate(headers):
            ttk.Label(prof_set, text=h).grid(row=0, column=i, sticky="w", padx=(0, 8))
        for c, w in enumerate([80, 220, 140, 140, 160]):
            prof_set.grid_columnconfigure(c, minsize=w)

        self.profile_rows = {}

        def parse_int_str(s: str) -> int:
            if s is None:
                return 0
            s = str(s).strip().replace(",", ".")
            try:
                # Only integers are allowed; clamp at 0
                v = int(float(s))
            except ValueError:
                v = 0
            return max(0, v)

        def add_profile_row(r, name, mn, mx, dur, lock_minmax=False):
            en = tk.BooleanVar(value=(dur > 0))
            mn_v, mx_v = tk.StringVar(value=str(mn)), tk.StringVar(value=str(mx))
            dur_v = tk.StringVar(value=str(dur))

            def on_en_toggled():
                # master checkbox mirrors duration: tick -> set default if 0; untick -> set 0
                cur = parse_int_str(dur_v.get())
                if en.get():
                    if cur == 0:
                        dur_v.set(str(self.DEFAULT_DUR.get(name, 5)))
                else:
                    dur_v.set("0")
                self._on_any_setting_changed()

            def on_duration_changed(*_):
                v = parse_int_str(dur_v.get())
                en.set(v > 0)  # checkbox mirrors duration
                self._on_any_setting_changed()

            ttk.Checkbutton(prof_set, variable=en, command=on_en_toggled).grid(
                row=r, column=0, sticky="w"
            )
            ttk.Label(prof_set, text=name).grid(row=r, column=1, sticky="w")

            e1 = ttk.Entry(
                prof_set,
                textvariable=mn_v,
                width=12,
                state=("disabled" if lock_minmax else "normal"),
            )
            e2 = ttk.Entry(
                prof_set,
                textvariable=mx_v,
                width=12,
                state=("disabled" if lock_minmax else "normal"),
            )
            e1.grid(row=r, column=2, sticky="w")
            e2.grid(row=r, column=3, sticky="w")
            if not lock_minmax:
                e1.bind("<KeyRelease>", self._on_any_setting_changed)
                e2.bind("<KeyRelease>", self._on_any_setting_changed)

            # Spinbox for duration (integer seconds, min 0)
            sb = tk.Spinbox(
                prof_set,
                from_=0,
                to=86400,
                increment=1,
                width=8,
                textvariable=dur_v,
                command=on_duration_changed,
                justify="left",
            )
            sb.grid(row=r, column=4, sticky="w")
            sb.bind("<KeyRelease>", on_duration_changed)

            self.profile_rows[name] = (en, mn_v, mx_v, dur_v, sb)

        rows = [
            ("STATIONARY", 0, 0, self.DEFAULT_DUR["STATIONARY"], True),
            ("WALKING", 1, 8, self.DEFAULT_DUR["WALKING"], False),
            ("CITY_DRIVING", 10, 60, self.DEFAULT_DUR["CITY_DRIVING"], False),
            ("HIGHWAY_DRIVING", 60, 100, self.DEFAULT_DUR["HIGHWAY_DRIVING"], False),
            ("HIGH_SPEED", 100, 399, self.DEFAULT_DUR["HIGH_SPEED"], False),
            ("NO_FIX", -1, -1, self.DEFAULT_DUR["NO_FIX"], True),
            ("GPS_DISABLE", -2, -2, self.DEFAULT_DUR["GPS_DISABLE"], True),
        ]
        for i, (n, mn, mx, du, lock) in enumerate(rows, start=1):
            add_profile_row(i, n, mn, mx, du, lock_minmax=lock)

        # Active profile label + progress (data-driven)
        style = ttk.Style()
        style.configure("Profile.Normal.Horizontal.TProgressbar", troughcolor="#f0f0f0")
        style.configure(
            "Profile.Lost.Horizontal.TProgressbar",
            troughcolor="#f0f0f0",
            background="#9e9e9e",
        )

        base_row = len(rows) + 2
        self.active_prof_label = ttk.Label(
            prof_set,
            text="Active profile:",
            font=self.active_prof_font,
            width=16,
            anchor="w",
        )
        self.active_prof_value = ttk.Label(
            prof_set, text="—", font=self.active_prof_font, width=22, anchor="w"
        )
        self.active_prog = ttk.Progressbar(
            prof_set,
            orient="horizontal",
            mode="determinate",
            maximum=1000,
            value=0,
            style="Profile.Normal.Horizontal.TProgressbar",
        )
        self.active_prof_label.grid(row=base_row, column=0, sticky="w", pady=(6, 0))
        self.active_prof_value.grid(row=base_row, column=1, sticky="w", pady=(6, 0))
        self.active_prog.grid(
            row=base_row, column=2, columnspan=3, sticky="ew", pady=(6, 0)
        )

        # ==== Calibration (speed offset) ====    <-- ADD THIS BLOCK
        cal = ttk.LabelFrame(root, text="Speed calibration", padding=8)
        cal.grid(row=3, column=0, sticky="ew", padx=8, pady=(0, 8))
        for c, w in enumerate([220, 180, 220, 180, 220, 180]):
            cal.grid_columnconfigure(c, minsize=w)

        ttk.Label(cal, text="speedCalibrationOffset (km/h):").grid(
            row=0, column=0, sticky="w"
        )
        self.calib_var = tk.StringVar(value="0")  # raw offset 0..5

        def _on_calib_changed(*_):
            # mark dirty? not needed for worker; just refresh effective display
            # <-- ADD: mark dirty so Apply to running is enabled
            self._mark_dirty(True)  # enable Apply when sim is running
            try:
                # force refresh of effective label using last-known speed
                kmh = getattr(self, "_last_speed_kmh", None)
                raw = self._parse_int_str(self.calib_var.get())
                eff = (
                    self._effective_offset_from_kmh(kmh, raw)
                    if kmh is not None
                    else raw
                )
                self.lbl_eff_offset.config(text=f"Effective offset: {eff} kmh")
            except Exception:
                pass

        self.calib_spin = tk.Spinbox(
            cal,
            from_=0,
            to=5,
            increment=1,
            width=6,
            textvariable=self.calib_var,
            command=_on_calib_changed,
            justify="left",
        )
        self.calib_spin.grid(row=0, column=1, sticky="w")
        self.calib_spin.bind("<KeyRelease>", _on_calib_changed)

        # Effective offset (auto-updated per packet)
        self.lbl_eff_offset = ttk.Label(
            cal, text="Effective offset: 0 kmh", font=self.mono_font
        )
        self.lbl_eff_offset.grid(row=0, column=2, sticky="w")

        # ==== Current status ====
        stat = ttk.LabelFrame(root, text="Current status", padding=8)
        stat.grid(row=4, column=0, sticky="ew", padx=8, pady=(0, 8))  # was row=3
        for i in range(6):
            stat.grid_columnconfigure(i, minsize=190)

        self.lbl_speed_big = ttk.Label(stat, text="Speed: —", font=self.big_speed_font)
        self.lbl_speed_big.grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 6)
        )  # was 6 -> now 3

        def mono_label(parent, txt, col, row):
            lbl = ttk.Label(parent, text=txt, font=self.mono_font, anchor="w")
            lbl.grid(row=row, column=col, sticky="w")
            return lbl

        self.lbl_time_utc = mono_label(stat, "Time (UTC): —", 0, 1)
        self.lbl_time_loc = mono_label(stat, "Time (Local): —", 1, 1)
        self.lbl_heading = mono_label(stat, "Heading: —°", 2, 1)
        self.lbl_sats = mono_label(stat, "Sats: —", 3, 1)
        self.lbl_fix_status = ttk.Label(
            stat, text="FIX: —", font=("Segoe UI", 11, "bold"), foreground=self.col_gray
        )
        self.lbl_fix_status.grid(row=1, column=4, sticky="w")
        self.lbl_lat = mono_label(stat, "Lat: —", 0, 2)
        self.lbl_lon = mono_label(stat, "Lon: —", 1, 2)

        # NEW: Packets total line (separate row)
        self.lbl_pkt_totals = ttk.Label(
            stat, text=self._totals_blank_text, font=self.mono_font, anchor="w"
        )
        # đặt riêng 1 dòng cuối cùng (ví dụ row=10 cho chắc, chiếm hết 6 cột)
        self.lbl_pkt_totals.grid(
            row=10, column=0, columnspan=6, sticky="w", pady=(4, 0)
        )

        # ==== Buttons ====
        btns = ttk.Frame(root, padding=(8, 0, 8, 8))
        btns.grid(row=5, column=0, sticky="ew")  # was row=4
        for c, w in enumerate([160, 160, 160, 360]):
            btns.grid_columnconfigure(c, minsize=w)
        self.start_btn = ttk.Button(
            btns, text="Start", command=self.start_sim, state="disabled"
        )
        self.pause_btn = ttk.Button(
            btns, text="Pause", command=self.toggle_pause, state="disabled"
        )
        self.stop_btn = ttk.Button(
            btns, text="Stop", command=self.stop_sim, state="disabled"
        )
        self.apply_btn = ttk.Button(
            btns,
            text="Apply to running (reset)",
            command=self.apply_to_running,
            state="disabled",
        )
        self.start_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.pause_btn.grid(row=0, column=1, sticky="ew", padx=(4, 4))
        self.stop_btn.grid(row=0, column=2, sticky="ew", padx=(4, 4))
        self.apply_btn.grid(row=0, column=3, sticky="ew", padx=(4, 0))

        # ==== TX Monitor ====
        monitor = ttk.LabelFrame(root, text="TX Monitor (latest)", padding=8)
        monitor.grid(row=6, column=0, sticky="ew", padx=8, pady=(0, 8))  # was row=5
        monitor.grid_columnconfigure(0, weight=1)

        # UBX
        ttk.Label(monitor, text="UBX (hex):").grid(row=0, column=0, sticky="w")
        ubx_frame = ttk.Frame(monitor)
        ubx_frame.grid(row=1, column=0, sticky="ew")
        ubx_frame.grid_columnconfigure(0, weight=1)
        self.ubx_tree = ttk.Treeview(
            ubx_frame, columns=("msg", "len", "hex"), show="headings", height=1
        )
        self.ubx_tree.heading("msg", text="Message")
        self.ubx_tree.heading("len", text="Bytes")
        self.ubx_tree.heading("hex", text="Hex (no spaces)")
        self.ubx_tree.column("msg", width=160, anchor="w", stretch=False)
        self.ubx_tree.column("len", width=70, anchor="center", stretch=False)
        self.ubx_tree.column("hex", width=800, anchor="w", stretch=True)
        self.ubx_tree.grid(row=0, column=0, sticky="ew")
        ubx_vsb = ttk.Scrollbar(
            ubx_frame, orient="vertical", command=self.ubx_tree.yview
        )
        ubx_hsb = ttk.Scrollbar(
            ubx_frame, orient="horizontal", command=self.ubx_tree.xview
        )
        self.ubx_tree.configure(yscroll=ubx_vsb.set, xscroll=ubx_hsb.set)
        ubx_vsb.grid(row=0, column=1, sticky="ns")
        ubx_hsb.grid(row=1, column=0, sticky="ew")

        ttk.Separator(monitor, orient="horizontal").grid(
            row=2, column=0, sticky="ew", pady=(4, 8)
        )

        # NMEA
        ttk.Label(monitor, text="NMEA (text):").grid(row=3, column=0, sticky="w")
        nmea_frame = ttk.Frame(monitor)
        nmea_frame.grid(row=4, column=0, sticky="ew")
        nmea_frame.grid_columnconfigure(0, weight=1)
        self.nmea_tree = ttk.Treeview(
            nmea_frame, columns=("msg", "text"), show="headings", height=1
        )
        self.nmea_tree.heading("msg", text="Message")
        self.nmea_tree.heading("text", text="Sentence")
        self.nmea_tree.column("msg", width=160, anchor="w", stretch=False)
        self.nmea_tree.column("text", width=970, anchor="w", stretch=True)
        self.nmea_tree.grid(row=0, column=0, sticky="ew")
        nmea_vsb = ttk.Scrollbar(
            nmea_frame, orient="vertical", command=self.nmea_tree.yview
        )
        nmea_hsb = ttk.Scrollbar(
            nmea_frame, orient="horizontal", command=self.nmea_tree.xview
        )
        self.nmea_tree.configure(yscroll=nmea_vsb.set, xscroll=nmea_hsb.set)
        nmea_vsb.grid(row=0, column=1, sticky="ns")
        nmea_hsb.grid(row=1, column=0, sticky="ew")

        # Context menus (copy)
        self._build_context_menus()

        # Init ports (do not auto-select)
        self.refresh_ports()
        self._update_start_btn_state()
        self._rebuild_tx_tables()
        # Queue drainers
        self.root.after(120, self._drain_rate_queue)
        self.root.after(
            30, self._drain_profile_queue
        )  # 30ms is fine for direct updates
        self.root.after(120, self._drain_stats_queue)
        self.root.after(80, self._drain_tx_queue)

        # Ensure treeviews render once on start
        self.root.after(50, self._force_refresh_treeviews)

    # ---------- Context menus ----------
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
        self.nmea_menu.add_command(
            label="Copy Payload", command=self._copy_selected_nmea
        )
        self.nmea_menu.add_command(
            label="Copy Row", command=self._copy_selected_nmea_row
        )
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

    # ---------- Copy helpers ----------
    def _parse_int_str(self, s: str) -> int:
        s = "" if s is None else str(s).strip().replace(",", ".")
        try:
            v = int(float(s))
        except ValueError:
            v = 0
        return max(0, v)

    # --- Effective offset logic (mirror main.cpp getEffectiveCalibrationOffset) ---
    def _effective_offset_from_kmh(self, kmh: float, raw_offset: int) -> int:
        """
        Piecewise calibration:
        <20 km/h  -> raw - 3
        20..40    -> raw - 2
        40..60    -> raw - 1
        >=60      -> raw
        Clamp [0..5]
        """
        if kmh is None:
            return max(0, min(5, raw_offset))
        eff = raw_offset
        if kmh < 20.0:
            eff = raw_offset - 3
        elif kmh < 40.0:
            eff = raw_offset - 2
        elif kmh < 60.0:
            eff = raw_offset - 1
        # else: raw
        if eff < 0:
            eff = 0
        if eff > 5:
            eff = 5
        return eff

    def _copy_selected_ubx(self):
        sel = self.ubx_tree.selection()
        if not sel:
            return
        payload = self.ubx_tree.item(sel[0], "values")[2]
        self._to_clipboard(payload)

    def _copy_selected_ubx_row(self):
        sel = self.ubx_tree.selection()
        if not sel:
            return
        vals = self.ubx_tree.item(sel[0], "values")
        self._to_clipboard(",".join(vals))

    def _copy_all_ubx(self):
        lines = [
            ",".join(self.ubx_tree.item(iid, "values"))
            for iid in self.ubx_tree.get_children()
        ]
        self._to_clipboard("\n".join(lines))

    def _copy_selected_nmea(self):
        sel = self.nmea_tree.selection()
        if not sel:
            return
        payload = self.nmea_tree.item(sel[0], "values")[1]
        self._to_clipboard(payload)

    def _copy_selected_nmea_row(self):
        sel = self.nmea_tree.selection()
        if not sel:
            return
        vals = self.nmea_tree.item(sel[0], "values")
        self._to_clipboard(",".join(vals))

    def _copy_all_nmea(self):
        lines = [
            ",".join(self.nmea_tree.item(iid, "values"))
            for iid in self.nmea_tree.get_children()
        ]
        self._to_clipboard("\n".join(lines))

    def _to_clipboard(self, text: str):
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
        except Exception:
            pass

    # ---------- Tree utilities ----------
    def _force_refresh_treeviews(self):
        self.ubx_tree.update_idletasks()
        self.nmea_tree.update_idletasks()

    def _set_tx_heights(self):
        ubx_count = sum(
            [
                self.ubx_master_var.get() and self.pvt_var.get(),
                self.ubx_master_var.get() and self.vel_var.get(),
                self.ubx_master_var.get() and self.sol_var.get(),
            ]
        )
        nmea_count = sum(
            1 for v in self.nmea_vars.values() if self.nmea_master_var.get() and v.get()
        )
        self.ubx_tree.configure(height=max(1, ubx_count))
        self.nmea_tree.configure(height=max(1, nmea_count))

    def _rebuild_tx_tables(self):
        # Clear & rebuild rows to exactly match enabled messages
        for tree in (self.ubx_tree, self.nmea_tree):
            for iid in tree.get_children():
                tree.delete(iid)

        if self.ubx_master_var.get():
            if self.pvt_var.get():
                self.ubx_tree.insert(
                    "", "end", iid="pvt", values=("UBX-NAV-PVT", "", "")
                )
            if self.vel_var.get():
                self.ubx_tree.insert(
                    "", "end", iid="velned", values=("UBX-NAV-VELNED", "", "")
                )
            if self.sol_var.get():
                self.ubx_tree.insert(
                    "", "end", iid="sol", values=("UBX-NAV-SOL", "", "")
                )

        if self.nmea_master_var.get():
            for key, label in (
                ("gga", "NMEA GGA"),
                ("gll", "NMEA GLL"),
                ("gsa", "NMEA GSA"),
                ("gsv", "NMEA GSV"),
                ("rmc", "NMEA RMC"),
                ("vtg", "NMEA VTG"),
            ):
                if self.nmea_vars.get(key, tk.BooleanVar()).get():
                    self.nmea_tree.insert("", "end", iid=key, values=(label, ""))

        self._set_tx_heights()
        self._force_refresh_treeviews()
        self._auto_resize_to_fit()

    def _clear_status_and_tx(self):
        # Reset colors
        self._set_status_area_color(self.col_text, include_fix=True)
        self._last_profile_seq = None
        self.active_prog["value"] = 0

        # Status text reset
        self.lbl_speed_big.config(text="Speed: —")
        self.lbl_time_utc.config(text="Time (UTC): —")
        self.lbl_time_loc.config(text="Time (Local): —")
        self.lbl_heading.config(text="Heading: —°")
        self.lbl_sats.config(text="Sats: —")
        self._set_fix_text("FIX: —", self.col_gray)
        self.lbl_lat.config(text="Lat: —")
        self.lbl_lon.config(text="Lon: —")
        self.active_prof_value.config(text="—")
        if self.lbl_pkt_totals is not None:
            self.lbl_pkt_totals.config(text=self._totals_blank_text)

        # Clear current TX values
        for iid in self.ubx_tree.get_children():
            self.ubx_tree.item(
                iid, values=(self.ubx_tree.item(iid, "values")[0], "", "")
            )
        for iid in self.nmea_tree.get_children():
            self.nmea_tree.item(iid, values=(self.nmea_tree.item(iid, "values")[0], ""))

    # ---------- Status color helpers ----------
    def _set_status_area_color(self, color: str, include_fix: bool = False):
        labels = [
            self.lbl_speed_big,
            self.lbl_time_utc,
            self.lbl_time_loc,
            self.lbl_heading,
            self.lbl_sats,
            self.lbl_lat,
            self.lbl_lon,
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

    # ---------- One-shot auto-resize (no jitter) ----------
    def _auto_resize_to_fit(self):
        try:
            ubx_rows = max(
                1,
                sum(
                    [
                        self.ubx_master_var.get() and self.pvt_var.get(),
                        self.ubx_master_var.get() and self.vel_var.get(),
                        self.ubx_master_var.get() and self.sol_var.get(),
                    ]
                ),
            )
            nmea_rows = max(
                1,
                sum(
                    1
                    for v in self.nmea_vars.values()
                    if self.nmea_master_var.get() and v.get()
                ),
            )
            per_row = 22
            base = 820  # base layout height without extra TX rows
            desired = base + (ubx_rows - 1) * per_row + (nmea_rows - 1) * per_row
            screen_h = self.root.winfo_screenheight()
            desired = min(desired, screen_h - 80)
            cur_w = self.root.winfo_width() or 1180
            self.root.geometry(f"{cur_w}x{int(max(self.root.winfo_height(), desired))}")
        except Exception:
            pass

    # ---------- Dirty & start-enable logic ----------
    def _mark_dirty(self, dirty=True):
        self._dirty = bool(dirty)
        self.apply_btn.configure(
            state=("normal" if (self.sim_running and self._dirty) else "disabled")
        )

    def _on_any_setting_changed(self, *_):
        self._mark_dirty(True)
        self._update_start_btn_state()
        self._rebuild_tx_tables()

    def _on_master_toggled(self):
        # Untick children if master unticked. Child tick re-ticks master (handled in child handlers).
        if not self.ubx_master_var.get():
            self.pvt_var.set(False)
            self.vel_var.set(False)
            self.sol_var.set(False)
        if not self.nmea_master_var.get():
            for v in self.nmea_vars.values():
                v.set(False)
        self._on_any_setting_changed()

    def _on_port_selected(self, *_):
        self.baud_cb.configure(
            state=("readonly" if bool(self.port_var.get().strip()) else "disabled")
        )
        self._on_any_setting_changed()

    def _has_any_message_selected(self) -> bool:
        any_ubx = self.ubx_master_var.get() and (
            self.pvt_var.get() or self.vel_var.get() or self.sol_var.get()
        )
        any_nmea = self.nmea_master_var.get() and any(
            v.get() for v in self.nmea_vars.values()
        )
        return any_ubx or any_nmea

    def _update_start_btn_state(self):
        if self.sim_running:
            self.start_btn.configure(state="disabled")
            return
        cfg_ok = self._current_config(validate_messages=False) is not None
        self.start_btn.configure(
            state=(
                "normal"
                if (cfg_ok and self._has_any_message_selected())
                else "disabled"
            )
        )

    # ---------- Config build ----------
    def refresh_ports(self):
        if serial is None:
            self.port_cb["values"] = []
            self.port_var.set("")
            self.baud_cb.configure(state="disabled")
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
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
            hz = float(self.hz_var.get().replace(",", "."))
            if hz <= 0:
                raise ValueError
        except ValueError:
            return None

        ubx_enable = {
            "pvt": bool(self.pvt_var.get()) if self.ubx_master_var.get() else False,
            "velned": bool(self.vel_var.get()) if self.ubx_master_var.get() else False,
            "sol": bool(self.sol_var.get()) if self.ubx_master_var.get() else False,
        }
        nmea_enable = {
            k: (bool(v.get()) if self.nmea_master_var.get() else False)
            for k, v in self.nmea_vars.items()
        }

        if (
            validate_messages
            and not any(ubx_enable.values())
            and not any(nmea_enable.values())
        ):
            return None

        profiles = {}
        for name, (en_v, mn_v, mx_v, du_v, _sb) in self.profile_rows.items():
            try:
                dur = max(0, int(float(du_v.get().replace(",", "."))))
                # Enabled derived from duration > 0 (checkbox mirrors this already)
                profiles[name] = {
                    "min": float(mn_v.get().replace(",", ".")),
                    "max": float(mx_v.get().replace(",", ".")),
                    "duration": dur,  # integer seconds
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

    # ---------- Queue drainers ----------
    def _drain_rate_queue(self):
        try:
            while True:
                rates = self.rate_queue.get_nowait()
                if self.sim_running:
                    # UBX
                    self.pvt_rate.config(
                        text=(
                            f"act: {rates.get('pvt', 0.0):.2f} Hz"
                            if self.pvt_var.get() and self.ubx_master_var.get()
                            else ""
                        )
                    )
                    self.vel_rate.config(
                        text=(
                            f"act: {rates.get('velned', 0.0):.2f} Hz"
                            if self.vel_var.get() and self.ubx_master_var.get()
                            else ""
                        )
                    )
                    self.sol_rate.config(
                        text=(
                            f"act: {rates.get('sol', 0.0):.2f} Hz"
                            if self.sol_var.get() and self.ubx_master_var.get()
                            else ""
                        )
                    )
                    # NMEA
                    for key in ("gga", "gll", "gsa", "gsv", "rmc", "vtg"):
                        lbl, var = (
                            self.nmea_rate_labels.get(key),
                            self.nmea_vars.get(key),
                        )
                        if lbl is not None and var is not None:
                            lbl.config(
                                text=(
                                    f"act: {rates.get(key, 0.0):.2f} Hz"
                                    if var.get() and self.nmea_master_var.get()
                                    else ""
                                )
                            )
        except queue.Empty:
            pass
        if self.paused:
            pass
        self.root.after(120, self._drain_rate_queue)

    def _drain_profile_queue(self):
        # Keep only the newest profile item each tick
        info = None
        try:
            while True:
                info = self.profile_queue.get_nowait()
        except queue.Empty:
            pass
        if info is not None and self.sim_running:
            # Detect profile switch via seq -> reset bar to 0
            seq = info.get("seq")
            if seq is not None and seq != self._last_profile_seq:
                self._last_profile_seq = seq
                self.active_prog["value"] = 0

            # Update active profile label and set progress directly
            self._current_is_lost = bool(info.get("is_lost", False))
            self._current_profile_name = info.get("name", "—")
            self.active_prof_value.config(text=self._current_profile_name)

            self.active_prog.configure(
                style=(
                    "Profile.Lost.Horizontal.TProgressbar"
                    if self._current_is_lost
                    else "Profile.Normal.Horizontal.TProgressbar"
                )
            )

            p = info.get("progress", 0.0)
            try:
                p = float(p)
            except Exception:
                p = 0.0
            p = 0.0 if p < 0.0 else (1.0 if p > 1.0 else p)
            self.active_prog["value"] = int(p * 1000)

        self.root.after(30, self._drain_profile_queue)

    def _set_fix_text(self, text: str, color: str):
        self.lbl_fix_status.configure(text=text, foreground=color)

    def _drain_stats_queue(self):
        try:
            while True:
                s = self.stats_queue.get_nowait()
                if not self.sim_running:
                    continue
                prof = self._current_profile_name
                if prof == "GPS_DISABLE":
                    und = self.UND
                    for lbl, txt in (
                        (self.lbl_speed_big, f"Speed: {und}"),
                        (self.lbl_time_utc, f"Time (UTC): {und}"),
                        (self.lbl_time_loc, f"Time (Local): {und}"),
                        (self.lbl_heading, f"Heading: {und}"),
                        (self.lbl_sats, f"Sats: {und}"),
                        (self.lbl_lat, f"Lat: {und}"),
                        (self.lbl_lon, f"Lon: {und}"),
                    ):
                        lbl.config(text=txt)
                    self._set_fix_text("FIX: GPS DISABLED", self.col_gray)
                    self._set_status_area_color(self.col_gray, include_fix=True)
                elif prof == "NO_FIX":
                    und = self.UND
                    for lbl, txt in (
                        (self.lbl_speed_big, f"Speed: {und}"),
                        (self.lbl_time_utc, f"Time (UTC): {und}"),
                        (self.lbl_time_loc, f"Time (Local): {und}"),
                        (self.lbl_heading, f"Heading: {und}"),
                        (self.lbl_sats, f"Sats: {und}"),
                        (self.lbl_lat, f"Lat: {und}"),
                        (self.lbl_lon, f"Lon: {und}"),
                    ):
                        lbl.config(text=txt)
                    self._reset_status_area_basecolor()
                    self._set_fix_text("FIX: NO FIX", self.col_red)
                else:
                    self._reset_status_area_basecolor()
                    fx = int(s.get("fix", 0))
                    self.lbl_time_utc.config(text=f"Time (UTC): {s.get('utc', '—')}")
                    self.lbl_time_loc.config(
                        text=f"Time (Local): {s.get('local', '—')}"
                    )
                    self.lbl_heading.config(
                        text=f"Heading: {s.get('heading', 0.0):.1f}°"
                    )
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

    def _parse_nav_pvt_kmh(self, ubx_hex: str):
        """Return speed in km/h parsed from UBX-NAV-PVT (current packet only)."""
        try:
            b = bytes.fromhex(ubx_hex)
            if len(b) < 8:  # header + len
                return None
            if b[0] != 0xB5 or b[1] != 0x62:
                return None
            if b[2] != 0x01 or b[3] != 0x07:  # NAV-PVT
                return None
            plen = int.from_bytes(b[4:6], "little")
            if len(b) < 6 + plen + 2:
                return None
            payload = b[6 : 6 + plen]
            # Simulator's builder order: gSpeed at payload offset 60 (int32, mm/s)
            if len(payload) < 64:
                return None
            gspeed_mms = int.from_bytes(payload[60:64], "little", signed=True)
            kmh = max(0.0, (gspeed_mms / 1000.0) * 3.6)
            return kmh
        except Exception:
            return None

    def _parse_vtg_kmh(self, sentence: str):
        """Return speed in km/h from NMEA VTG (..,K field)."""
        try:
            s = sentence.strip()
            if not s.startswith("$") or "*" not in s:
                return None
            core = s[1 : s.index("*")]
            parts = core.split(",")
            if not parts or parts[0][-3:].upper() != "VTG":
                return None
            # VTG: ... , <spd_knots>, 'N', <spd_kmh>, 'K', mode
            # find the 'K' token and read the number just before it
            for i, tok in enumerate(parts):
                if tok.upper() == "K" and i > 0:
                    return float(parts[i - 1])
            return None
        except Exception:
            return None

    def _parse_rmc_kmh(self, sentence: str):
        """Return speed in km/h from NMEA RMC (speed over ground in knots -> km/h)."""
        try:
            s = sentence.strip()
            if not s.startswith("$") or "*" not in s:
                return None
            core = s[1 : s.index("*")]
            parts = core.split(",")
            if not parts or parts[0][-3:].upper() != "RMC":
                return None
            # RMC fields: ... , spd_knots (index 7), ...
            if len(parts) > 7 and parts[7]:
                knots = float(parts[7])
                return knots * 1.852  # 1 knot = 1.852 km/h
            return None
        except Exception:
            return None

    def _drain_tx_queue(self):
        try:
            while True:
                nmea_map, ubx_map, totals = self.tx_queue.get_nowait()
                if not self.sim_running:
                    continue
                # UBX (hex + computed byte length)
                for key, label in (
                    ("pvt", "UBX-NAV-PVT"),
                    ("velned", "UBX-NAV-VELNED"),
                    ("sol", "UBX-NAV-SOL"),
                ):
                    if key in ubx_map and key in self.ubx_tree.get_children():
                        hexstr = ubx_map[key]
                        bytelen = len(hexstr) // 2
                        self.ubx_tree.item(key, values=(label, str(bytelen), hexstr))

                        if key == "pvt":
                            kmh = self._parse_nav_pvt_kmh(hexstr)
                            if kmh is not None and self._current_profile_name not in (
                                "NO_FIX",
                                "GPS_DISABLE",
                            ):
                                # Update Speed & last3 immediately (current packet)
                                # When got kmh from PVT/VTG/RMC:
                                self._last_speed_kmh = float(
                                    kmh
                                )  # cache last real speed
                                raw = self._parse_int_str(self.calib_var.get())
                                eff = self._effective_offset_from_kmh(
                                    self._last_speed_kmh, raw
                                )
                                disp = max(0.0, self._last_speed_kmh + eff)
                                self.lbl_speed_big.config(
                                    text=f"Speed: {self._last_speed_kmh:.1f} kmh.    Display speed {disp:.1f} kmh"
                                )

                                # Update Effective offset label (this is the missing part)
                                self.lbl_eff_offset.config(
                                    text=f"Effective offset: {eff} kmh"
                                )
                # Update totals line every time we get a TX tick
                if isinstance(totals, dict) and self.lbl_pkt_totals is not None:
                    # show only keys with >0 counts, ordered by a stable list
                    order = [
                        "pvt",
                        "velned",
                        "sol",
                        "gga",
                        "gll",
                        "gsa",
                        "gsv",
                        "rmc",
                        "vtg",
                    ]
                    parts = [
                        f"{k}:{totals.get(k, 0)}" for k in order if totals.get(k, 0) > 0
                    ]
                    txt = (
                        self._totals_blank_text
                        if not parts
                        else ("Packets total: " + ", ".join(parts))
                    )
                    self.lbl_pkt_totals.config(text=txt)
                # NMEA (text)
                for key, label in (
                    ("gga", "NMEA GGA"),
                    ("gll", "NMEA GLL"),
                    ("gsa", "NMEA GSA"),
                    ("gsv", "NMEA GSV"),
                    ("rmc", "NMEA RMC"),
                    ("vtg", "NMEA VTG"),
                ):
                    if key in nmea_map and key in self.nmea_tree.get_children():
                        sentence = nmea_map[key]
                        self.nmea_tree.item(key, values=(label, sentence))

                        # NEW: If VTG/RMC arrives, allow updating Speed from the current sentence
                        if self._current_profile_name not in ("NO_FIX", "GPS_DISABLE"):
                            kmh = None
                            if key == "vtg":
                                kmh = self._parse_vtg_kmh(sentence)
                            elif key == "rmc":
                                kmh = self._parse_rmc_kmh(sentence)
                            if kmh is not None:
                                # When got kmh from PVT/VTG/RMC:
                                self._last_speed_kmh = float(
                                    kmh
                                )  # cache last real speed
                                raw = self._parse_int_str(self.calib_var.get())
                                eff = self._effective_offset_from_kmh(
                                    self._last_speed_kmh, raw
                                )
                                disp = max(0.0, self._last_speed_kmh + eff)
                                self.lbl_speed_big.config(
                                    text=f"Speed: {self._last_speed_kmh:.1f} kmh.    Display speed {disp:.1f} kmh"
                                )
                                # also refresh the Effective offset label near the spinbox
                                self.lbl_eff_offset.config(
                                    text=f"Effective offset: {eff} kmh"
                                )
        except queue.Empty:
            pass
        self.root.after(80, self._drain_tx_queue)

    # ---------- Start / Stop / Apply ----------
    def start_sim(self):
        if self.sim_running:
            return
        cfg = self._current_config(validate_messages=True)
        if not cfg:
            messagebox.showerror(
                "Error", "Invalid configuration or no messages selected."
            )
            return

        self.stop_event = threading.Event()
        self._rebuild_tx_tables()
        self._clear_status_and_tx()

        def rate_cb(r):
            self.rate_queue.put(r)

        def prof_cb(p):
            self.profile_queue.put(p)

        def stats_cb(s):
            self.stats_queue.put(s)

        def tx_cb(n, u):
            self.tx_queue.put((n, u))

        def worker():
            try:
                # NOTE: tx_cb now accepts (nmea_map, ubx_map, totals_dict)
                def _tx_cb(n, u, t):
                    self.tx_queue.put((n, u, t))

                sim = UbxNmeaSimulator(
                    cfg,
                    self.ctrl_queue,
                    rate_cb,
                    prof_cb,
                    stats_cb,
                    _tx_cb,
                    self.stop_event,
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
        # Reset pause state on start
        self.paused = False
        self.pause_btn.configure(text=self._pause_label_cache[0])
        self._auto_resize_to_fit()

    def stop_sim(self):
        if not self.sim_running:
            return
        self.stop_event.set()  # snapshot remains visible

        # UI-side reset for totals line (new Start will recreate worker with zeroed totals)
        try:
            if self.lbl_pkt_totals is not None:
                self.lbl_pkt_totals.config(text=self._totals_blank_text)
        except Exception:
            pass

    # Pause/Resume toggle: ensure we display the latest stats just before pausing
    def toggle_pause(self):
        if not self.sim_running:
            return
        if not self.paused:
            # Drain queues để “đóng băng” UI ở trạng thái mới nhất
            self._drain_all_queues_once()
            self.paused = True
            try:
                self.pause_btn.configure(text=self._pause_label_cache[1])
            except Exception:
                pass
            # NEW: thông báo worker dừng sinh dữ liệu
            try:
                self.ctrl_queue.put_nowait({"cmd": "pause", "value": True})
            except Exception:
                pass
        else:
            self.paused = False
            try:
                self.pause_btn.configure(text=self._pause_label_cache[0])
            except Exception:
                pass
            # NEW: thông báo worker tiếp tục
            try:
                self.ctrl_queue.put_nowait({"cmd": "pause", "value": False})
            except Exception:
                pass

    # Internal helper: drain each queue synchronously exactly once (no reschedule)
    def _drain_all_queues_once(self):
        # Rate
        try:
            while True:
                rates = self.rate_queue.get_nowait()
                # mimic _drain_rate_queue body (UI labels update)
                self.pvt_rate.config(
                    text=(
                        f"act: {rates.get('pvt', 0.0):.2f} Hz"
                        if self.pvt_var.get() and self.ubx_master_var.get()
                        else ""
                    )
                )
                self.vel_rate.config(
                    text=(
                        f"act: {rates.get('velned', 0.0):.2f} Hz"
                        if self.vel_var.get() and self.ubx_master_var.get()
                        else ""
                    )
                )
                self.sol_rate.config(
                    text=(
                        f"act: {rates.get('sol', 0.0):.2f} Hz"
                        if self.sol_var.get() and self.ubx_master_var.get()
                        else ""
                    )
                )
                for key in ("gga", "gll", "gsa", "gsv", "rmc", "vtg"):
                    lbl, var = self.nmea_rate_labels.get(key), self.nmea_vars.get(key)
                    if lbl is not None and var is not None:
                        lbl.config(
                            text=(
                                f"act: {rates.get(key, 0.0):.2f} Hz"
                                if var.get() and self.nmea_master_var.get()
                                else ""
                            )
                        )
        except queue.Empty:
            pass
        # Profile
        try:
            while True:
                info = self.profile_queue.get_nowait()
                self._current_is_lost = bool(info.get("is_lost", False))
                self._current_profile_name = info.get("name", "—")
                self.active_prof_value.config(text=self._current_profile_name)
                p = float(info.get("progress", 0.0))
                self.active_prog["value"] = int(max(0.0, min(1.0, p)) * 1000)
        except queue.Empty:
            pass
        # Stats
        self._drain_stats_queue()  # will consume everything pending and update labels
        # TX
        self._drain_tx_queue()  # will consume pending TX once

    def apply_to_running(self):
        if not self.sim_running or not self._dirty:
            return
        cfg = self._current_config(validate_messages=True)
        if not cfg:
            messagebox.showerror(
                "Error", "Invalid configuration or no messages selected."
            )
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
        self.start_btn.configure(
            state=(
                "disabled"
                if running
                else (
                    "normal"
                    if self._has_any_message_selected()
                    and self._current_config(validate_messages=False)
                    else "disabled"
                )
            )
        )
        self.pause_btn.configure(state=("normal" if running else "disabled"))
        self.stop_btn.configure(state=("normal" if running else "disabled"))
        self.apply_btn.configure(
            state=("normal" if (running and self._dirty) else "disabled")
        )
        self.port_cb.configure(state=("disabled" if running else "readonly"))
        self.baud_cb.configure(
            state=(
                "disabled"
                if running
                else ("readonly" if self.port_var.get() else "disabled")
            )
        )
        # When stopping, ensure Pause label resets
        if not running:
            self.pause_btn.configure(text=self._pause_label_cache[0])

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
