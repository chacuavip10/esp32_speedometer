#!/usr/bin/env python3
"""
UBX NAV Message Simulator (Windows-optimized)

- Sends UBX NAV-PVT (default), and optionally NAV-VELNED, NAV-SOL
- Selectable update rate (float Hz) and optional baudrate (default 115200)
- Profiles: GPS_LOST (stream no-fix), GPS_LOST_SILENT (silent 10s)  [toggle via --disable-gps-lost]
- Windows optimizations: single write per tick, perf_counter scheduling, enlarged buffers
- STATUS shows configured and ACTUAL send rate per message type
- Speed profiles now ramp linearly from minimum to maximum over profile duration

Usage:
  python ubx_simulator.py PORT [--rate float] [--baudrate int] [--enable list] [--disable list] [--disable-gps-lost]
  lists are comma-separated among: pvt,velned,sol

Notes:
  - Default behavior (no --enable): PVT ON, VELNED OFF, SOL OFF.
  - If --enable is provided, it OVERRIDES defaults (starts with all OFF),
    and only the listed message types are enabled. This allows turning PVT OFF.

Examples:
  python ubx_simulator.py COM3                       # PVT@10Hz, 115200
  python ubx_simulator.py COM3 --rate 12.5           # PVT@12.5Hz
  python ubx_simulator.py COM3 --enable velned,sol   # Only VELNED+SOL (PVT OFF)
  python ubx_simulator.py COM3 --disable-gps-lost    # No LOST profiles in cycle
"""

import serial
import struct
import time
import math
import random
import sys
from datetime import datetime, timezone
from typing import Optional, Dict

# ----------------- UBX helpers -----------------


def ubx_checksum(body: bytes):
    ck_a = 0
    ck_b = 0
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
    r22 = cos_lon
    r23 = 0.0
    r31 = -cos_lat * cos_lon
    r32 = -cos_lat * sin_lon
    r33 = -sin_lat
    vx = r11 * vn + r12 * ve + r13 * vd
    vy = r21 * vn + r22 * ve + r23 * vd
    vz = r31 * vn + r32 * ve + r33 * vd
    return vx, vy, vz


# ----------------- Simulator -----------------


class UbxNavPvtSimulator:
    def __init__(
        self,
        serial_port: str,
        baudrate: int = 115200,
        send_hz: float = 10.0,
        enable: Optional[Dict[str, bool]] = None,
        gps_lost_enabled: bool = True,
    ):
        # Serial (low-latency)
        self.serial = serial.Serial(serial_port, baudrate, timeout=0, write_timeout=0)
        try:
            # Windows-specific buffer enlargement
            self.serial.set_buffer_size(rx_size=8192, tx_size=8192)
        except Exception:
            pass

        if send_hz <= 0:
            raise ValueError("send_hz must be > 0")
        self.send_hz = float(send_hz)
        self.send_interval = 1.0 / self.send_hz

        # Message enable
        self.enabled = {"pvt": True, "velned": False, "sol": False}
        if enable is not None:
            # If caller passed an 'enable' map (e.g., via --enable), it overrides defaults
            self.enabled.update(enable)

        # LOST profiles toggle
        self.gps_lost_enabled = gps_lost_enabled

        print("\n" + "=" * 72)
        print(
            f"[CONNECTED] Port: {serial_port}  Baud: {baudrate}  Rate: {self.send_hz:.3g} Hz"
        )
        print("[MESSAGES ] " + self._format_msgs_line_config_only())
        print(f"[GPS_LOST ] {'ENABLED' if self.gps_lost_enabled else 'DISABLED'}")
        print("=" * 72)

        # Initial state (London)
        self.current_lat = 51.5074
        self.current_lon = -0.1278
        self.height_m = 15.0

        self.simulated_speed = 0.0  # km/h
        self.simulated_heading = 0.0  # deg
        self.time_of_week = 0  # ms

        # Profiles with min/max speed ranges
        self.PROFILES = {
            "STATIONARY": {"min": 0, "max": 0},
            "WALKING": {"min": 1, "max": 8},
            "CITY_DRIVING": {"min": 10, "max": 60},
            "HIGHWAY_DRIVING": {"min": 60, "max": 100},
            "HIGH_SPEED": {"min": 100, "max": 399},
            "GPS_LOST": {"min": -1, "max": -1},
            "GPS_LOST_SILENT": {"min": -2, "max": -2},
        }

        self.current_profile = "CITY_DRIVING"
        self.profile_change_time = time.perf_counter()
        self.profile_duration = 5.0  # seconds per profile
        self.gps_lost_start = 0.0

        # Quality
        self.fix_type = 3
        self.num_satellites = 12
        self.horizontal_accuracy = 2000  # mm
        self.speed_accuracy = 300  # mm/s

        # Actual-rate tracking
        self._sent_counts = {"pvt": 0, "velned": 0, "sol": 0}
        self._actual_rates = {"pvt": 0.0, "velned": 0.0, "sol": 0.0}
        self._last_status_time = time.perf_counter()

        print("\n" + "=" * 72)
        print("[UBX NAV SIMULATOR STARTED]")
        print(
            "Profiles cycle includes LOST profiles unless disabled via --disable-gps-lost"
        )
        print("Speed profiles now ramp linearly from min to max over 15 seconds")
        print("Press Ctrl+C to stop")
        print("=" * 72)

    # ---------- Helpers for status ----------

    def _format_msgs_line_config_only(self) -> str:
        parts = []
        for key, label in (("pvt", "PVT"), ("velned", "VELNED"), ("sol", "SOL")):
            if self.enabled.get(key, False):
                parts.append(f"{label}[ON] cfg:{self.send_hz:.3g}Hz")
            else:
                parts.append(f"{label}[OFF]")
        return " | ".join(parts)

    def _format_msgs_line_with_actual(self) -> str:
        parts = []
        for key, label in (("pvt", "PVT"), ("velned", "VELNED"), ("sol", "SOL")):
            if self.enabled.get(key, False):
                act = self._actual_rates.get(key, 0.0)
                parts.append(f"{label}[ON] cfg:{self.send_hz:.3g}Hz → act:{act:.2f}Hz")
            else:
                parts.append(f"{label}[OFF]")
        return " | ".join(parts)

    # ---------- Speed calculation with linear ramp ----------

    def _calculate_target_speed(self, now_pc: float) -> float:
        """Calculate target speed with linear ramp from min to max over profile duration"""
        profile_info = self.PROFILES.get(self.current_profile, {"min": 0, "max": 0})
        min_speed = profile_info["min"]
        max_speed = profile_info["max"]

        # Handle special profiles (GPS_LOST, etc.)
        if min_speed < 0:
            return min_speed

        # For STATIONARY profile
        if min_speed == 0 and max_speed == 0:
            return 0.0

        # Calculate elapsed time in current profile
        elapsed_time = now_pc - self.profile_change_time

        # Calculate progress through the profile (0.0 to 1.0)
        progress = min(1.0, elapsed_time / self.profile_duration)

        # Linear interpolation from min to max
        target_speed = min_speed + (max_speed - min_speed) * progress

        # Add small random variation
        variation = random.uniform(-2.0, 2.0)
        target_speed = max(0.0, target_speed + variation)

        return target_speed

    # ---------- Builders ----------

    def build_nav_pvt(self) -> bytes:
        now = datetime.now(timezone.utc)
        payload = struct.pack("<L", int(self.time_of_week))
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
        payload += struct.pack("<B", self.num_satellites)

        payload += struct.pack("<l", int(self.current_lon * 1e7))
        payload += struct.pack("<l", int(self.current_lat * 1e7))
        nowt = time.perf_counter()
        payload += struct.pack(
            "<l", int((self.height_m * 1000.0) + math.sin(nowt) * 5000)
        )
        payload += struct.pack(
            "<l", int((self.height_m * 1000.0) - 2000 + math.sin(nowt) * 5000)
        )
        payload += struct.pack("<L", self.horizontal_accuracy)
        payload += struct.pack("<L", 5000)

        speed_mms = self.simulated_speed * 1000.0 / 3.6
        heading_rad = math.radians(self.simulated_heading)
        vel_n = int(math.cos(heading_rad) * speed_mms)
        vel_e = int(math.sin(heading_rad) * speed_mms)
        vel_d = random.randint(-100, 100)

        payload += struct.pack("<lll", vel_n, vel_e, vel_d)
        payload += struct.pack("<l", int(abs(speed_mms)))  # gSpeed
        payload += struct.pack("<l", int(self.simulated_heading * 1e5))  # headMot
        payload += struct.pack("<L", self.speed_accuracy)  # sAcc
        payload += struct.pack("<L", 1_000_000)  # headAcc (~5.7 deg)
        payload += struct.pack("<H", 120)  # pDOP (1.20)
        payload += b"\x00" * 6
        payload += struct.pack("<l", int(self.simulated_heading * 1e5))  # headVeh
        payload += struct.pack("<hH", 500, 1000)  # magDec, magAcc
        return to_ubx(0x01, 0x07, payload)

    def build_nav_velned(self) -> bytes:
        speed_mms = self.simulated_speed * 1000.0 / 3.6
        speed_cms = int(abs(speed_mms) / 10.0)
        heading = int(self.simulated_heading * 1e5)
        sacc_cms = max(1, int(self.speed_accuracy / 10.0))
        cacc = 1_000_000  # ~10 deg
        heading_rad = math.radians(self.simulated_heading)
        velN_cms = int((math.cos(heading_rad) * speed_mms) / 10.0)
        velE_cms = int((math.sin(heading_rad) * speed_mms) / 10.0)
        velD_cms = int(random.randint(-100, 100) / 10.0)
        payload = struct.pack("<L", int(self.time_of_week))
        payload += struct.pack("<lll", velN_cms, velE_cms, velD_cms)
        payload += struct.pack("<L", speed_cms)  # speed (3D)
        payload += struct.pack("<L", speed_cms)  # gSpeed (2D)
        payload += struct.pack("<l", heading)
        payload += struct.pack("<LL", sacc_cms, cacc)
        return to_ubx(0x01, 0x12, payload)

    def build_nav_sol(self) -> bytes:
        now = datetime.now(timezone.utc)
        week, itow_ms = gps_week_and_tow_ms(now)
        x_m, y_m, z_m = geodetic_to_ecef(
            self.current_lat, self.current_lon, self.height_m
        )
        x_cm = int(x_m * 100.0)
        y_cm = int(y_m * 100.0)
        z_cm = int(z_m * 100.0)
        speed_mms = self.simulated_speed * 1000.0 / 3.6
        vn = math.cos(math.radians(self.simulated_heading)) * (speed_mms / 1000.0)
        ve = math.sin(math.radians(self.simulated_heading)) * (speed_mms / 1000.0)
        vd = 0.0
        vx_mps, vy_mps, vz_mps = ned_to_ecef(
            self.current_lat, self.current_lon, vn, ve, vd
        )
        vx_cms = int(vx_mps * 100.0)
        vy_cms = int(vy_mps * 100.0)
        vz_cms = int(vz_mps * 100.0)
        pacc_cm = max(1, int(self.horizontal_accuracy / 10.0))
        sacc_cms = max(1, int(self.speed_accuracy / 10.0))
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
        payload += struct.pack("<H", 120)  # pDOP 1.20
        payload += struct.pack("<B", 0)  # reserved1
        payload += struct.pack("<B", self.num_satellites)
        payload += struct.pack("<L", 0)  # reserved2
        return to_ubx(0x01, 0x06, payload)

    # ---------- Simulation loop state ----------

    def _profiles_cycle(self):
        base = [
            "STATIONARY",
            "WALKING",
            "CITY_DRIVING",
            "HIGHWAY_DRIVING",
            "HIGH_SPEED",
        ]
        if self.gps_lost_enabled:
            base += ["GPS_LOST", "GPS_LOST_SILENT"]
        return base

    def update_simulation(self, now_pc: float):
        # Cycle profile every 15 seconds
        if now_pc - self.profile_change_time > self.profile_duration:
            profiles = self._profiles_cycle()
            if self.current_profile not in profiles:
                self.current_profile = "CITY_DRIVING"
            idx = profiles.index(self.current_profile)
            self.current_profile = profiles[(idx + 1) % len(profiles)]
            self.profile_change_time = now_pc
            self.gps_lost_start = (
                now_pc
                if self.current_profile in ("GPS_LOST", "GPS_LOST_SILENT")
                else 0.0
            )

            profile_info = self.PROFILES.get(self.current_profile, {"min": 0, "max": 0})
            print("\n" + "=" * 72)
            print(f"[PROFILE CHANGE] >>> {self.current_profile.upper()}")
            if self.current_profile == "GPS_LOST":
                print("   Mode: STREAMING NO-FIX for 10s")
            elif self.current_profile == "GPS_LOST_SILENT":
                print("   Mode: SILENT (no messages) for 10s")
            else:
                if profile_info["min"] == profile_info["max"]:
                    print(f"   Speed: {profile_info['min']} km/h (constant)")
                else:
                    print(
                        f"   Speed: {profile_info['min']} → {profile_info['max']} km/h (linear ramp over 15s)"
                    )
            print("=" * 72)

        # LOST windows (skip entirely if disabled)
        if self.gps_lost_enabled and self.current_profile in (
            "GPS_LOST",
            "GPS_LOST_SILENT",
        ):
            if self.gps_lost_start == 0.0:
                self.gps_lost_start = now_pc
            if now_pc - self.gps_lost_start >= 10.0:
                profiles = self._profiles_cycle()
                idx = profiles.index(self.current_profile)
                self.current_profile = profiles[(idx + 1) % len(profiles)]
                self.profile_change_time = now_pc
                self.gps_lost_start = (
                    now_pc
                    if self.current_profile in ("GPS_LOST", "GPS_LOST_SILENT")
                    else 0.0
                )

                print("\n" + "=" * 72)
                print(f"[GPS RESTORED] >>> Next: {self.current_profile}")
                print("=" * 72)

            if self.current_profile == "GPS_LOST_SILENT":
                return False  # don't send anything
            if self.current_profile == "GPS_LOST":
                self.fix_type = 0
                self.num_satellites = random.randint(0, 4)
                self.horizontal_accuracy = random.randint(20000, 50000)
                self.speed_accuracy = random.randint(1500, 5000)

        # Speed/heading/position update with linear ramping
        target_speed = self._calculate_target_speed(now_pc)

        if target_speed >= 0:
            # Smooth transition to target speed
            speed_diff = target_speed - self.simulated_speed
            if abs(speed_diff) > 0.1:
                # Gradual acceleration/deceleration
                acceleration_rate = (
                    0.1  # Adjust this to control how quickly speed changes
                )
                self.simulated_speed += speed_diff * acceleration_rate
            else:
                self.simulated_speed = target_speed

            # Ensure speed doesn't go negative
            self.simulated_speed = max(0.0, self.simulated_speed)

            # Update heading with slight random changes
            self.simulated_heading = (
                self.simulated_heading + random.uniform(-1, 1)
            ) % 360

            # Update position if moving
            if self.simulated_speed > 0.1:
                # Scale position change based on speed
                position_factor = (
                    self.simulated_speed / 100.0
                )  # Normalize to reasonable movement
                self.current_lat += random.uniform(-0.0001, 0.0001) * position_factor
                self.current_lon += random.uniform(-0.0001, 0.0001) * position_factor

        # iTOW ms counter
        self.time_of_week = (
            self.time_of_week + int(self.send_interval * 1000)
        ) % 604_800_000

        # Quality (unless forced no-fix)
        if not (self.gps_lost_enabled and self.current_profile == "GPS_LOST"):
            if random.random() < 0.05:
                self.fix_type = 2
                self.num_satellites = random.randint(4, 8)
                self.horizontal_accuracy = random.randint(5000, 15000)
                self.speed_accuracy = random.randint(500, 2000)
            else:
                self.fix_type = 3
                self.num_satellites = random.randint(8, 16)
                self.horizontal_accuracy = random.randint(1000, 5000)
                self.speed_accuracy = random.randint(200, 800)
            if random.random() < 0.002:
                self.fix_type = 0
                self.num_satellites = random.randint(0, 4)

        return True

    # ---------- Status printing ----------

    def print_status(self, now_pc: float):
        elapsed = max(1e-6, now_pc - self._last_status_time)
        for k in self._sent_counts:
            self._actual_rates[k] = self._sent_counts[k] / elapsed
        self._sent_counts = {k: 0 for k in self._sent_counts}
        self._last_status_time = now_pc

        msgs_line = self._format_msgs_line_with_actual()

        # Calculate progress and target speed for status display
        profile_info = self.PROFILES.get(self.current_profile, {"min": 0, "max": 0})
        elapsed_time = now_pc - self.profile_change_time
        progress = min(100.0, (elapsed_time / self.profile_duration) * 100.0)
        target_speed = self._calculate_target_speed(now_pc)

        if self.gps_lost_enabled and self.current_profile == "GPS_LOST_SILENT":
            remaining = max(0.0, 10.0 - (now_pc - self.gps_lost_start))
            print("\n" + "*" * 60)
            print("[STATUS] GPS LOST SILENT")
            print(f"   No signal ({remaining:.1f}s remaining)")
            print(f"   Messages: {msgs_line}")
            print("   Lat: ---.------, Lon: ---.------")
            print("*" * 60)
        elif self.gps_lost_enabled and self.current_profile == "GPS_LOST":
            remaining = max(0.0, 10.0 - (now_pc - self.gps_lost_start))
            print("\n" + "*" * 60)
            print("[STATUS] GPS LOST - STREAMING NO-FIX")
            print(f"   Remaining: {remaining:.1f}s")
            print(f"   Messages:  {msgs_line}")
            print(f"   Sats: {self.num_satellites}, Fix: {self.fix_type}")
            print(f"   Lat: {self.current_lat:.6f}, Lon: {self.current_lon:.6f}")
            print("*" * 60)
        else:
            print("\n" + "-" * 60)
            profile_display = f"{self.current_profile} ({progress:.1f}%)"
            print(
                f"[STATUS] {profile_display}   (GPS_LOST={'ON' if self.gps_lost_enabled else 'OFF'})"
            )
            print(f"   Messages:  {msgs_line}")

            if profile_info["min"] == profile_info["max"]:
                print(f"   Speed:   {self.simulated_speed:.1f} km/h (constant)")
            else:
                print(
                    f"   Speed:   {self.simulated_speed:.1f} km/h (target: {target_speed:.1f}, range: {profile_info['min']}-{profile_info['max']})"
                )

            print(f"   Heading: {self.simulated_heading:.1f}°")
            print(f"   Sats:    {self.num_satellites}, Fix: {self.fix_type}")
            print(f"   Lat/Lon: {self.current_lat:.6f}, {self.current_lon:.6f}")
            print("-" * 60)

    # ---------- Main loop ----------

    def run(self):
        try:
            self.serial.reset_output_buffer()
        except Exception:
            pass

        next_send = time.perf_counter() + self.send_interval
        next_status = time.perf_counter() + 2.0
        try:
            while True:
                now_pc = time.perf_counter()

                should_send = self.update_simulation(now_pc)

                if should_send and now_pc >= next_send:
                    out = bytearray()
                    if self.enabled.get("pvt", False):
                        out += self.build_nav_pvt()
                        self._sent_counts["pvt"] += 1
                    if self.enabled.get("velned", False):
                        out += self.build_nav_velned()
                        self._sent_counts["velned"] += 1
                    if self.enabled.get("sol", False):
                        out += self.build_nav_sol()
                        self._sent_counts["sol"] += 1
                    if out:
                        try:
                            self.serial.write(out)
                        except serial.SerialTimeoutException:
                            pass

                    skips = max(1, int((now_pc - next_send) // self.send_interval) + 1)
                    next_send += skips * self.send_interval

                if now_pc >= next_status:
                    self.print_status(now_pc)
                    steps = max(1, int((now_pc - next_status) // 2.0) + 1)
                    next_status += steps * 2.0

                sleep_time = min(0.01, max(0.0, next_send - time.perf_counter()))
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\n" + "=" * 72)
            print("[STOP] Simulation stopped by user")
            print("=" * 72)
        finally:
            self.serial.close()
            print("\n" + "=" * 72)
            print("[CLOSED] Serial port closed")
            print("=" * 72)


# ----------------- CLI -----------------


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print(
            "  python ubx_simulator.py PORT [--rate float] [--baudrate int] [--enable list] [--disable list] [--disable-gps-lost]"
        )
        print("  lists are comma-separated among: pvt,velned,sol")
        sys.exit(1)

    port = sys.argv[1]
    send_hz = 10.0
    baudrate = 115200
    # Defaults: PVT ON, others OFF
    enable = {"pvt": True, "velned": False, "sol": False}
    gps_lost_enabled = True  # default ON

    i = 2
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == "--rate" and (i + 1) < len(sys.argv):
            try:
                send_hz = float(sys.argv[i + 1])
                if send_hz <= 0:
                    raise ValueError
            except ValueError:
                print("Error: --rate must be a positive float.")
                sys.exit(2)
            i += 2
        elif arg == "--baudrate" and (i + 1) < len(sys.argv):
            try:
                baudrate = int(sys.argv[i + 1])
                if baudrate <= 0:
                    raise ValueError
            except ValueError:
                print("Error: --baudrate must be a positive integer.")
                sys.exit(2)
            i += 2
        elif arg == "--enable" and (i + 1) < len(sys.argv):
            # OVERRIDE defaults: start all OFF, enable only those listed (allows turning PVT off)
            enable = {"pvt": False, "velned": False, "sol": False}
            items = [s.strip().lower() for s in sys.argv[i + 1].split(",") if s.strip()]
            for k in items:
                if k in enable:
                    enable[k] = True
            i += 2
        elif arg == "--disable" and (i + 1) < len(sys.argv):
            items = [s.strip().lower() for s in sys.argv[i + 1].split(",") if s.strip()]
            for k in items:
                if k in enable:
                    enable[k] = False
            i += 2
        elif arg == "--disable-gps-lost":
            gps_lost_enabled = False
            i += 1
        else:
            i += 1

    try:
        sim = UbxNavPvtSimulator(
            port,
            baudrate=baudrate,
            send_hz=send_hz,
            enable=enable,
            gps_lost_enabled=gps_lost_enabled,
        )
        sim.run()
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print("\nAvailable ports:")
        import serial.tools.list_ports

        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} - {p.description}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
