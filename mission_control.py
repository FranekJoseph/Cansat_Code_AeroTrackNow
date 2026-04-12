#!/usr/bin/env python3
"""
AeroTrackNow Mission Control — Ground Station for Competition
==============================================================
Combines three functions in one script:
  1. Sends TargetPacket to flight controller (tells it where to land)
  2. Relays RTK corrections from base station GNSS via LoRa
  3. Displays live flight telemetry

Usage:
    python mission_control.py --gs-port <port> --target-lat <lat> --target-lon <lon> --target-alt <alt>

    # With RTK base station:
    python mission_control.py --gs-port /dev/ttyACM0 --base-port /dev/ttyUSB0 \
        --target-lat 52.229770 --target-lon 21.011780 --target-alt 100.0

    # Without RTK (telemetry + target only):
    python mission_control.py --gs-port /dev/ttyACM0 \
        --target-lat 52.229770 --target-lon 21.011780 --target-alt 100.0

Install dependency:
    pip install pyserial
"""

import sys
import struct
import time
import math
import datetime
import argparse
import serial

# ===================== CONSTANTS =====================
MSG_TARGET      = 0x01
MSG_RTK         = 0x02
MSG_TELEMETRY   = 0x03
MSG_GROUND_WIND = 0x04  # v5.3: ground wind uplink

RTCM3_PREAMBLE = 0xD3
MAX_RTCM_FRAME = 254       # LoRa payload limit
TDM_SEND_DELAY_S = 0.02   # 20ms after telemetry RX — matches rtk_relay.py
TDM_MAX_WAIT_S = 1.5       # fallback if no telemetry received
TARGET_SEND_INTERVAL_S = 5.0   # Re-send target every 5s until confirmed
GROUND_WIND_INTERVAL_S = 5.0  # v5.3: send ground wind every 5s
MIN_TX_GAP_S = 0.2             # 200ms between any LoRa transmissions
BAUD = 115200

RTCM_PRIORITY = {
    1005: 0, 1077: 1, 1074: 1, 1097: 2, 1094: 2, 1127: 3, 1124: 3,
}

# ===================== TELEMETRY PACKET FORMAT =====================
# Must match TelemetryPacket struct in flight.ino (50 bytes)
PACKET_FORMAT = '<BIiihHhHhhiihhhHHHBBBH'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

# Flight firmware states (v4.0)
FLIGHT_STATE_NAMES = {
    0: "BOOT", 1: "WAIT_DROP", 2: "WAIT_STABLE", 3: "GUIDED",
    4: "FINAL_APP", 5: "TERMINAL_HOME", 6: "FLARE", 7: "TERMINAL", 8: "LANDED"
}
# Test firmware states (for compatibility)
TEST_STATE_NAMES = {0: "WAITING", 1: "DESCENDING", 2: "LANDED"}

FIX_NAMES = {0: "No fix", 1: "Dead reck", 2: "2D", 3: "3D", 4: "3D+DGNSS", 5: "Time-only"}


# ===================== CRC16-CCITT =====================
def crc16_ccitt(data):
    """CRC16-CCITT matching the firmware implementation."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ===================== TARGET PACKET =====================
def build_target_packet(lat_deg, lon_deg, alt_m):
    """Build a 15-byte TargetPacket with CRC16.

    Format: <Biii H  (1+4+4+4+2 = 15 bytes)
      [0x01][lat_e7 int32][lon_e7 int32][alt_cm int32][crc16 uint16]
    """
    lat_e7 = int(round(lat_deg * 1e7))
    lon_e7 = int(round(lon_deg * 1e7))
    alt_cm = int(round(alt_m * 100))

    # Pack everything except CRC
    data = struct.pack('<Biii', MSG_TARGET, lat_e7, lon_e7, alt_cm)
    crc = crc16_ccitt(data)
    return data + struct.pack('<H', crc)


# ===================== GROUND WIND PACKET (v5.3) =====================
def build_ground_wind_packet(wind_n_mps, wind_e_mps):
    """Build a 7-byte GroundWindPacket with CRC16.

    Format: <Bhh H  (1+2+2+2 = 7 bytes)
      [0x04][wind_n_cms int16][wind_e_cms int16][crc16 uint16]
    """
    wind_n_cms = int(round(wind_n_mps * 100))
    wind_e_cms = int(round(wind_e_mps * 100))
    wind_n_cms = max(-2000, min(2000, wind_n_cms))  # clamp ±20 m/s
    wind_e_cms = max(-2000, min(2000, wind_e_cms))
    data = struct.pack('<Bhh', MSG_GROUND_WIND, wind_n_cms, wind_e_cms)
    crc = crc16_ccitt(data)
    return data + struct.pack('<H', crc)


# ===================== RTCM3 PARSER =====================
def _build_crc24q_table():
    """Build CRC-24Q lookup table (Qualcomm polynomial 0x1864CFB)."""
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        table.append(crc & 0xFFFFFF)
    return table

_CRC24Q_TABLE = _build_crc24q_table()

def crc24q(data: bytes) -> int:
    """Compute CRC-24Q over data (used by RTCM3 for frame integrity)."""
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24Q_TABLE[(crc >> 16) ^ byte]
    return crc


class RTCM3Parser:
    """Parses RTCM3 frames from a raw byte stream with CRC-24Q validation."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        """Feed raw bytes, return list of (msg_type, frame_bytes)."""
        self.buffer.extend(data)
        frames = []

        while len(self.buffer) >= 6:
            try:
                idx = self.buffer.index(RTCM3_PREAMBLE)
            except ValueError:
                self.buffer.clear()
                break

            if idx > 0:
                self.buffer = self.buffer[idx:]

            if len(self.buffer) < 3:
                break

            length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
            if length > 1023:
                self.buffer = self.buffer[1:]
                continue

            frame_size = length + 6
            if len(self.buffer) < frame_size:
                break

            frame = bytes(self.buffer[:frame_size])
            self.buffer = self.buffer[frame_size:]

            # CRC-24Q validation — reject corrupted frames before forwarding
            received_crc = int.from_bytes(frame[-3:], 'big')
            if crc24q(frame[:-3]) != received_crc:
                continue  # corrupted frame, skip

            msg_type = 0
            if length >= 2:
                msg_type = (frame[3] << 4) | ((frame[4] >> 4) & 0x0F)

            if frame_size > MAX_RTCM_FRAME:
                # ✓ v5.5 AUDIT NOTE: Accepted limitation — see rtk_relay.py for explanation.
                print(f"  WARN: RTCM msg {msg_type} dropped ({frame_size}B > {MAX_RTCM_FRAME}B LoRa limit — OK if GPS+Galileo)")
                continue

            frames.append((msg_type, frame))

        if len(self.buffer) > 4096:
            self.buffer = self.buffer[-1024:]

        return frames


# ===================== TELEMETRY DECODER =====================
def decode_telemetry(raw):
    """Decode a 50-byte telemetry packet. Returns dict or None."""
    if len(raw) != PACKET_SIZE:
        return None
    fields = struct.unpack(PACKET_FORMAT, raw)
    if fields[0] != MSG_TELEMETRY:
        return None
    if crc16_ccitt(raw[:-2]) != fields[-1]:
        return None

    mission_state = fields[18]

    # Default to flight interpretation. Latch to flight once state > 2 is seen.
    is_flight = mission_state > 2 or decode_telemetry._latched_flight
    if mission_state > 2:
        decode_telemetry._latched_flight = True

    carr_soln = fields[19]
    num_sats = fields[20]

    if is_flight:
        # Flight firmware: fields[12]=wind_north_cms, fields[13]=wind_east_cms
        return {
            'time_ms': fields[1], 'lat': fields[2] / 1e7, 'lon': fields[3] / 1e7,
            'height_dm': fields[4], 'press_hPa': fields[5] / 10.0,
            'temp_C': fields[6] / 10.0, 'speed_mps': fields[7] / 100.0,
            'servo_L': fields[8] / 1000.0, 'servo_R': fields[9] / 1000.0,
            'pred_lat': fields[10] / 1e7, 'pred_lon': fields[11] / 1e7,
            'wind_n': fields[12] / 100.0, 'wind_e': fields[13] / 100.0,
            'heading_deg': fields[14] / 10.0, 'heading_conf': fields[15] / 1000.0,
            'wind_rejects': fields[16], 'wind_layer_rejects': fields[17],
            'state': mission_state, 'is_flight': True,
            'carr_soln': carr_soln, 'num_sats': num_sats,
        }
    else:
        # Test firmware: fields[12]=gnss_fix, fields[13]=gnss_hacc_mm
        return {
            'time_ms': fields[1], 'lat': fields[2] / 1e7, 'lon': fields[3] / 1e7,
            'height_dm': fields[4], 'press_hPa': fields[5] / 10.0,
            'temp_C': fields[6] / 10.0, 'speed_mps': fields[7] / 100.0,
            'servo_L': fields[8] / 1000.0, 'servo_R': fields[9] / 1000.0,
            'gnss_fix': fields[12], 'gnss_hacc': fields[13],
            'heading_deg': fields[14] / 10.0, 'heading_conf': fields[15] / 1000.0,
            'test_type': fields[16], 'test_phase': fields[17],
            'state': mission_state, 'is_flight': False,
            'carr_soln': carr_soln, 'num_sats': num_sats,
        }

decode_telemetry._latched_flight = False


# ===================== DISPLAY =====================
def display_flight(pkt, stats, target):
    """Display flight firmware telemetry."""
    print("\033[2J\033[H", end="")

    state_name = FLIGHT_STATE_NAMES.get(pkt['state'], f"STATE_{pkt['state']}")
    height_m = pkt['height_dm'] / 10.0
    wind_speed = (pkt['wind_n']**2 + pkt['wind_e']**2)**0.5

    print("=" * 62)
    print("   AeroTrackNow MISSION CONTROL — Competition Flight")
    print("=" * 62)

    # State with color hint
    if pkt['state'] <= 2:
        print(f"  State: {state_name:15s}  (pre-flight)")
    elif pkt['state'] <= 5:
        print(f"  State: {state_name:15s}  (GUIDING)")
    elif pkt['state'] == 6:
        print(f"  State: {state_name:15s}  *** FLARE ***")
    elif pkt['state'] == 8:
        print(f"  State: {state_name:15s}  *** LANDED ***")
    else:
        print(f"  State: {state_name}")

    print(f"  Time:  {pkt['time_ms']/1000:.0f} s")
    print("-" * 62)

    # Position
    print(f"  Position:  {pkt['lat']:.7f}N  {pkt['lon']:.7f}E")
    print(f"  Altitude:  {height_m:8.1f} m AGL")
    print(f"  Speed:     {pkt['speed_mps']:8.2f} m/s")
    print("-" * 62)

    # Target info
    if target:
        dist = distance_m(pkt['lat'], pkt['lon'], target[0], target[1])
        print(f"  Target:    {target[0]:.7f}N  {target[1]:.7f}E  ({target[2]:.0f}m MSL)")
        print(f"  Distance:  {dist:.1f} m to target")
        if pkt.get('pred_lat') is not None and pkt.get('pred_lon') is not None:
            pred_dist = distance_m(pkt['pred_lat'], pkt['pred_lon'], target[0], target[1])
            print(f"  Predicted: {pred_dist:.1f} m landing error")
            if pred_dist > 2.0 and 3 <= pkt['state'] <= 5:
                print(f"\a  *** PREDICTED MISS: {pred_dist:.1f} m ***")
    print("-" * 62)

    # Wind
    wind_dir = (math.degrees(math.atan2(pkt['wind_e'], pkt['wind_n'])) + 180) % 360
    print(f"  Wind:      {wind_speed:.1f} m/s from {wind_dir:.0f} deg")
    print(f"  Heading:   {pkt['heading_deg'] % 360:6.1f} deg  (conf: {pkt['heading_conf']:.2f})")
    # v5.0: RTK status from carrier solution field
    carr_names = {0: "None", 1: "RTK Float", 2: "RTK Fixed"}
    carr = pkt.get('carr_soln', 0)
    sats = pkt.get('num_sats', 0)
    rtk_str = f"  RTK:       {carr_names.get(carr, '?'):10s}  Sats: {sats}"
    if carr == 2 and stats['rtk_fixed_since'] > 0:
        rtk_dur = time.time() - stats['rtk_fixed_since']
        rtk_str += f"  Fixed {rtk_dur:.0f}s"
    print(rtk_str)
    if 3 <= pkt['state'] <= 6 and carr < 2:
        print("\a  *** WARNING: RTK NOT FIXED DURING GUIDANCE ***")
    if pkt['state'] >= 3 and carr == 2 and stats['rtk_fixed_since'] > 0:
        rtk_dur = time.time() - stats['rtk_fixed_since']
        if rtk_dur < 30:
            print(f"  *** EKF CONVERGING — RTK Fixed only {rtk_dur:.0f}s (need ~30s) ***")
    print("-" * 62)

    # Servos
    print(f"  Servo L:   {pkt['servo_L']:+.3f}    Servo R: {pkt['servo_R']:+.3f}")
    print("-" * 62)

    # Stats
    tgt_status = "CONFIRMED" if stats['target_confirmed'] else "SENDING..."
    print(f"  Target:    {tgt_status}")
    print(f"  Telemetry: {stats['pkt_count']} pkts  Errors: {stats['err_count']}")
    total = stats['pkt_count'] + stats['err_count']
    if total > 20 and stats['err_count'] / total > 0.10:
        # ✓ v5.5 AUDIT FIX: Raised threshold from 5% to 10%. False sync attempts on
        # byte 0x03 in RTCM/ASCII data inflate error count — this is normal, not a link issue.
        print(f"\a  *** LINK QUALITY: {stats['err_count']/total*100:.0f}% error rate ***")
    if stats['rtcm_bytes'] > 0:
        drops = stats['rtcm_drops']
        drop_str = f"  drops: {drops}" if drops > 0 else ""
        print(f"  RTCM:      {stats['rtcm_bytes']} bytes  {stats['rtcm_rate']:.1f} fr/s{drop_str}")
    else:
        print(f"  RTCM:      No base station connected")
    if stats['last_rtcm_forward'] > 0 and 3 <= pkt['state'] <= 6:
        rtcm_age = time.time() - stats['last_rtcm_forward']
        if rtcm_age > 2.0:
            print(f"\a  *** RTCM DRY — no corrections for {rtcm_age:.0f}s ***")
    print("=" * 62)

    # Warnings
    if height_m < 5 and 3 <= pkt['state'] <= 6:
        print("\a  *** LOW ALTITUDE ***")
    if pkt['speed_mps'] > 15:
        print("\a  *** HIGH SPEED ***")
    if wind_speed > 8:
        print("\a  *** HIGH WIND — guidance may abort ***")


def display_test(pkt, stats):
    """Display test firmware telemetry (fallback)."""
    print("\033[2J\033[H", end="")
    state_name = TEST_STATE_NAMES.get(pkt['state'], f"STATE_{pkt['state']}")
    fix_name = FIX_NAMES.get(pkt.get('gnss_fix', 0), "??")
    height_m = pkt['height_dm'] / 10.0
    test_names = {0: "FREE_FLIGHT", 1: "SWEEP_UP", 2: "SWEEP_DOWN", 3: "TURN_RATE"}
    test_name = test_names.get(pkt.get('test_type', 0), "??")

    print("=" * 55)
    print("   AeroTrackNow — TEST FIRMWARE Telemetry")
    print("=" * 55)
    # v5.0: RTK status from carrier solution field (not fixType)
    carr = pkt.get('carr_soln', 0)
    sats = pkt.get('num_sats', 0)
    if carr >= 2:
        print(f"  >>> RTK FIXED <<<   hAcc: {pkt.get('gnss_hacc', '?')} mm  Sats: {sats}")
    elif carr >= 1:
        print(f"  >>> RTK FLOAT <<<   hAcc: {pkt.get('gnss_hacc', '?')} mm  Sats: {sats}")
    else:
        print(f"  GNSS: {fix_name}        hAcc: {pkt.get('gnss_hacc', '?')} mm  Sats: {sats}")
    print("-" * 55)
    print(f"  State: {state_name:12s}  Test: {test_name}")
    print(f"  Phase: {pkt.get('test_phase', '?'):>3}           Time: {pkt['time_ms']/1000:.0f} s")
    print("-" * 55)
    print(f"  GNSS:  {pkt['lat']:.7f}N  {pkt['lon']:.7f}E")
    print(f"  Alt:   {height_m:8.1f} m AGL")
    print(f"  Speed: {pkt['speed_mps']:8.2f} m/s")
    print("-" * 55)
    print(f"  Servo L: {pkt['servo_L']:+.3f}    Servo R: {pkt['servo_R']:+.3f}")
    print("-" * 55)
    print(f"  Packets: {stats['pkt_count']}    Errors: {stats['err_count']}")
    if stats['rtcm_bytes'] > 0:
        print(f"  RTCM: {stats['rtcm_bytes']} bytes  {stats['rtcm_rate']:.1f} fr/s")
    print("=" * 55)


def distance_m(lat1, lon1, lat2, lon2):
    """Haversine distance in meters."""
    R = 6371000.0
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = (math.sin(dLat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dLon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ===================== MAIN =====================
def main():
    parser = argparse.ArgumentParser(
        description=(
            "AeroTrackNow Mission Control — Target + RTK + Telemetry. "
            "NOTE: --base-port requires the Transport GS firmware "
            "(GroundStation_transport_seqfix). It will NOT work with "
            "AeroTrackNow_ground_station_code — that firmware does not frame "
            "RTCM as MSG_RTK=0x02 and CanSat will drop unframed RTCM."
        ))
    parser.add_argument("--gs-port", required=True,
                        help="Ground station Pico serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--base-port", default=None,
                        help="Base station GNSS serial port (e.g. /dev/ttyUSB0 or COM5). "
                             "Omit if no RTK base station.")
    parser.add_argument("--base-baud", type=int, default=BAUD,
                        help=f"Base GNSS baud rate (default {BAUD}). "
                             "Common alternatives: 9600, 38400, 230400.")
    parser.add_argument("--target-lat", type=float, required=True,
                        help="Target latitude in decimal degrees (e.g. 52.229770)")
    parser.add_argument("--target-lon", type=float, required=True,
                        help="Target longitude in decimal degrees (e.g. 21.011780)")
    parser.add_argument("--target-alt", type=float, required=True,
                        help="Target altitude in meters MSL (e.g. 100.0)")
    # v5.3: Ground wind uplink
    parser.add_argument("--wind-speed", type=float, default=None,
                        help="Ground wind speed in m/s (use with --wind-dir)")
    parser.add_argument("--wind-dir", type=float, default=None,
                        help="Ground wind FROM direction in degrees (meteorological, 0=N, 90=E)")
    parser.add_argument("--wind-n", type=float, default=None,
                        help="Ground wind north component m/s (alternative to --wind-speed/dir)")
    parser.add_argument("--wind-e", type=float, default=None,
                        help="Ground wind east component m/s (alternative to --wind-speed/dir)")
    args = parser.parse_args()

    # Validate wind arguments — must be paired
    if (args.wind_speed is not None) != (args.wind_dir is not None):
        parser.error("--wind-speed and --wind-dir must be used together")
    if (args.wind_n is not None) != (args.wind_e is not None):
        parser.error("--wind-n and --wind-e must be used together")

    target = (args.target_lat, args.target_lon, args.target_alt)
    # H4 FIX: Validate target coordinates before sending
    if not (-90 < target[0] < 90) or not (-180 < target[1] < 180):
        print(f"ERROR: Invalid target coordinates: {target[0]}, {target[1]}")
        sys.exit(1)
    if not (-500 < target[2] < 5000):
        print(f"ERROR: Invalid target altitude: {target[2]} m")
        sys.exit(1)
    target_packet = build_target_packet(*target)

    # v5.3: Resolve ground wind
    ground_wind_n = None
    ground_wind_e = None
    ground_wind_packet = None
    if args.wind_speed is not None and args.wind_dir is not None:
        # Meteorological convention: --wind-dir is direction wind blows FROM
        # Convert to N/E components (direction wind blows TO = FROM + 180)
        wind_to_rad = math.radians(args.wind_dir + 180.0)
        ground_wind_n = args.wind_speed * math.cos(wind_to_rad)
        ground_wind_e = args.wind_speed * math.sin(wind_to_rad)
    elif args.wind_n is not None and args.wind_e is not None:
        ground_wind_n = args.wind_n
        ground_wind_e = args.wind_e

    if ground_wind_n is not None:
        ground_wind_packet = build_ground_wind_packet(ground_wind_n, ground_wind_e)

    print(f"Target: {target[0]:.7f}N, {target[1]:.7f}E, {target[2]:.1f}m MSL")
    print(f"Target packet: {target_packet.hex()} ({len(target_packet)} bytes)")
    if ground_wind_packet:
        wind_spd = math.sqrt(ground_wind_n**2 + ground_wind_e**2)
        wind_from = math.degrees(math.atan2(-ground_wind_e, -ground_wind_n)) % 360
        print(f"Ground wind: {wind_spd:.1f} m/s from {wind_from:.0f} deg "
              f"(N={ground_wind_n:.2f}, E={ground_wind_e:.2f})")
    else:
        print("Ground wind: not set (use --wind-speed/--wind-dir or --wind-n/--wind-e)")
    print()

    # Open ground station serial
    print(f"Opening ground station on {args.gs_port}...")
    try:
        ser_gs = serial.Serial(args.gs_port, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.gs_port}: {e}")
        sys.exit(1)
    print(f"  Ground station connected.")

    # Open base station serial (optional)
    ser_base = None
    if args.base_port:
        print(f"Opening base GNSS on {args.base_port} at {args.base_baud} baud...")
        try:
            ser_base = serial.Serial(args.base_port, args.base_baud, timeout=0.05)
            print(f"  Base GNSS connected.")
        except serial.SerialException as e:
            print(f"WARNING: Cannot open base GNSS {args.base_port}: {e}")
            print(f"  Continuing without RTK.")
            ser_base = None

    print()
    print("Mission Control active!")
    print("  Sending target to flight controller...")
    print("  Press Ctrl+C to quit.")
    print()

    rtcm_parser = RTCM3Parser() if ser_base else None
    telem_buffer = bytearray()

    log_filename = f"flight_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_filename, 'w')
    log_file.write("timestamp,time_ms,lat,lon,height_m,pressure_hPa,temp_C,"
                   "speed_mps,heading_deg,heading_conf,servo_L,servo_R,"
                   "pred_lat,pred_lon,pred_error_m,"
                   "wind_n,wind_e,wind_rejects,wind_layer_rejects,"
                   "state,carr_soln,num_sats,dist_to_target,"
                   "rtcm_bytes_fwd,rtcm_rate,link_err_rate\n")
    print(f"  Logging to {log_filename}")

    # Pre-flight checklist
    print()
    print("=" * 62)
    print("  PRE-FLIGHT CHECKLIST")
    print("=" * 62)
    print(f"  [{'OK' if abs(target[0]) > 1 and abs(target[1]) > 1 else 'FAIL'}] Target coords valid: {target[0]:.7f}N, {target[1]:.7f}E")
    print(f"  [{'OK' if target[2] < 5000 else 'FAIL'}] Target alt reasonable: {target[2]:.0f} m MSL")
    print(f"  [OK] Ground station serial: {args.gs_port}")
    print(f"  [{'OK' if ser_base else '--'}] Base GNSS serial: {args.base_port or 'not connected'}"
          f"{' @ ' + str(args.base_baud) + ' baud' if ser_base else ''}")
    print(f"  [{'OK' if ground_wind_packet else '--'}] Ground wind: {'set' if ground_wind_packet else 'not set'}")
    print("=" * 62)

    checks_ok = (abs(target[0]) > 1 and abs(target[1]) > 1 and target[2] < 5000)
    if not checks_ok:
        resp = input("\n  *** PRE-FLIGHT CHECKS FAILED — Continue? (type 'yes'): ")
        if resp.strip().lower() != 'yes':
            print("  Aborting.")
            ser_gs.close()
            if ser_base:
                ser_base.close()
            sys.exit(1)
    print()

    stats = {
        'pkt_count': 0, 'err_count': 0,
        'rtcm_bytes': 0, 'rtcm_frames': 0, 'rtcm_drops': 0, 'rtcm_rate': 0.0,
        'target_confirmed': False, 'target_sends': 0,
        'last_rtcm_forward': 0.0,
        'rtk_fixed_since': 0.0,
        'start_time': time.time(),
        'max_height_m': 0.0,
        'rtk_fixed_total_s': 0.0,
        'last_dist': 0.0,
        'rtk_fixed_last_check': 0.0,
        'last_gs_rx_print': 0.0,
        'silent_gs_warned': False,
    }
    rtcm_frames_window = []
    pending_rtcm = []
    last_rtcm_send = 0.0
    last_target_send = 0.0
    last_ground_wind_send = 0.0  # v5.3
    last_tx_time = 0.0   # any TX (target or RTCM)
    pkt_state_cache = 0
    last_telem_rx_time = 0.0
    tdm_burst_sent = False
    landed_summary_printed = False

    try:
        while True:
            now = time.time()
            can_tx = (now - last_tx_time) >= MIN_TX_GAP_S
            try:
                # --- Send TargetPacket periodically (always during pre-flight, stop once guiding) ---
                if can_tx and (now - last_target_send) >= TARGET_SEND_INTERVAL_S:
                    # Stop once in guided flight — target is clearly received
                    if pkt_state_cache < 3:  # pre-flight states: BOOT, WAIT_DROP, WAIT_STABLE
                        ser_gs.write(target_packet)
                        last_target_send = now
                        last_tx_time = now
                        stats['target_sends'] += 1
                        can_tx = False
                        if pending_rtcm:
                            time.sleep(0.05)  # ensure Pico drains target before RTCM arrives

                # --- v5.3: Send ground wind periodically ---
                if ground_wind_packet and can_tx and pkt_state_cache < 8:
                    if (now - last_ground_wind_send) >= GROUND_WIND_INTERVAL_S:
                        ser_gs.write(ground_wind_packet)
                        last_ground_wind_send = now
                        last_tx_time = now
                        can_tx = False

                # --- Read RTCM3 from base GNSS ---
                if ser_base and rtcm_parser:
                    base_data = ser_base.read(512)
                    if base_data:
                        frames = rtcm_parser.feed(base_data)
                        for msg_type, frame in frames:
                            priority = RTCM_PRIORITY.get(msg_type, 10)
                            pending_rtcm.append((priority, msg_type, frame, now))
                            stats['rtcm_frames'] += 1
                            rtcm_frames_window.append(now)

                    # Send batched RTCM — TDM-aligned (shortly after telemetry) or fallback
                    if pending_rtcm and can_tx:
                        since_telem = now - last_telem_rx_time if last_telem_rx_time > 0 else 999.0
                        fallback = (now - last_rtcm_send) >= TDM_MAX_WAIT_S
                        if fallback:
                            tdm_burst_sent = False
                        if not tdm_burst_sent and (since_telem >= TDM_SEND_DELAY_S or fallback):
                            pending_rtcm.sort(key=lambda x: x[0])
                            batch = bytearray()
                            remaining = []
                            for priority, msg_type, frame, ts in pending_rtcm:
                                if now - ts > 5.0:
                                    stats['rtcm_drops'] += 1
                                    continue  # drop stale RTCM (>5s old)
                                if len(batch) + len(frame) <= MAX_RTCM_FRAME:
                                    batch.extend(frame)
                                elif priority <= 2 and len(remaining) < 10:
                                    remaining.append((priority, msg_type, frame, ts))
                                else:
                                    stats['rtcm_drops'] += 1
                            if batch:
                                # ✓ FIX C3: Transport GS adds its own [0x02][seq_hi][seq_lo] framing.
                                # Adding MSG_RTK (0x02) prefix here corrupted RTCM — GNSS saw
                                # [0x02][RTCM...] after GS stripped its 3-byte header, breaking RTK fix.
                                packet = bytes(batch)
                                ser_gs.write(packet)
                                stats['rtcm_bytes'] += len(batch)
                                stats['last_rtcm_forward'] = now
                                last_tx_time = now
                                last_rtcm_send = now
                            pending_rtcm = remaining
                            tdm_burst_sent = True

                # --- Read telemetry from ground station ---
                gs_data = ser_gs.read(512)
                if gs_data:
                    # Surface Pico diagnostic messages (WARN/ERROR/FATAL/GS_STATUS, heartbeat, RSSI)
                    _pico_prefixes = ('WARN:', 'ERROR:', 'FATAL:', 'GS_STATUS:', '#GS_RX', '#GS ')
                    if any(p.encode() in gs_data for p in _pico_prefixes):
                        try:
                            for line in gs_data.decode('ascii', errors='ignore').split('\n'):
                                line = line.strip()
                                if not line:
                                    continue
                                if not any(line.startswith(p) or p in line for p in _pico_prefixes):
                                    continue
                                # Rate-limit #GS_RX so it updates at most 1 Hz
                                if line.startswith('#GS_RX'):
                                    if now - stats['last_gs_rx_print'] < 1.0:
                                        continue
                                    stats['last_gs_rx_print'] = now
                                print(f"  [PICO] {line}")
                        except Exception:
                            pass
                    telem_buffer.extend(gs_data)
                    if len(telem_buffer) > 4096:
                        telem_buffer = telem_buffer[-1024:]

                # --- Detect silent ground station: RTCM flowing but no telemetry for 15s ---
                if (ser_base and stats['rtcm_bytes'] > 1000 and
                        last_telem_rx_time == 0.0 and
                        (now - stats['start_time']) > 15.0 and
                        not stats['silent_gs_warned']):
                    print("\n  *** WARNING: No telemetry received after 15s, but base is producing RTCM. ***")
                    print("  *** Check: (1) Transport GS flashed & powered? (2) UPLINK_PRIORITY_MODE=false? ***")
                    print("  *** (3) --gs-port pointing at Transport GS (not AeroTrackNow GS)? ***\n")
                    stats['silent_gs_warned'] = True

                # --- Decode telemetry ---
                while len(telem_buffer) >= PACKET_SIZE:
                    try:
                        idx = telem_buffer.index(MSG_TELEMETRY)
                    except ValueError:
                        telem_buffer.clear()
                        break
                    if idx > 0:
                        telem_buffer = telem_buffer[idx:]
                    if len(telem_buffer) < PACKET_SIZE:
                        break
                    candidate = bytes(telem_buffer[:PACKET_SIZE])
                    pkt = decode_telemetry(candidate)
                    if pkt is not None:
                        stats['pkt_count'] += 1
                        last_telem_rx_time = now
                        tdm_burst_sent = False

                        if pkt['state'] < 3 and pkt_state_cache >= 3:
                            stats['target_confirmed'] = False
                            last_target_send = 0.0
                            landed_summary_printed = False
                            print("\n  *** STATE REGRESSION — CanSat rebooted! Re-sending target. ***\n")
                        pkt_state_cache = pkt['state']
                        if pkt['state'] >= 3:
                            stats['target_confirmed'] = True

                        # RTCM frame rate
                        cutoff = now - 5.0
                        rtcm_frames_window[:] = [t for t in rtcm_frames_window if t > cutoff]
                        stats['rtcm_rate'] = len(rtcm_frames_window) / 5.0

                        # CSV logging
                        dist = distance_m(pkt['lat'], pkt['lon'], target[0], target[1]) if target else 0
                        pred_lat = pkt.get('pred_lat', 0)
                        pred_lon = pkt.get('pred_lon', 0)
                        pred_err = distance_m(pred_lat, pred_lon, target[0], target[1]) if target and (pred_lat != 0 or pred_lon != 0) else 0
                        log_file.write(f"{time.time():.3f},{pkt['time_ms']},"
                                       f"{pkt['lat']:.7f},{pkt['lon']:.7f},"
                                       f"{pkt['height_dm']/10.0:.1f},{pkt['press_hPa']:.1f},{pkt['temp_C']:.1f},"
                                       f"{pkt['speed_mps']:.2f},{pkt['heading_deg']:.1f},{pkt.get('heading_conf',0):.3f},"
                                       f"{pkt['servo_L']:.3f},{pkt['servo_R']:.3f},"
                                       f"{pred_lat:.7f},{pred_lon:.7f},{pred_err:.1f},"
                                       f"{pkt.get('wind_n',0):.2f},{pkt.get('wind_e',0):.2f},"
                                       f"{pkt.get('wind_rejects',0)},{pkt.get('wind_layer_rejects',0)},"
                                       f"{pkt['state']},{pkt.get('carr_soln',0)},{pkt.get('num_sats',0)},"
                                       f"{dist:.1f},"
                                       f"{stats['rtcm_bytes']},{stats['rtcm_rate']:.1f},"
                                       f"{stats['err_count']/(stats['pkt_count']+stats['err_count'])*100:.1f}\n")
                        log_file.flush()

                        # RTK Fixed duration tracking
                        carr = pkt.get('carr_soln', 0)
                        if carr == 2:
                            if stats['rtk_fixed_since'] == 0.0:
                                stats['rtk_fixed_since'] = now
                        else:
                            stats['rtk_fixed_since'] = 0.0

                        # Post-flight summary tracking
                        if pkt['height_dm'] / 10.0 > stats['max_height_m']:
                            stats['max_height_m'] = pkt['height_dm'] / 10.0
                        stats['last_dist'] = dist
                        if carr == 2 and stats['rtk_fixed_last_check'] > 0:
                            stats['rtk_fixed_total_s'] += now - stats['rtk_fixed_last_check']
                        stats['rtk_fixed_last_check'] = now if carr == 2 else 0.0

                        # Display
                        if pkt.get('is_flight', False):
                            display_flight(pkt, stats, target)
                        else:
                            display_test(pkt, stats)

                        # Auto-summary on first LANDED detection
                        if pkt['state'] == 8 and not landed_summary_printed:
                            landed_summary_printed = True
                            elapsed = time.time() - stats['start_time']
                            print(f"\n  === LANDED — Flight Summary ===")
                            print(f"  Duration: {elapsed:.0f}s ({elapsed/60:.1f} min)")
                            print(f"  Max altitude: {stats['max_height_m']:.1f} m AGL")
                            print(f"  Final distance: {dist:.1f} m to target")
                            if elapsed > 0:
                                rtk_pct = (stats['rtk_fixed_total_s'] / elapsed) * 100
                                print(f"  RTK Fixed uptime: {stats['rtk_fixed_total_s']:.0f}s ({rtk_pct:.0f}%)")
                            print(f"  RTCM forwarded: {stats['rtcm_bytes']} bytes  Drops: {stats['rtcm_drops']}")
                            print(f"  ================================\n")

                        telem_buffer = telem_buffer[PACKET_SIZE:]
                    else:
                        stats['err_count'] += 1
                        telem_buffer = telem_buffer[1:]

                time.sleep(0.01)
            except serial.SerialException as e:
                print(f"\n*** SERIAL ERROR: {e} ***")
                print("USB disconnected. Flight continues autonomously.")
                print("Reconnect USB and restart this script.")
                break

    except KeyboardInterrupt:
        print("\n\nMission Control stopped.")
        print(f"  Target sends: {stats['target_sends']}")
        print(f"  Target confirmed: {stats['target_confirmed']}")
        print(f"  Telemetry packets: {stats['pkt_count']}")
        print(f"  Telemetry errors: {stats['err_count']}")
        print(f"  RTCM bytes forwarded: {stats['rtcm_bytes']}  dropped: {stats['rtcm_drops']}")
        elapsed = time.time() - stats['start_time']
        print(f"  Flight duration: {elapsed:.0f}s ({elapsed/60:.1f} min)")
        print(f"  Max altitude: {stats['max_height_m']:.1f} m AGL")
        print(f"  Final distance to target: {stats['last_dist']:.1f} m")
        if elapsed > 0:
            rtk_pct = (stats['rtk_fixed_total_s'] / elapsed) * 100
            print(f"  RTK Fixed uptime: {stats['rtk_fixed_total_s']:.0f}s ({rtk_pct:.0f}%)")
    finally:
        log_file.close()
        ser_gs.close()
        if ser_base:
            ser_base.close()


if __name__ == "__main__":
    main()
