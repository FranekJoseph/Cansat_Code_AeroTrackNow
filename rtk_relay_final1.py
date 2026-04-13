#!/usr/bin/env python3
"""
AeroTrackNow RTK Relay v0.6.3 — based on proven (2)3.py loop

Core relay loop is identical to rtk_relay_transport_final (2)3.py which
works reliably. Only addition: auto-reconnect on USB disconnect.

Usage:
  python "rtk_relay_transport_final (2).py" <base_gnss_port> <ground_station_port>
"""
import sys
import struct
import time
from collections import deque

import serial

RTCM3_PREAMBLE = 0xD3
BASE_BAUD = 115200
GS_BAUD = 115200
BASE_READ_BYTES = 1024
GS_READ_BYTES = 512
STATUS_INTERVAL_S = 1.0
BASE_RTCM_TIMEOUT_S = 3.0
GS_QUEUE_WARN_BYTES = 16384
MAX_PENDING_BYTES = 131072
FRAME_TTL_S = 4.0

# ---------- Per-frame RTCM logging (BASE_IN / GS_OUT) ----------
VERBOSE_RTCM = True            # master toggle — set False for silent operation
VERBOSE_RTCM_EVERY_N = 1       # print every Nth frame; 1 = every frame
VERBOSE_RTCM_LOG_DROPS = True  # also log TTL / queue-overflow drops

PACKET_FORMAT = '<BIiihHhHhhiihhhHHHBBBH'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
MSG_TELEMETRY = 0x03

RECONNECT_DELAY_S = 3.0
MAX_RECONNECT_ATTEMPTS = 50


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _build_crc24q_table():
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
    crc = 0
    for byte in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24Q_TABLE[(crc >> 16) ^ byte]
    return crc


# ---------- RTCM3 parser — IDENTICAL to working (2)3.py ----------
class RTCM3Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
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
            received_crc = int.from_bytes(frame[-3:], 'big')
            if crc24q(frame[:-3]) != received_crc:
                continue
            msg_type = (frame[3] << 4) | ((frame[4] >> 4) & 0x0F) if length >= 2 else 0
            frames.append((msg_type, frame))
        if len(self.buffer) > 4096:
            self.buffer = self.buffer[-1024:]
        return frames


# ---------- Telemetry decoder — IDENTICAL to working (2)3.py ----------
def decode_packet(raw):
    if len(raw) != PACKET_SIZE:
        return None
    fields = struct.unpack(PACKET_FORMAT, raw)
    if fields[0] != MSG_TELEMETRY:
        return None
    if crc16_ccitt(raw[:-2]) != fields[-1]:
        return None
    return {
        'time_ms': fields[1],
        'lat': fields[2] / 1e7,
        'lon': fields[3] / 1e7,
        'height_m': fields[4] / 10.0,
        'speed_mps': fields[7] / 100.0,
        'fix': fields[12],
        'hacc_mm': fields[13],
        'heading_deg': fields[14] / 10.0,
        'state': fields[18],
        'carr_soln': fields[19],
        'sats': fields[20],
    }


def safe_out_waiting(ser) -> int:
    try:
        return int(getattr(ser, 'out_waiting', 0) or 0)
    except Exception:
        return 0


def main():
    if len(sys.argv) != 3:
        print('Usage: python "rtk_relay_transport_final (2).py" <base_gnss_port> <ground_station_port>')
        sys.exit(1)

    base_port = sys.argv[1]
    gs_port = sys.argv[2]

    reconnect_count = 0
    while reconnect_count < MAX_RECONNECT_ATTEMPTS:
        ser_base = None
        ser_gs = None
        try:
            if reconnect_count > 0:
                print(f'\n--- Reconnect attempt {reconnect_count}/{MAX_RECONNECT_ATTEMPTS} '
                      f'(waiting {RECONNECT_DELAY_S:.0f}s) ---')
                time.sleep(RECONNECT_DELAY_S)

            print(f'Opening base GNSS on {base_port} at {BASE_BAUD} baud...')
            ser_base = serial.Serial(base_port, BASE_BAUD, timeout=0.05, write_timeout=0.5)
            print(' Base GNSS connected.')

            print(f'Opening ground station on {gs_port} at {GS_BAUD} baud...')
            ser_gs = serial.Serial(gs_port, GS_BAUD, timeout=0.05, write_timeout=0.5)
            print(' Ground station connected.')

            print('\nRTK Relay active! Forwarding complete RTCM frames in-order...')
            print('No automatic base configuration is performed in this version.')
            if reconnect_count > 0:
                print(f'(Reconnected after {reconnect_count} attempt(s))')
            print('Press Ctrl+C to quit.\n')

            reconnect_count = 0
            _run_relay_loop(ser_base, ser_gs)

        except serial.SerialException as e:
            print(f'\n\nSerial error (USB disconnect?): {e}')
            print('Will auto-reconnect...')
            reconnect_count += 1
        except KeyboardInterrupt:
            print('\n\nRTK Relay stopped by user.')
            break
        finally:
            for s in (ser_base, ser_gs):
                if s is not None:
                    try:
                        s.close()
                    except Exception:
                        pass

    if reconnect_count >= MAX_RECONNECT_ATTEMPTS:
        print(f'\nGave up after {MAX_RECONNECT_ATTEMPTS} reconnect attempts.')


# ---------- Main relay loop — IDENTICAL to working (2)3.py ----------
def _run_relay_loop(ser_base, ser_gs):
    """Main relay loop — raises SerialException on disconnect."""
    rtcm_parser = RTCM3Parser()
    telem_buffer = bytearray()
    pending_frames = deque()
    pending_bytes = 0

    pkt_count = 0
    err_count = 0
    base_serial_bytes_total = 0
    base_rtcm_bytes_total = 0
    rtcm_frames_total = 0
    rtcm_frames_forwarded = 0
    rtcm_frames_dropped = 0
    rtcm_bytes_forwarded = 0
    queue_high_water_frames = 0
    queue_high_water_bytes = 0
    last_base_rtcm_rx = time.time()
    t_start = time.time()  # reference for relative timestamps in BASE_IN / GS_OUT logs
    stats_last_print = time.time()
    last_telem_print = 0.0
    last_telem_rx = 0.0

    stats_prev_base_serial = 0
    stats_prev_base_rtcm = 0
    stats_prev_forwarded = 0
    stats_prev_frames_fwd = 0
    stats_prev_frames_drop = 0

    while True:
        now = time.time()

        base_data = ser_base.read(BASE_READ_BYTES)
        if base_data:
            base_serial_bytes_total += len(base_data)
            frames = rtcm_parser.feed(base_data)
            for msg_type, frame in frames:
                pending_frames.append((now, msg_type, frame))
                pending_bytes += len(frame)
                base_rtcm_bytes_total += len(frame)
                rtcm_frames_total += 1
                last_base_rtcm_rx = now
                if VERBOSE_RTCM and (rtcm_frames_total % VERBOSE_RTCM_EVERY_N) == 0:
                    hex_head = ' '.join(f'{b:02x}' for b in frame[:6])
                    print(
                        f"BASE_IN  t={now - t_start:7.2f}s msg={msg_type:4d} "
                        f"len={len(frame):4d}B head={hex_head}"
                    )

            if len(pending_frames) > queue_high_water_frames:
                queue_high_water_frames = len(pending_frames)
            if pending_bytes > queue_high_water_bytes:
                queue_high_water_bytes = pending_bytes

        # Forward complete RTCM frames to the GS strictly in original order.
        while pending_frames:
            ts, msg_type, frame = pending_frames[0]
            if (now - ts) > FRAME_TTL_S:
                pending_frames.popleft()
                pending_bytes -= len(frame)
                rtcm_frames_dropped += 1
                if VERBOSE_RTCM and VERBOSE_RTCM_LOG_DROPS:
                    print(
                        f"DROP_TTL t={now - t_start:7.2f}s msg={msg_type:4d} "
                        f"len={len(frame):4d}B age={(now - ts):.2f}s"
                    )
                continue
            if pending_bytes > MAX_PENDING_BYTES:
                pending_frames.popleft()
                pending_bytes -= len(frame)
                rtcm_frames_dropped += 1
                if VERBOSE_RTCM and VERBOSE_RTCM_LOG_DROPS:
                    print(
                        f"DROP_OVF t={now - t_start:7.2f}s msg={msg_type:4d} "
                        f"len={len(frame):4d}B queue={pending_bytes}B"
                    )
                continue
            # Keep the host->GS serial TX buffer from running away.
            if safe_out_waiting(ser_gs) > GS_QUEUE_WARN_BYTES:
                break
            try:
                written = ser_gs.write(frame)
            except serial.SerialTimeoutException:
                break
            pending_frames.popleft()
            pending_bytes -= len(frame)
            rtcm_frames_forwarded += 1
            rtcm_bytes_forwarded += written
            if VERBOSE_RTCM and (rtcm_frames_forwarded % VERBOSE_RTCM_EVERY_N) == 0:
                ok = (written == len(frame))
                flag = "OK  " if ok else "SHRT"
                print(
                    f"GS_OUT   t={now - t_start:7.2f}s msg={msg_type:4d} "
                    f"len={len(frame):4d}B wrote={written:4d}B {flag}"
                )
            now = time.time()

        gs_data = ser_gs.read(GS_READ_BYTES)
        if gs_data:
            telem_buffer.extend(gs_data)
            if len(telem_buffer) > 4096:
                telem_buffer = telem_buffer[-1024:]

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
            pkt = decode_packet(candidate)
            if pkt is not None:
                pkt_count += 1
                last_telem_rx = now
                carr = {0: 'None', 1: 'Float', 2: 'Fixed'}.get(pkt['carr_soln'], str(pkt['carr_soln']))
                # Print every telemetry packet (1Hz from CanSat — won't flood)
                print(
                    f"TELEM #{pkt_count} t={pkt['time_ms']/1000:.1f}s state={pkt['state']} "
                    f"fix={pkt['fix']} rtk={carr} sats={pkt['sats']} hAcc={pkt['hacc_mm']}mm "
                    f"alt={pkt['height_m']:.1f}m spd={pkt['speed_mps']:.2f} "
                    f"hdg={pkt['heading_deg']:.1f} "
                    f"lat={pkt['lat']:.7f} lon={pkt['lon']:.7f}"
                )
                last_telem_print = now
                telem_buffer = telem_buffer[PACKET_SIZE:]
            else:
                err_count += 1
                telem_buffer = telem_buffer[1:]

        if now - stats_last_print >= STATUS_INTERVAL_S:
            dt = max(now - stats_last_print, 1e-3)
            base_serial_rate = (base_serial_bytes_total - stats_prev_base_serial) / dt
            base_rtcm_rate = (base_rtcm_bytes_total - stats_prev_base_rtcm) / dt
            forward_rate = (rtcm_bytes_forwarded - stats_prev_forwarded) / dt
            frames_fwd_rate = (rtcm_frames_forwarded - stats_prev_frames_fwd) / dt
            frames_drop_rate = (rtcm_frames_dropped - stats_prev_frames_drop) / dt
            parse_eff = (100.0 * base_rtcm_rate / base_serial_rate) if base_serial_rate > 1e-6 else 0.0
            fwd_eff = (100.0 * forward_rate / base_rtcm_rate) if base_rtcm_rate > 1e-6 else 0.0
            print(
                f"RTCM_RATE base_serial={base_serial_rate:.0f}B/s base_valid={base_rtcm_rate:.0f}B/s "
                f"forwarded={forward_rate:.0f}B/s parse_eff={parse_eff:.0f}% fwd_eff={fwd_eff:.0f}% "
                f"frames_fwd={frames_fwd_rate:.1f}/s frames_drop={frames_drop_rate:.1f}/s "
                f"queue={len(pending_frames)} ({pending_bytes}B) gs_out_wait={safe_out_waiting(ser_gs)}B"
            )
            stats_last_print = now
            stats_prev_base_serial = base_serial_bytes_total
            stats_prev_base_rtcm = base_rtcm_bytes_total
            stats_prev_forwarded = rtcm_bytes_forwarded
            stats_prev_frames_fwd = rtcm_frames_forwarded
            stats_prev_frames_drop = rtcm_frames_dropped

        if now - last_base_rtcm_rx > BASE_RTCM_TIMEOUT_S:
            print(f" [WARN] No RTCM from BASE for {now - last_base_rtcm_rx:.1f}s")
            last_base_rtcm_rx = now


if __name__ == '__main__':
    main()
