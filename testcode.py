#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nurirobot RS-485 control demo for MC/SA series
- Read current settings via feedback requests, then re-apply the same settings via "set" commands
- Run: Pos&Speed (0x01), AccPos (0x02), AccSpeed (0x03), and (if SA) Open-Loop (0x11) for 1s each, then stop
Spec refs:
  * MC-RS485 V1.0.1: control (0x01~0x03), setters (0x04~0x0F/0x10), feedback (0xA0.. -> 0xD0..FD)
  * SA-RS485 V1.0.2: Open-Loop drive (0x11)
"""

import argparse
import time
from dataclasses import dataclass
from typing import List, Dict, Optional

import serial
try:
    from serial.rs485 import RS485Settings
except Exception:
    RS485Settings = None


# -----------------------------
# Common protocol constants
# -----------------------------
HEADER = bytes((0xFF, 0xFE))

# Mode values (MC shared)
MODE_POS_SPEED         = 0x01  # 위치, 속도제어 (direction[1], pos[2](0.01°), spd[2](0.1RPM))
MODE_ACC_POS           = 0x02  # 가감속 위치제어 (direction[1], pos[2], arrive[1](0.1s))
MODE_ACC_SPEED         = 0x03  # 가감속 속도제어 (direction[1], spd[2], arrive[1](0.1s))
MODE_SET_POS_CTRL      = 0x04  # 위치제어기 설정 (Kp,Ki,Kd, rated[0.1A])
MODE_SET_SPD_CTRL      = 0x05  # 속도제어기 설정 (Kp,Ki,Kd, rated[0.1A])
MODE_SET_ID            = 0x06  # ID 설정 (주의: 실장비 변경 위험)
MODE_SET_BAUD          = 0x07  # 통신속도 설정 (주의: 즉시 변경)
MODE_SET_RESPTIME      = 0x08  # 통신 응답시간 설정 (value[100us])
MODE_SET_RATED_SPD     = 0x09  # 모터 정격속도 설정 (RPM, 2B)
MODE_SET_RESOLUTION    = 0x0A  # 분해능 설정 (2B)
MODE_SET_RATIO         = 0x0B  # 감속비 설정 (0.1ratio, 2B)
MODE_SET_ONOFF         = 0x0C  # 제어 On/Off (0x00 On, 0x01 Off)
MODE_SET_POS_MODE      = 0x0D  # 위치제어 모드 (0x00 절대, 0x01 상대)
MODE_SET_CTRL_DIR      = 0x0E  # 제어 방향 (0x00 CCW, 0x01 CW)
MODE_RESET_POS         = 0x0F  # 위치 초기화
MODE_RESET_FACTORY     = 0x10  # 공장 초기화 (주의)

# SA series only
MODE_OPEN_LOOP         = 0x11  # Open-Loop 구동 (direction[1], duty[2] in 0.01%)

# Feedback requests (송신)
REQ = {
    "ping":           0xA0,
    "pos":            0xA1,
    "spd":            0xA2,
    "pos_ctrl":       0xA3,
    "spd_ctrl":       0xA4,
    "resp_time":      0xA5,
    "rated_speed":    0xA6,
    "resolution":     0xA7,
    "ratio":          0xA8,
    "ctrl_onoff":     0xA9,
    "pos_ctrl_mode":  0xAA,
    "ctrl_dir":       0xAB,
    "firmware":       0xCD,
}
# Feedback responses (수신)
FEED = {
    "ping":           0xD0,
    "pos":            0xD1,
    "spd":            0xD2,
    "pos_ctrl":       0xD3,
    "spd_ctrl":       0xD4,
    "resp_time":      0xD5,
    "rated_speed":    0xD6,
    "resolution":     0xD7,
    "ratio":          0xD8,
    "ctrl_onoff":     0xD9,
    "pos_ctrl_mode":  0xDA,
    "ctrl_dir":       0xDB,
    "firmware":       0xFD,
}


# -----------------------------
# Bytes / checksum helpers
# -----------------------------
def be16(msb: int, lsb: int) -> int:
    return ((msb & 0xFF) << 8) | (lsb & 0xFF)

def to_be16(v: int) -> bytes:
    return bytes(((v >> 8) & 0xFF, v & 0xFF))

def calc_checksum(frame_with_placeholder: bytes) -> int:
    # ~(sum of all bytes except header[0:2] and checksum[4]) & 0xFF
    s = 0
    for i, b in enumerate(frame_with_placeholder):
        if i in (0, 1, 4):
            continue
        s = (s + (b & 0xFF)) & 0xFF
    return (~s) & 0xFF

def build_frame(dev_id: int, mode: int, data: bytes = b'') -> bytes:
    # Frame: FF FE | ID | LEN | CHK | MODE | DATA...
    # LEN includes CHK + MODE + DATA
    size = 1 + 1 + len(data)
    buf = bytearray(6 + len(data))
    buf[0:2] = HEADER
    buf[2] = dev_id & 0xFF
    buf[3] = size & 0xFF
    buf[5] = mode & 0xFF
    if data:
        buf[6:] = data
    buf[4] = calc_checksum(buf)
    return bytes(buf)


# -----------------------------
# Stream parser (robust)
# -----------------------------
class StreamParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk: bytes) -> List[bytes]:
        frames: List[bytes] = []
        if not chunk:
            return frames
        self.buf.extend(chunk)

        while True:
            start = self.buf.find(HEADER)
            if start < 0:
                self.buf.clear()
                break
            if start > 0:
                del self.buf[:start]
            if len(self.buf) < 6:
                break

            length = self.buf[3]
            total = length + 4
            if total < 6:
                del self.buf[:2]
                continue

            if len(self.buf) < total:
                nxt = self.buf.find(HEADER, 2)
                if 0 <= nxt < len(self.buf):
                    del self.buf[:nxt]
                    continue
                break

            frame = bytes(self.buf[:total])
            try:
                print("Recv Frame:", ' '.join(f"{b:02X}" for b in frame))
            except Exception:
                # 출력 도중 문제가 생겨도 파싱에는 영향이 없도록 무시합니다.
                pass
            
            if calc_checksum(frame) != frame[4]:
                del self.buf[:2]
                continue
            frames.append(frame)
            del self.buf[:total]
        return frames


# -----------------------------
# Serial link
# -----------------------------
@dataclass
class LinkConfig:
    port: str
    baud: int = 9600
    timeout: float = 0.2
    rs485_rts_enable: bool = False

class Link:
    def __init__(self, cfg: LinkConfig):
        self.cfg = cfg
        self.ser: Optional[serial.Serial] = None
        self.parser = StreamParser()

    def open(self):
        self.ser = serial.Serial()
        self.ser.port = self.cfg.port
        self.ser.baudrate = self.cfg.baud
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = self.cfg.timeout
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        if self.cfg.rs485_rts_enable and RS485Settings is not None:
            self.ser.rs485_mode = RS485Settings(
                rts_level_for_tx=True, rts_level_for_rx=False,
                delay_before_tx=0.0, delay_before_rx=0.0
            )
        self.ser.open()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def write(self, frame: bytes):
        self.ser.write(frame)
        self.ser.flush()
        try:
            print("Send Frame:", ' '.join(f"{b:02X}" for b in frame))
        except Exception:
            # 출력 도중 문제가 생겨도 파싱에는 영향이 없도록 무시합니다.
            pass   

    def read_frames(self) -> List[bytes]:
        chunk = self.ser.read(1024)
        return self.parser.feed(chunk) if chunk else []


# -----------------------------
# Feedback: read current settings (MC spec)
# -----------------------------
def request_and_wait(link: Link, dev_id: int, req_mode: int, expect_feed_mode: int, wait_s=0.35):
    frm = build_frame(dev_id, req_mode, b'')
    link.write(frm)
    end = time.monotonic() + wait_s
    while time.monotonic() < end:
        for f in link.read_frames():
            if len(f) >= 6 and f[2] == (dev_id & 0xFF) and f[5] == (expect_feed_mode & 0xFF):
                return f
        time.sleep(0.005)
    return None

def parse_feed_pos_ctrl(f: bytes) -> Dict:
    d = f[6:]
    return {"kp": d[0], "ki": d[1], "kd": d[2], "rated_current_a": d[3] * 0.1}

def parse_feed_spd_ctrl(f: bytes) -> Dict:
    d = f[6:]
    return {"kp": d[0], "ki": d[1], "kd": d[2], "rated_current_a": d[3] * 0.1}

def parse_feed_settings(link: Link, dev_id: int) -> Dict:
    """
    읽을 수 있는 현재 설정/상태를 모두 피드백 요청으로 수집.
    (D3/D4/D5/D6/D7/D8/D9/DA/DB/FD 등)
    """
    out: Dict = {}
    q = [
        ("pos_ctrl",      REQ["pos_ctrl"],      FEED["pos_ctrl"],      parse_feed_pos_ctrl),
        ("spd_ctrl",      REQ["spd_ctrl"],      FEED["spd_ctrl"],      parse_feed_spd_ctrl),
        ("resp_time",     REQ["resp_time"],     FEED["resp_time"],     lambda f: {"resptime_100us": f[6]}),
        ("rated_speed",   REQ["rated_speed"],   FEED["rated_speed"],   lambda f: {"rated_rpm": be16(f[6], f[7])}),
        ("resolution",    REQ["resolution"],    FEED["resolution"],    lambda f: {"resolution": be16(f[6], f[7])}),
        ("ratio",         REQ["ratio"],         FEED["ratio"],         lambda f: {"ratio": be16(f[6], f[7]) / 10.0}),
        ("ctrl_onoff",    REQ["ctrl_onoff"],    FEED["ctrl_onoff"],    lambda f: {"on": (f[6] == 0x00)}),
        ("pos_ctrl_mode", REQ["pos_ctrl_mode"], FEED["pos_ctrl_mode"], lambda f: {"pos_mode": "absolute" if f[6]==0x00 else "relative"}),
        ("ctrl_dir",      REQ["ctrl_dir"],      FEED["ctrl_dir"],      lambda f: {"ctrl_dir": "CCW" if f[6]==0x00 else "CW"}),
        ("firmware",      REQ["firmware"],      FEED["firmware"],      lambda f: {"fw_version": f[6]}),
    ]
    for key, req_m, feed_m, parser in q:
        fr = request_and_wait(link, dev_id, req_m, feed_m)
        if fr:
            out[key] = parser(fr)
        else:
            out[key] = None
    return out


# -----------------------------
# Setters (MC spec + SA Open-Loop)
# -----------------------------
def set_pos_ctrl(link: Link, dev_id: int, kp: int, ki: int, kd: int, rated_current_a: float):
    data = bytes((kp & 0xFF, ki & 0xFF, kd & 0xFF, int(round(rated_current_a * 10)) & 0xFF))
    link.write(build_frame(dev_id, MODE_SET_POS_CTRL, data))

def set_spd_ctrl(link: Link, dev_id: int, kp: int, ki: int, kd: int, rated_current_a: float):
    data = bytes((kp & 0xFF, ki & 0xFF, kd & 0xFF, int(round(rated_current_a * 10)) & 0xFF))
    link.write(build_frame(dev_id, MODE_SET_SPD_CTRL, data))

def set_resp_time(link: Link, dev_id: int, hundred_us: int):
    link.write(build_frame(dev_id, MODE_SET_RESPTIME, bytes((hundred_us & 0xFF,))))

def set_rated_speed(link: Link, dev_id: int, rpm: int):
    link.write(build_frame(dev_id, MODE_SET_RATED_SPD, to_be16(max(1, min(65533, int(rpm))))))

def set_resolution(link: Link, dev_id: int, res: int):
    link.write(build_frame(dev_id, MODE_SET_RESOLUTION, to_be16(max(1, min(65533, int(res))))))

def set_ratio(link: Link, dev_id: int, ratio: float):
    raw = max(1, min(65533, int(round(ratio * 10.0))))
    link.write(build_frame(dev_id, MODE_SET_RATIO, to_be16(raw)))

def set_onoff(link: Link, dev_id: int, on: bool):
    link.write(build_frame(dev_id, MODE_SET_ONOFF, bytes((0x00 if on else 0x01,))))

def set_pos_mode(link: Link, dev_id: int, absolute: bool):
    link.write(build_frame(dev_id, MODE_SET_POS_MODE, bytes((0x00 if absolute else 0x01,))))

def set_ctrl_dir(link: Link, dev_id: int, cw: bool):
    link.write(build_frame(dev_id, MODE_SET_CTRL_DIR, bytes((0x01 if cw else 0x00,))))

def reset_pos(link: Link, dev_id: int):
    link.write(build_frame(dev_id, MODE_RESET_POS, b''))


# -----------------------------
# Controllers (run then stop)
# -----------------------------
def cmd_pos_speed(link: Link, dev_id: int, direction_cw: bool, pos_deg: float, spd_rpm: float):
    pos_raw = max(0, min(65533, int(round(pos_deg * 100.0))))     # 0.01°
    spd_raw = max(0, min(65533, int(round(spd_rpm * 10.0))))      # 0.1RPM
    data = bytes((0x01 if direction_cw else 0x00,)) + to_be16(pos_raw) + to_be16(spd_raw)
    link.write(build_frame(dev_id, MODE_POS_SPEED, data))

def cmd_acc_pos(link: Link, dev_id: int, direction_cw: bool, pos_deg: float, arrive_s: float):
    pos_raw = max(0, min(65533, int(round(pos_deg * 100.0))))     # 0.01°
    arr_raw = max(1, min(255, int(round(arrive_s * 10.0))))       # 0.1s
    data = bytes((0x01 if direction_cw else 0x00,)) + to_be16(pos_raw) + bytes((arr_raw,))
    link.write(build_frame(dev_id, MODE_ACC_POS, data))

def cmd_acc_speed(link: Link, dev_id: int, direction_cw: bool, spd_rpm: float, arrive_s: float):
    spd_raw = max(0, min(65533, int(round(spd_rpm * 10.0))))      # 0.1RPM
    arr_raw = max(1, min(255, int(round(arrive_s * 10.0))))       # 0.1s
    data = bytes((0x01 if direction_cw else 0x00,)) + to_be16(spd_raw) + bytes((arr_raw,))
    link.write(build_frame(dev_id, MODE_ACC_SPEED, data))

def cmd_open_loop(link: Link, dev_id: int, direction_cw: bool, duty_percent: float):
    # SA 전용: duty in 0.01% (0..100.00%)
    duty_raw = max(0, min(10000, int(round(duty_percent * 100.0))))
    data = bytes((0x01 if direction_cw else 0x00,)) + to_be16(duty_raw)
    link.write(build_frame(dev_id, MODE_OPEN_LOOP, data))

def safe_stop(link: Link, dev_id: int):
    # 속도를 0RPM으로 짧게 감속 후 제어 Off
    cmd_acc_speed(link, dev_id, direction_cw=False, spd_rpm=0.0, arrive_s=0.2)
    time.sleep(0.25)
    set_onoff(link, dev_id, on=False)  # 0x01 = Off
    time.sleep(0.05)


# -----------------------------
# Demo flow
# -----------------------------
def demo(series: str, port: str, dev_id: int, baud: int, rs485_rts_enable: bool):
    link = Link(LinkConfig(port=port, baud=baud, rs485_rts_enable=rs485_rts_enable))
    link.open()
    try:
        print(f"[OPEN] {port} @{baud}bps, id=0x{dev_id:02X}, series={series}")

        # 1) 현재 설정값 읽기 (feedback)
        cur = parse_feed_settings(link, dev_id)
        print("\n[CURRENT SETTINGS]")
        for k, v in cur.items():
            print(f"- {k}: {v}")

        # 2) 읽은 설정을 그대로 설정(송신)으로 반영 (안전 데모)
        if cur.get("pos_ctrl"):
            set_pos_ctrl(link, dev_id, **cur["pos_ctrl"])
        if cur.get("spd_ctrl"):
            set_spd_ctrl(link, dev_id, **cur["spd_ctrl"])
        if cur.get("resp_time"):
            set_resp_time(link, dev_id, cur["resp_time"]["resptime_100us"])
        if cur.get("rated_speed"):
            set_rated_speed(link, dev_id, cur["rated_speed"]["rated_rpm"])
        if cur.get("resolution"):
            set_resolution(link, dev_id, cur["resolution"]["resolution"])
        if cur.get("ratio"):
            set_ratio(link, dev_id, cur["ratio"]["ratio"])
        if cur.get("pos_ctrl_mode"):
            set_pos_mode(link, dev_id, cur["pos_ctrl_mode"]["pos_mode"] == "absolute")
        if cur.get("ctrl_dir"):
            set_ctrl_dir(link, dev_id, cw=(cur["ctrl_dir"]["ctrl_dir"] == "CW"))
        if cur.get("ctrl_onoff"):
            set_onoff(link, dev_id, on=cur["ctrl_onoff"]["on"])

        time.sleep(0.05)

        # 3) 제어 데모: 각 1초 구동 후 정지
        print("\n[DEMO] 제어 On")
        set_onoff(link, dev_id, on=True)           # 제어 On (0x00)
        time.sleep(0.05)

        # (a) 위치, 속도제어 1s
        print("[RUN] 위치, 속도제어 (0x01) -> 1s")
        # 예: CCW로 180.00도, 5.0RPM
        cmd_pos_speed(link, dev_id, direction_cw=False, pos_deg=180.00, spd_rpm=5.0)  # MC 스펙: dir/pos(0.01°)/spd(0.1RPM) :contentReference[oaicite:3]{index=3}
        time.sleep(1.0)
        safe_stop(link, dev_id)
        print("[STOP] done\n")
        time.sleep(0.2)
        set_onoff(link, dev_id, on=True); time.sleep(0.05)

        # (b) 가감속 위치제어 1s
        print("[RUN] 가감속 위치제어 (0x02) -> 1s")
        # 예: CW로 360.00도, 도달시간 1.0s
        cmd_acc_pos(link, dev_id, direction_cw=True, pos_deg=360.00, arrive_s=1.0)    # MC 스펙: dir/pos/arrive(0.1s) :contentReference[oaicite:4]{index=4}
        time.sleep(1.0)
        safe_stop(link, dev_id)
        print("[STOP] done\n")
        time.sleep(0.2)
        set_onoff(link, dev_id, on=True); time.sleep(0.05)

        # (c) 가감속 속도제어 1s
        print("[RUN] 가감속 속도제어 (0x03) -> 1s")
        # 예: CCW로 100RPM, 도달시간 0.5s
        cmd_acc_speed(link, dev_id, direction_cw=False, spd_rpm=100.0, arrive_s=0.5)  # MC 스펙: dir/spd/arrive(0.1s) :contentReference[oaicite:5]{index=5}
        time.sleep(1.0)
        safe_stop(link, dev_id)
        print("[STOP] done\n")

        set_onoff(link, dev_id, on=True); time.sleep(0.05)
        print("[RUN] Open-Loop (0x11) -> 1s")
        # 예: CCW 50% duty
        cmd_open_loop(link, dev_id, direction_cw=False, duty_percent=50.0)        # SA 스펙: dir/duty(0.01%) :contentReference[oaicite:6]{index=6}
        time.sleep(1.0)
        safe_stop(link, dev_id)
        print("[STOP] done\n")

        print("[DEMO] 완료")

    finally:
        link.close()


def main():
    ap = argparse.ArgumentParser(description="Nurirobot RS-485 MC/SA control demo")
    ap.add_argument("--series", choices=["MC", "SA"], default="MC", help="제품 시리즈 (MC 기본, SA는 Open-Loop 지원)")
    ap.add_argument("--port", default="COM10",help="시리얼 포트 (예: /dev/ttyUSB0, COM5)")
    ap.add_argument("--id", type=lambda x: int(x, 0), default=0x00, help="장치 ID (기본 0x00)")
    ap.add_argument("--baud", type=int, default=115200, help="Baudrate (기본 9600)")
    ap.add_argument("--rts485", action="store_true", help="어댑터가 RTS 기반 DE/RE 전환 필요 시 사용")
    args = ap.parse_args()

    demo(args.series, args.port, args.id, args.baud, args.rts485)


if __name__ == "__main__":
    main()
