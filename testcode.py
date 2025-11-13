#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import logging
from dataclasses import dataclass
from typing import Optional, List, Tuple, Union

import serial

try:
    # 일부 어댑터는 자동 DE/RE, 일부는 수동 필요
    from serial.rs485 import RS485, RS485Settings
    RS485_AVAILABLE = True
except Exception:
    RS485_AVAILABLE = False


# -----------------------------
# 프로토콜 상수 (MC-RS485 V1.0.1)
# -----------------------------
HEADER = b'\xFF\xFE'  # 0xFF 0xFE
BROADCAST_ID = 0xFF
DEFAULT_ID = 0x00

# 요청(송신) 모드 -> 응답(수신) 모드 매핑
REQ = {
    "ping":           0xA0,
    "pos":            0xA1,  # 위치 피드백 요청
    "spd":            0xA2,  # 속도 피드백 요청
    "pos_ctrl":       0xA3,  # 위치제어기 피드백 요청
    "spd_ctrl":       0xA4,  # 속도제어기 피드백 요청
    "resp_time":      0xA5,  # 통신 응답시간 피드백 요청
    "rated_speed":    0xA6,  # 모터 정격속도 피드백 요청
    "resolution":     0xA7,  # 분해능 피드백 요청
    "ratio":          0xA8,  # 감속비 피드백 요청
    "ctrl_onoff":     0xA9,  # 제어 On/Off 피드백 요청
    "pos_ctrl_mode":  0xAA,  # 위치제어 모드 피드백 요청
    "ctrl_dir":       0xAB,  # 제어 방향 피드백 요청
    "firmware":       0xCD,  # 펌웨어 버전 피드백 요청
}
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

# 방향/상태 상수 (MC 문서)
DIR_CCW = 0x00
DIR_CW  = 0x01
ON      = 0x00
OFF     = 0x01
POS_ABS = 0x00
POS_REL = 0x01


# -----------------------------
# 유틸리티
# -----------------------------
def u16_be(hi: int, lo: int) -> int:
    """Big-Endian 16-bit -> int"""
    return ((hi & 0xFF) << 8) | (lo & 0xFF)

def checksum_for_frame(frame_wo_checksum: bytes) -> int:
    """
    체크섬: Header(2)와 Checksum 바이트(자기 자신)를 제외한 모든 바이트 합에 bitwise not.
    문서/예제 및 레퍼런스 코드 동일 규칙.  
    """
    # frame 구조: [0]=0xFF, [1]=0xFE, [2]=ID, [3]=Size, [4]=Checksum(placeholder), [5]=Mode, [6:]=Data...
    total = 0
    for i, b in enumerate(frame_wo_checksum):
        if i in (0, 1, 4):  # header 2B + checksum 1B 제외
            continue
        total = (total + (b & 0xFF)) & 0xFF
    return (~total) & 0xFF

def build_request(id_: int, mode: int, data: bytes = b'') -> bytes:
    """
    요청 프레임 생성.
    - 총길이 = size + 4
    - size = 2 + len(data)  (Mode 1B + Data NB = size-1, 체계적으로 계산하면 len(frame)-4)
    """
    size = 2 + len(data)  # (Mode + Data)
    buf = bytearray()
    buf += HEADER
    buf.append(id_ & 0xFF)
    buf.append(size & 0xFF)
    buf.append(0x00)          # checksum placeholder
    buf.append(mode & 0xFF)   # mode
    buf += data               # data
    cs = checksum_for_frame(buf)
    buf[4] = cs
    return bytes(buf)


# -----------------------------
# 수신 프레임 파서 (스트림 안전)
# -----------------------------
class StreamParser:
    """
    RS-485 수신 스트림에서 프레임 단위로 분리/검증.
    - 헤더 0xFFFE 동기화
    - Data 크기 기반으로 완전성 판단(총길이=size+4)
    - 중간에 새로운 헤더가 나타나면 이전 미완 전문 폐기  :contentReference[oaicite:4]{index=4}
    """
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk: bytes) -> List[bytes]:
        self.buf += chunk
        frames: List[bytes] = []

        while True:
            # 1) 헤더 탐색
            i = self.buf.find(HEADER)
            if i < 0:
                # 헤더 못 찾으면 마지막 1바이트만 남김(0xFF 가능성)
                self.buf = self.buf[-1:] if len(self.buf) > 0 else bytearray()
                break

            # 헤더 이전 노이즈 폐기
            if i > 0:
                del self.buf[:i]

            # 최소 길이 체크
            if len(self.buf) < 6:  # FF FE ID SIZE CS MODE
                break

            # 2) 길이 계산
            size = self.buf[3]
            total_len = size + 4  # 문서 규칙
            # 3) 완전성 체크
            if len(self.buf) < total_len:
                # 혹시 중간에 새로운 헤더가 끼어들었는지 확인 -> 끼어들면 이전 미완 프레임 폐기
                j = self.buf.find(HEADER, 2)  # 다음 헤더
                if 0 <= j < total_len:
                    del self.buf[:j]  # 앞부분 버리고 다음 헤더부터 다시
                    continue
                break  # 더 읽어야 함

            # 4) 프레임 추출
            frame = bytes(self.buf[:total_len])
            del self.buf[:total_len]

            # 5) 헤더/길이/체크섬 확인
            if frame[:2] != HEADER or (frame[3] + 4) != len(frame):
                continue  # 불량

            # 디버그: 수신된 프레임을 16진수 바이트로 출력합니다.
            try:
                print("Recv Frame:", ' '.join(f"{b:02X}" for b in frame))
            except Exception:
                # 출력 도중 문제가 생겨도 파싱에는 영향이 없도록 무시합니다.
                pass
              
            cs = checksum_for_frame(frame)
            if frame[4] != cs:
                continue  # 체크섬 불일치 -> 폐기


            frames.append(frame)

        return frames


# -----------------------------
# 응답 데이터 구조체
# -----------------------------
@dataclass
class Ping:
    id: int

@dataclass
class PositionFeedback:
    id: int
    direction: int       # 0:CCW, 1:CW
    position_deg: float  # 0.01°
    speed_rpm: float     # 0.1 RPM
    current_a: float     # 100 mA

@dataclass
class SpeedFeedback:
    id: int
    direction: int       # 0:CCW, 1:CW
    speed_rpm: float     # 0.1 RPM
    position_deg: float  # 0.1°  (MC 예제 기준)  :contentReference[oaicite:5]{index=5}
    current_a: float     # 100 mA

@dataclass
class PosCtrlFeedback:
    id: int
    kp: int
    ki: int
    kd: int
    rated_current_a: float  # [100 mA]

@dataclass
class SpdCtrlFeedback:
    id: int
    kp: int
    ki: int
    kd: int
    rated_current_a: float  # [100 mA]

@dataclass
class RespTimeFeedback:
    id: int
    resp_time_us: int  # [100 us] 단위

@dataclass
class RatedSpeedFeedback:
    id: int
    rated_rpm: int     # [RPM]

@dataclass
class ResolutionFeedback:
    id: int
    resolution: int    # [encoder pulse or pole]

@dataclass
class RatioFeedback:
    id: int
    ratio: float       # [0.1 ratio]

@dataclass
class CtrlOnOffFeedback:
    id: int
    onoff: int         # 0:On, 1:Off

@dataclass
class PosCtrlModeFeedback:
    id: int
    mode: int          # 0:절대, 1:상대

@dataclass
class CtrlDirFeedback:
    id: int
    direction: int     # 0:CCW, 1:CW

@dataclass
class FirmwareFeedback:
    id: int
    version: int


# -----------------------------
# 프레임 -> 구조체 파싱
# -----------------------------
def parse_frame(frame: bytes) -> Optional[Union[
    Ping, PositionFeedback, SpeedFeedback, PosCtrlFeedback, SpdCtrlFeedback,
    RespTimeFeedback, RatedSpeedFeedback, ResolutionFeedback, RatioFeedback,
    CtrlOnOffFeedback, PosCtrlModeFeedback, CtrlDirFeedback, FirmwareFeedback
]]:
    # frame: FF FE ID SIZE CS MODE [DATA...]
    id_  = frame[2]
    mode = frame[5]
    data = frame[6:]

    if mode == FEED["ping"]:
        return Ping(id_)

    elif mode == FEED["pos"]:
        # [dir][pos_hi][pos_lo][spd_hi][spd_lo][curr]
        if len(data) >= 6:
            direction = data[0]
            pos = u16_be(data[1], data[2]) * 0.01   # 0.01°
            spd = u16_be(data[3], data[4]) * 0.1    # 0.1 RPM
            cur = (data[5] & 0xFF) * 0.1            # 100 mA
            return PositionFeedback(id_, direction, pos, spd, cur)

    elif mode == FEED["spd"]:
        # [dir][spd_hi][spd_lo][pos_hi][pos_lo][curr]
        if len(data) >= 6:
            direction = data[0]
            spd = u16_be(data[1], data[2]) * 0.1    # 0.1 RPM
            pos = u16_be(data[3], data[4]) * 0.1    # 0.1° (MC 예제)  :contentReference[oaicite:6]{index=6}
            cur = (data[5] & 0xFF) * 0.1            # 100 mA
            return SpeedFeedback(id_, direction, spd, pos, cur)

    elif mode == FEED["pos_ctrl"]:
        # [Kp][Ki][Kd][RatedCurrent]
        if len(data) >= 4:
            kp, ki, kd, rc = data[0], data[1], data[2], data[3]
            return PosCtrlFeedback(id_, kp, ki, kd, rc * 0.1)

    elif mode == FEED["spd_ctrl"]:
        if len(data) >= 4:
            kp, ki, kd, rc = data[0], data[1], data[2], data[3]
            return SpdCtrlFeedback(id_, kp, ki, kd, rc * 0.1)

    elif mode == FEED["resp_time"]:
        if len(data) >= 1:
            return RespTimeFeedback(id_, (data[0] & 0xFF) * 100)  # 100us 단위

    elif mode == FEED["rated_speed"]:
        if len(data) >= 2:
            return RatedSpeedFeedback(id_, u16_be(data[0], data[1]))  # RPM

    elif mode == FEED["resolution"]:
        if len(data) >= 2:
            return ResolutionFeedback(id_, u16_be(data[0], data[1]))

    elif mode == FEED["ratio"]:
        if len(data) >= 2:
            return RatioFeedback(id_, u16_be(data[0], data[1]) / 10.0)  # 0.1 ratio

    elif mode == FEED["ctrl_onoff"]:
        if len(data) >= 1:
            return CtrlOnOffFeedback(id_, data[0])

    elif mode == FEED["pos_ctrl_mode"]:
        if len(data) >= 1:
            return PosCtrlModeFeedback(id_, data[0])

    elif mode == FEED["ctrl_dir"]:
        if len(data) >= 1:
            return CtrlDirFeedback(id_, data[0])

    elif mode == FEED["firmware"]:
        if len(data) >= 1:
            return FirmwareFeedback(id_, data[0])

    return None


# -----------------------------
# 직렬 포트 래퍼
# -----------------------------
class MCSerial:
    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 0.2):
        """
        - 기본 8N1, None 파리티, HW flow off
        - RS485 모드가 되면 RTS로 DE 제어 (가능한 어댑터일 때)  :contentReference[oaicite:7]{index=7}
        """
        if RS485_AVAILABLE:
            self.ser = RS485(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
            )
            # RS-485 DE/RE 설정 (필요시 조정)
            self.ser.rs485_mode = RS485Settings(
                rts_level_for_tx=True,   # 전송 시 DE=High
                rts_level_for_rx=False,  # 수신 시 DE=Low
                loopback=False,
                delay_before_tx=0.0,
                delay_before_rx=0.0,
            )
        else:
            # 많은 USB-485 어댑터는 자동으로 DE/RE 처리: 일반 Serial로도 동작
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

        self.parser = StreamParser()

    def write(self, data: bytes):
        self.ser.write(data)
        self.ser.flush()

    def read_frames(self) -> List[bytes]:
        frames: List[bytes] = []
        # 한번에 너무 큰 블록을 기다리지 않고, timeout 내에서 여러 번 읽어 합칩니다.
        # (제품 제어 문서 권장: 타임아웃 설계 중요)  :contentReference[oaicite:8]{index=8}
        start = time.time()
        while (time.time() - start) < (self.ser.timeout or 0.2):
            n = self.ser.in_waiting if hasattr(self.ser, "in_waiting") else 0
            if n <= 0:
                time.sleep(0.005)
                continue
            chunk = self.ser.read(n)
            frames.extend(self.parser.feed(chunk))
        return frames

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


# -----------------------------
# 요청/응답 헬퍼
# -----------------------------
def request_and_wait(
    mc: MCSerial,
    id_: int,
    req_mode: int,
    expect_feed_mode: int,
    wait_s: float = 0.1,
    timeout_s: float = 0.5
) -> Optional[Tuple[bytes, object]]:
    """
    단일 요청 전송 -> 해당 ID/모드의 응답 1개 확보
    """
    frame = build_request(id_, req_mode)
    mc.write(frame)
    try:
        print("Send Frame:", ' '.join(f"{b:02X}" for b in frame))
    except Exception:
        # 출력 도중 문제가 생겨도 파싱에는 영향이 없도록 무시합니다.
        pass    
    # 약간의 처리 여유
    time.sleep(wait_s)

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        frames = mc.read_frames()
        for f in frames:
            if f[2] != (id_ & 0xFF):   # ID 필터
                continue
            if f[5] != (expect_feed_mode & 0xFF):
                continue
            parsed = parse_frame(f)
            if parsed is not None:
                return f, parsed
        time.sleep(0.01)
    return None


def test_all_requests(
    port: str,
    id_: int = DEFAULT_ID,
    baudrate: int = 9600,
    per_req_wait_s: float = 0.05,
    per_req_timeout_s: float = 0.5,
):
    """
    MC 모듈에 대해 모든 '피드백 요청(송신)'을 테스트하고 파싱 결과를 출력.
    """
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    log = logging.getLogger("mc")

    mc = MCSerial(port=port, baudrate=baudrate, timeout=per_req_timeout_s)
    try:
        plan = [
            ("ping",          REQ["ping"],          FEED["ping"]),
            ("pos",           REQ["pos"],           FEED["pos"]),
            ("spd",           REQ["spd"],           FEED["spd"]),
            ("pos_ctrl",      REQ["pos_ctrl"],      FEED["pos_ctrl"]),
            ("spd_ctrl",      REQ["spd_ctrl"],      FEED["spd_ctrl"]),
            ("resp_time",     REQ["resp_time"],     FEED["resp_time"]),
            ("rated_speed",   REQ["rated_speed"],   FEED["rated_speed"]),
            ("resolution",    REQ["resolution"],    FEED["resolution"]),
            ("ratio",         REQ["ratio"],         FEED["ratio"]),
            ("ctrl_onoff",    REQ["ctrl_onoff"],    FEED["ctrl_onoff"]),
            ("pos_ctrl_mode", REQ["pos_ctrl_mode"], FEED["pos_ctrl_mode"]),
            ("ctrl_dir",      REQ["ctrl_dir"],      FEED["ctrl_dir"]),
            ("firmware",      REQ["firmware"],      FEED["firmware"]),
        ]

        for name, req_mode, feed_mode in plan:
            log.info(f"\n==> Request: {name} (ID={id_:#04x})")
            got = request_and_wait(
                mc, id_=id_, req_mode=req_mode, expect_feed_mode=feed_mode,
                wait_s=per_req_wait_s, timeout_s=per_req_timeout_s
            )
            if not got:
                log.warning(f"   [Timeout] {name} 응답 없음")
                continue
            raw, parsed = got
            log.info(f"   Raw: {' '.join(f'{b:02X}' for b in raw)}")
            log.info(f"   Parsed: {parsed}")

            # 버스 충돌/지연 가능성 -> 약간의 간격을 둔다.  :contentReference[oaicite:9]{index=9}
            time.sleep(0.03)

    finally:
        mc.close()


# -----------------------------
# 실행 예시
# -----------------------------
if __name__ == "__main__":
    # 환경에 맞게 수정:
    # - Linux:  '/dev/ttyUSB0'
    # - Windows:'COM3'
    PORT = "COM10"
    BAUD = 115200  # 장비 설정과 일치시켜야 함 (문서상 초기값 9600bps 예시 있음).  :contentReference[oaicite:10]{index=10}
    DEV_ID = 0x00

    test_all_requests(port=PORT, id_=DEV_ID, baudrate=BAUD)
