"""WHEELTEC S100 底盤序列協定 — 純函式，不碰 ROS 也不碰硬體，可單獨測。

協定內容是 2026-09-09 在實機上實讀驗證過的（5 秒 100 幀 / 0 校驗失敗、
加速度 Z=+1.00 g、電池 25.40 V）。
"""
import struct
from dataclasses import dataclass

HEAD, TAIL = 0x7B, 0x7D
FRAME_LEN = 24
CMD_LEN = 11

# 次要幀：8 bytes，頭 0x7C 尾 0x7F，XOR 同規則。目前內容全零、用途未知，
# 解析時必須認得它才不會把它當成垃圾一個 byte 一個 byte 丟掉。
AUX_HEAD, AUX_TAIL = 0x7C, 0x7F
AUX_LEN = 8

GYRO_RATIO = 0.00026644    # 1/65.5/57.3，MPU6050 FS_SEL=1 (±500 deg/s) → rad/s
ACCEL_RATIO = 16384.0      # ±2g → g
GRAVITY = 9.80665


def xor(buf, n):
    c = 0
    for b in buf[:n]:
        c ^= b
    return c


def _s16(hi, lo):
    return struct.unpack('>h', bytes([hi, lo]))[0]


def encode_cmd(vx, vy, wz, model=0, enable=0):
    """組 11-byte 指令幀。速度是 short 大端、原值×1000（m/s→mm/s、rad/s→mrad/s）。"""
    b = bytearray([HEAD, model & 0xFF, enable & 0xFF])
    for v in (vx, vy, wz):
        b += struct.pack('>h', max(-32768, min(32767, int(round(v * 1000)))))
    b.append(xor(b, 9))
    b.append(TAIL)
    return bytes(b)


@dataclass
class ChassisState:
    """一幀回傳，已換算成 SI 單位。"""
    stop_flag: int
    vx: float       # m/s
    vy: float       # m/s（差速車恆為 0）
    wz: float       # rad/s
    ax: float       # m/s^2
    ay: float
    az: float
    gx: float       # rad/s
    gy: float
    gz: float
    voltage: float  # V


def decode_frame(f):
    """解 24-byte 幀（呼叫端須先確認頭尾與校驗）。"""
    return ChassisState(
        stop_flag=f[1],
        vx=_s16(f[2], f[3]) / 1000.0,
        vy=_s16(f[4], f[5]) / 1000.0,
        wz=_s16(f[6], f[7]) / 1000.0,
        ax=_s16(f[8], f[9]) / ACCEL_RATIO * GRAVITY,
        ay=_s16(f[10], f[11]) / ACCEL_RATIO * GRAVITY,
        az=_s16(f[12], f[13]) / ACCEL_RATIO * GRAVITY,
        gx=_s16(f[14], f[15]) * GYRO_RATIO,
        gy=_s16(f[16], f[17]) * GYRO_RATIO,
        gz=_s16(f[18], f[19]) * GYRO_RATIO,
        voltage=((f[20] << 8) | f[21]) / 1000.0,
    )


class FrameParser:
    """位元組流 → ChassisState。認得 0x7B/0x7D 主幀與 0x7C/0x7F 次幀。"""

    def __init__(self):
        self.buf = bytearray()
        self.ok = 0        # 通過校驗的主幀
        self.bad = 0       # 頭尾對但 XOR 不對
        self.aux = 0       # 次要幀
        self.dropped = 0   # 丟掉的孤兒 byte

    def feed(self, data):
        self.buf += data
        out = []
        while self.buf:
            h = self.buf[0]
            if h == HEAD:
                if len(self.buf) < FRAME_LEN:
                    break
                f = bytes(self.buf[:FRAME_LEN])
                if f[FRAME_LEN - 1] != TAIL:
                    self.buf.pop(0)
                    self.dropped += 1
                    continue
                del self.buf[:FRAME_LEN]
                if f[FRAME_LEN - 2] != xor(f, FRAME_LEN - 2):
                    self.bad += 1
                    continue
                self.ok += 1
                out.append(decode_frame(f))
            elif h == AUX_HEAD:
                if len(self.buf) < AUX_LEN:
                    break
                if self.buf[AUX_LEN - 1] != AUX_TAIL:
                    self.buf.pop(0)
                    self.dropped += 1
                    continue
                del self.buf[:AUX_LEN]
                self.aux += 1
            else:
                self.buf.pop(0)
                self.dropped += 1
        return out
