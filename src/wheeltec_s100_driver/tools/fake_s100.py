#!/usr/bin/env python3
"""假底盤：開一個 TCP 埠，照實機協定回 24-byte 幀，用來在車沒接上時驗證驅動。

行為刻意複製實機量到的幾件事：
  * 回傳 20 Hz
  * 速度一階延遲，約 0.2 秒收斂
  * STM32 看門狗：1.0 秒沒收到指令就歸零
  * 靜止時 az = +1.00 g、電池 25.40 V

用法：
    python3 fake_s100.py [port]          # 預設 5757
    ros2 run wheeltec_s100_driver s100_driver --ros-args \
        -p port:=socket://127.0.0.1:5757
（pyserial 的 socket:// URL handler 讓驅動不必改一行程式就能接上假底盤。）
"""
import math
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from wheeltec_s100_driver.protocol import (  # noqa: E402
    ACCEL_RATIO, GYRO_RATIO, HEAD, TAIL, xor)

WATCHDOG = 1.0
TAU = 0.07          # 一階時間常數，約 3τ ≈ 0.2s 收斂
PERIOD = 0.05       # 20 Hz


def build_frame(vx, vy, wz, volt=25.40):
    b = bytearray([HEAD, 0])
    for v in (vx, vy, wz):
        b += struct.pack('>h', int(round(v * 1000)))
    for a in (0.0, 0.0, 1.0):                       # 加速度 (g)，靜止時 Z=+1.00
        b += struct.pack('>h', int(round(a * ACCEL_RATIO)))
    for g in (0.0, 0.0, wz):                        # 陀螺 (rad/s)
        b += struct.pack('>h', int(round(g / GYRO_RATIO)))
    b += struct.pack('>H', int(round(volt * 1000)))
    b.append(xor(b, 22))
    b.append(TAIL)
    return bytes(b)


def serve(conn):
    cmd_vx = cmd_wz = vx = wz = 0.0
    last_cmd = t_prev = time.monotonic()
    buf = bytearray()
    n = 0
    conn.setblocking(False)

    while True:
        time.sleep(PERIOD)
        try:
            chunk = conn.recv(1024)
            if not chunk:
                print('  [假底盤] 連線關閉')
                return
            buf += chunk
        except BlockingIOError:
            pass

        while len(buf) >= 11:
            if buf[0] != HEAD:
                buf.pop(0)
                continue
            f = bytes(buf[:11])
            if f[10] != TAIL or f[9] != xor(f, 9):
                buf.pop(0)
                continue
            del buf[:11]
            cmd_vx = struct.unpack('>h', f[3:5])[0] / 1000.0
            cmd_wz = struct.unpack('>h', f[7:9])[0] / 1000.0
            last_cmd = time.monotonic()

        now = time.monotonic()
        dt, t_prev = now - t_prev, now
        barking = now - last_cmd > WATCHDOG
        tgt_vx, tgt_wz = (0.0, 0.0) if barking else (cmd_vx, cmd_wz)
        a = 1.0 - math.exp(-dt / TAU)
        vx += (tgt_vx - vx) * a
        wz += (tgt_wz - wz) * a

        try:
            conn.sendall(build_frame(vx, 0.0, wz))
        except (BrokenPipeError, ConnectionResetError, OSError):
            print('  [假底盤] 連線中斷')
            return

        n += 1
        if n % 20 == 0:
            print(f'  [假底盤] 指令 vx={cmd_vx:+.3f} wz={cmd_wz:+.3f} | '
                  f'實測 vx={vx:+.3f} wz={wz:+.3f}'
                  f'{"  <看門狗歸零>" if barking else ""}', flush=True)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 5757
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', port))
    srv.listen(1)
    print(f'假底盤已就緒：socket://127.0.0.1:{port}', flush=True)
    try:
        while True:
            conn, _ = srv.accept()
            print('  [假底盤] 驅動已連上', flush=True)
            with conn:
                serve(conn)
    except KeyboardInterrupt:
        print('\n假底盤結束')
    finally:
        srv.close()


if __name__ == '__main__':
    main()
