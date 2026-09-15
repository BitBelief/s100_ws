import struct

from wheeltec_s100_driver.protocol import (
    ACCEL_RATIO, AUX_HEAD, AUX_TAIL, FrameParser, GRAVITY, GYRO_RATIO,
    HEAD, TAIL, encode_cmd, xor)


def make_frame(vx=0.0, wz=0.0, az_g=1.0, volt=25.40):
    b = bytearray([HEAD, 0])
    for v in (vx, 0.0, wz):
        b += struct.pack('>h', int(round(v * 1000)))
    for a in (0.0, 0.0, az_g):
        b += struct.pack('>h', int(round(a * ACCEL_RATIO)))
    for g in (0.0, 0.0, wz):
        b += struct.pack('>h', int(round(g / GYRO_RATIO)))
    b += struct.pack('>H', int(round(volt * 1000)))
    b.append(xor(b, 22))
    b.append(TAIL)
    return bytes(b)


def test_encode_cmd_matches_measured_protocol():
    # 上週實機送的就是這個：vx=0.100 m/s → 100 mm/s，short 大端
    f = encode_cmd(0.1, 0.0, 0.0)
    assert len(f) == 11
    assert f[0] == HEAD and f[10] == TAIL
    assert f[3:5] == struct.pack('>h', 100)
    assert f[9] == xor(f, 9)


def test_encode_cmd_negative_and_angular():
    f = encode_cmd(-0.2, 0.0, 0.8)
    assert struct.unpack('>h', f[3:5])[0] == -200
    assert struct.unpack('>h', f[7:9])[0] == 800


def test_decode_units():
    st = FrameParser().feed(make_frame(vx=0.15, wz=-0.5))[0]
    assert abs(st.vx - 0.15) < 1e-3
    assert abs(st.wz + 0.5) < 1e-3
    assert abs(st.az - GRAVITY) < 1e-2      # 靜止時 Z 軸 = 重力
    assert abs(st.gz + 0.5) < 1e-3
    assert abs(st.voltage - 25.40) < 1e-3


def test_parser_skips_aux_frames_and_garbage():
    p = FrameParser()
    aux = bytes([AUX_HEAD]) + bytes(6) + bytes([AUX_TAIL])
    stream = b'\xff\xfe' + make_frame(vx=0.3) + aux + make_frame(vx=0.4)
    out = p.feed(stream)
    assert len(out) == 2
    assert abs(out[0].vx - 0.3) < 1e-3 and abs(out[1].vx - 0.4) < 1e-3
    assert p.aux == 1 and p.dropped == 2 and p.bad == 0


def test_parser_handles_split_reads():
    p = FrameParser()
    f = make_frame(vx=0.25)
    assert p.feed(f[:7]) == []
    out = p.feed(f[7:])
    assert len(out) == 1 and abs(out[0].vx - 0.25) < 1e-3


def test_parser_rejects_bad_checksum():
    p = FrameParser()
    f = bytearray(make_frame(vx=0.2))
    f[22] ^= 0xFF
    assert p.feed(bytes(f)) == []
    assert p.bad == 1
