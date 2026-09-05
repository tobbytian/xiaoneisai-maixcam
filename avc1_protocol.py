"""
AVC1 车体速度帧 — 纯 Python（PC + MaixCAM 共用）。

契约来源（C 板 feat/ps2-auto-vision）：
  https://github.com/shiuhou/pnx_newtemplate_F4/blob/feat/ps2-auto-vision/docs/vision-auto-chassis-interface.md
  https://github.com/shiuhou/pnx_newtemplate_F4/blob/feat/ps2-auto-vision/docs/vision-auto-chassis-quick-start.md

接线（V1 单向）：
  MaixCAM UART1 TX (A19) -> C 板 USART6_RX (PG9)，共地 GND
  115200 8N1，3.3V TTL；C 板 TX 可不接（无 ACK）

帧：固定 24 字节小端，20–50 Hz 持续发送；seq 每帧递增（含 valid=0）
坐标：+vx 车头向前，+vy 车体向左（m/s）；无 wz，C 板 auto 固定 wz=0
C 板：合法新鲜帧 + 已解锁进 auto 且按住 L1 才执行；>200ms 无新帧或 valid=0 -> 停车
"""
import struct

AVC1_MAGIC = 0x31435641  # 线上: 41 56 43 31
AVC1_FRAME_SIZE = 24
AVC1_CHECKSUM_SEED = 0xA5A51234
PROTOCOL_MAX_MPS = 0.8
# 契约建议持续发送 20–50 Hz（C 板 200 ms 超时）
DEFAULT_SEND_HZ = 30

# 接口文档黄金向量（完整 24 字节 hex）
GOLDEN_CASES = (
    # seq, vx, vy, valid, expected_hex
    (1, 0.5, -0.25, 1, "41564331010000000000003f000080be010000002eb88888"),
    (2, 0.0, 0.0, 0, "4156433102000000000000000000000000000000deb9eb44"),
    (0, -0.8, 0.8, 1, "4156433100000000cdcc4cbfcdcc4c3f01000000ed6d7100"),
)


def clamp_axis(v, limit=PROTOCOL_MAX_MPS):
    v = float(v)
    if v > limit:
        return limit
    if v < -limit:
        return -limit
    return v


def avc1_checksum(data):
    """对前 20 字节 payload 校验；种子 0xA5A51234。"""
    value = AVC1_CHECKSUM_SEED
    for byte in data:
        value = ((value << 5) & 0xFFFFFFFF) ^ (value >> 2) ^ byte
    return value & 0xFFFFFFFF


def pack_avc1(seq, vx_mps, vy_mps, valid, max_mps=PROTOCOL_MAX_MPS):
    """
    打一帧 AVC1。

    小端布局: magic u32 | seq u32 | vx f32 | vy f32 | valid u8 | pad 3x | csum u32
    +x 向前，+y 向左（m/s）。valid 只能是 0 或 1。
    """
    vx = clamp_axis(vx_mps, max_mps)
    vy = clamp_axis(vy_mps, max_mps)
    vflag = 1 if valid else 0
    payload = struct.pack(
        "<IIffB3x",
        AVC1_MAGIC,
        seq & 0xFFFFFFFF,
        vx,
        vy,
        vflag,
    )
    if len(payload) != 20:
        raise RuntimeError("AVC1 payload 长度异常")
    return payload + struct.pack("<I", avc1_checksum(payload))


def unpack_avc1(frame):
    """
    解析/校验 24 字节帧。
    返回 (seq, vx, vy, valid, ok)；magic/pad/checksum/尺寸不对则 ok=False。
    """
    if frame is None or len(frame) != AVC1_FRAME_SIZE:
        return 0, 0.0, 0.0, 0, False
    magic, seq, vx, vy, valid = struct.unpack_from("<IIffB", frame, 0)
    pad = frame[17:20]
    csum_got = struct.unpack_from("<I", frame, 20)[0]
    csum_exp = avc1_checksum(frame[:20])
    ok = (
        magic == AVC1_MAGIC
        and pad == b"\x00\x00\x00"
        and valid in (0, 1)
        and csum_got == csum_exp
    )
    return seq, float(vx), float(vy), int(valid), ok


def selftest():
    """打包对齐黄金向量且能 round-trip 则返回 True。"""
    ok = True
    for seq, vx, vy, valid, exp_hex in GOLDEN_CASES:
        got = pack_avc1(seq, vx, vy, valid)
        exp = bytes.fromhex(exp_hex)
        if got != exp or len(got) != AVC1_FRAME_SIZE:
            ok = False
            print("FAIL pack seq=%s" % seq)
            print("  got", got.hex())
            print("  exp", exp_hex)
            continue
        u_seq, u_vx, u_vy, u_valid, u_ok = unpack_avc1(got)
        if not u_ok or u_seq != (seq & 0xFFFFFFFF) or u_valid != valid:
            ok = False
            print("FAIL unpack seq=%s" % seq)
            continue
        if abs(u_vx - vx) > 1e-6 or abs(u_vy - vy) > 1e-6:
            ok = False
            print("FAIL values seq=%s %s,%s" % (seq, u_vx, u_vy))
            continue
        print("OK  seq=%s vx=%s vy=%s valid=%s len=%s" % (seq, vx, vy, valid, len(got)))
    return ok


if __name__ == "__main__":
    raise SystemExit(0 if selftest() else 1)
