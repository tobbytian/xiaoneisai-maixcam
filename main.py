"""
识别核心：装甲板模板匹配 + 测距/中心 + AVC1 跟随（MaixCAM）

链路：
  相机 → matchTemplate（ROI/尺度）→ 像素中心 + 针孔测距
       → 车体 vx/vy → UART1 AVC1 → 屏上框选

装甲板实物：8 cm × 8 cm。
距离（针孔）：Z = f_px * W_real / w_px
  横向位置：X = (cx - cx0) * Z / f_px   （相机右为正）
控制目标：把装甲板保持在画面中心，并维持目标距离 Z_TARGET_M。
  AVC1：+vx 车头前，+vy 车体左。

接线：A19/UART1_TX → C 板 PG9，GND 共地；115200；20–50 Hz；seq 每帧 +1
板端需：main.py、avc1_protocol.py、target1.jpg 同目录

参考：
  https://github.com/shiuhou/pnx_newtemplate_F4/tree/feat/ps2-auto-vision
  https://wiki.sipeed.com/maixpy/doc/zh/vision/opencv.html
  https://wiki.sipeed.com/maixpy/doc/zh/peripheral/uart.html
"""
import os
from maix import camera, display, image, time, app, uart, pinmap, err
import cv2

from avc1_protocol import (
    DEFAULT_SEND_HZ,
    PROTOCOL_MAX_MPS,
    pack_avc1,
)

# ======== 串口 / 协议 ========
BAUD_RATE = 115200
SERIAL_DEVICE = "/dev/ttyS1"
PIN_TX = "A19"
PIN_RX = "A18"
MAX_SPEED = PROTOCOL_MAX_MPS
SEND_HZ = DEFAULT_SEND_HZ
FRAME_MS = max(1, int(1000 / SEND_HZ))
SEND_UART = True

# ======== 匹配 ========
CAM_W, CAM_H = 320, 240
MATCH_W, MATCH_H = 40, 30

TEMPLATE_CANDIDATES = (
    "/root/target1.jpg",
    "target1.jpg",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "target1.jpg"),
)

MATCH_MIN_SCORE = 0.62
SCALE_MIN = 0.3
SCALE_MAX = 0.9
SCALE_STEP = 0.05
TEMPLATE_BASE_MAX_SIDE = 96

# ======== 实物尺寸 + 针孔测距（320×240 像素焦距）=======
# 装甲板外接正方形边长（米）
ARMOR_W_M = 0.08
ARMOR_H_M = 0.08
FOCAL_PX = 277.0
# 希望停稳的距离（米）
Z_TARGET_M = 0.2
# 米制死区
X_DEAD_M = 0.02
Z_DEAD_M = 0.04
# 速度增益：m/s per m 误差
KP_LAT = 1.2   # 横向（消 X）
KP_RANGE = 0.9  # 前后（消 Z-Z_TARGET）
SMOOTH_A = 0.5

# ======== 跟踪加速 ========
ROI_EXPAND = 1.8
MAX_TRACK_LOST = 3
FULL_SCAN_INTERVAL = 8
SCALE_NEIGHBOR = 2
SCORE_PRUNE = MATCH_MIN_SCORE * 0.8

SX = float(CAM_W) / float(MATCH_W)
SY = float(CAM_H) / float(MATCH_H)
CX0 = CAM_W * 0.5
CY0 = CAM_H * 0.5


def resolve_template_path():
    for p in TEMPLATE_CANDIDATES:
        if p and os.path.isfile(p):
            return p
    return None


def load_template_pyramid():
    path = resolve_template_path()
    if path is None:
        raise FileNotFoundError("找不到 target1.jpg（脚本同目录或 /root/target1.jpg）")
    raw = cv2.imread(path, cv2.IMREAD_COLOR)
    if raw is None:
        raise RuntimeError("无法解码模板: " + path)

    h0, w0 = raw.shape[:2]
    side = max(h0, w0)
    if side > TEMPLATE_BASE_MAX_SIDE:
        r = float(TEMPLATE_BASE_MAX_SIDE) / float(side)
        raw = cv2.resize(
            raw,
            (max(8, int(w0 * r)), max(8, int(h0 * r))),
            interpolation=cv2.INTER_AREA,
        )
    base = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
    bh, bw = base.shape[:2]
    aspect = float(bw) / float(max(bh, 1))

    short = float(min(MATCH_W, MATCH_H))
    scales = []
    s = SCALE_MIN
    while s <= SCALE_MAX + 1e-9:
        scales.append(round(s, 4))
        s += SCALE_STEP

    pyramid = []
    for sc in scales:
        tw = int(short * sc)
        th = max(8, int(tw / aspect))
        tw = max(8, tw)
        if tw >= MATCH_W or th >= MATCH_H:
            continue
        tmpl = cv2.resize(base, (tw, th), interpolation=cv2.INTER_AREA)
        if tmpl.shape[0] < 8 or tmpl.shape[1] < 8:
            continue
        pyramid.append({"scale": sc, "tmpl": tmpl, "w": tw, "h": th})

    if not pyramid:
        raise RuntimeError("尺度金字塔为空，检查 SCALE 参数")
    # 启动时确认加载的模板，避免 main/debug 各自拿到不同文件
    print("template=%s levels=%d" % (path, len(pyramid)))
    return pyramid


def build_roi_from_hit(hit):
    x0 = hit["x0"] / SX
    y0 = hit["y0"] / SY
    x1 = hit["x1"] / SX
    y1 = hit["y1"] / SY
    bw = max(1.0, x1 - x0)
    bh = max(1.0, y1 - y0)
    cx = (x0 + x1) * 0.5
    cy = (y0 + y1) * 0.5
    rw = bw * ROI_EXPAND
    rh = bh * ROI_EXPAND
    rx = int(cx - rw * 0.5)
    ry = int(cy - rh * 0.5)
    return (rx, ry, int(rw), int(rh))


def select_levels(pyramid, last_scale, neighbor_only):
    if (not neighbor_only) or last_scale is None or not pyramid:
        return pyramid
    best_i = 0
    best_d = abs(pyramid[0]["scale"] - last_scale)
    for i in range(1, len(pyramid)):
        d = abs(pyramid[i]["scale"] - last_scale)
        if d < best_d:
            best_d = d
            best_i = i
    lo = max(0, best_i - SCALE_NEIGHBOR)
    hi = min(len(pyramid), best_i + SCALE_NEIGHBOR + 1)
    levels = pyramid[lo:hi]
    return levels if levels else pyramid


def match_armor(small_gray, pyramid, roi=None, levels=None):
    use_levels = levels if levels is not None else pyramid
    if not use_levels:
        return None

    if roi is not None:
        rx, ry, rw, rh = roi
        x1 = max(0, int(rx))
        y1 = max(0, int(ry))
        x2 = min(small_gray.shape[1], x1 + max(1, int(rw)))
        y2 = min(small_gray.shape[0], y1 + max(1, int(rh)))
        if x2 - x1 < 8 or y2 - y1 < 8:
            return None
        patch = small_gray[y1:y2, x1:x2]
        off_x, off_y = x1, y1
    else:
        patch = small_gray
        off_x, off_y = 0, 0

    ph, pw = patch.shape[:2]
    best = None

    for level in use_levels:
        tmpl = level["tmpl"]
        th, tw = tmpl.shape[:2]
        if ph <= th or pw <= tw:
            continue

        resp = cv2.matchTemplate(patch, tmpl, cv2.TM_CCOEFF_NORMED)
        _, max_v, _, max_loc = cv2.minMaxLoc(resp)
        if max_v < SCORE_PRUNE:
            continue

        x = int(max_loc[0]) + off_x
        y = int(max_loc[1]) + off_y
        cand = {
            "x0": x * SX,
            "y0": y * SY,
            "x1": (x + tw) * SX,
            "y1": (y + th) * SY,
            "cx": (x + tw * 0.5) * SX,
            "cy": (y + th * 0.5) * SY,
            "bw": float(tw) * SX,
            "bh": float(th) * SY,
            "score": float(max_v),
            "scale": level["scale"],
        }
        if best is None or cand["score"] > best["score"]:
            best = cand

    if best is None or best["score"] < MATCH_MIN_SCORE:
        return None
    return best


def estimate_pose(hit):
    """
    由框像素 + 8cm 实尺 → 中心像素、相机系位置与距离。

    相机系（光轴朝前）：X 右，Y 下，Z 前（米）
    返回字段供控制与显示：
      cx_px, cy_px  图像中心（像素，相对 320×240）
      z_m           距离
      x_m, y_m      光心坐标系下目标中心
    """
    w_px = max(float(hit["bw"]), 1.0)
    h_px = max(float(hit["bh"]), 1.0)
    # 正方形板：宽高各估一次再平均，减轻单边抖动
    z_w = FOCAL_PX * ARMOR_W_M / w_px
    z_h = FOCAL_PX * ARMOR_H_M / h_px
    z_m = 0.5 * (z_w + z_h)
    if z_m < 0.05:
        z_m = 0.05

    cx_px = float(hit["cx"])
    cy_px = float(hit["cy"])
    x_m = (cx_px - CX0) * z_m / FOCAL_PX
    y_m = (cy_px - CY0) * z_m / FOCAL_PX
    return {
        "cx_px": cx_px,
        "cy_px": cy_px,
        "x_m": x_m,
        "y_m": y_m,
        "z_m": z_m,
        "w_px": w_px,
        "h_px": h_px,
    }


def calculate_speed(pose):
    """
    米制误差 → 车体速度，使板在画面中心并靠近 Z_TARGET_M。
    目标在相机右侧 x_m>0 → 车向右平移 → vy<0（+vy 为左）
    目标太远 z_m>Z_TARGET → 前进 vx>0
    """
    x_m = pose["x_m"]
    z_m = pose["z_m"]

    if abs(x_m) < X_DEAD_M:
        vy = 0.0
    else:
        vy = -KP_LAT * x_m

    ez = z_m - Z_TARGET_M
    if abs(ez) < Z_DEAD_M:
        vx = 0.0
    else:
        vx = KP_RANGE * ez

    if vx > MAX_SPEED:
        vx = MAX_SPEED
    elif vx < -MAX_SPEED:
        vx = -MAX_SPEED
    if vy > MAX_SPEED:
        vy = MAX_SPEED
    elif vy < -MAX_SPEED:
        vy = -MAX_SPEED
    return vx, vy


def open_uart():
    err.check_raise(pinmap.set_pin_function(PIN_TX, "UART1_TX"), "UART1_TX 引脚失败")
    err.check_raise(pinmap.set_pin_function(PIN_RX, "UART1_RX"), "UART1_RX 引脚失败")
    return uart.UART(SERIAL_DEVICE, BAUD_RATE)


def main():
    pyramid = load_template_pyramid()

    try:
        cam = camera.Camera(CAM_W, CAM_H, image.Format.FMT_BGR888)
    except Exception:
        cam = camera.Camera(CAM_W, CAM_H)
    disp = display.Display()
    uart_dev = open_uart() if SEND_UART else None

    seq = 0
    smooth_vx = 0.0
    smooth_vy = 0.0
    last_hit = None
    lost_cnt = 0
    frame_cnt = 0
    a = float(SMOOTH_A)
    b = 1.0 - a

    while not app.need_exit():
        t0 = time.ticks_ms()
        img = cam.read()
        img_bgr = image.image2cv(img, ensure_bgr=True, copy=False)

        small_bgr = cv2.resize(img_bgr, (MATCH_W, MATCH_H), interpolation=cv2.INTER_AREA)
        small_gray = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2GRAY)

        force_full = (frame_cnt % FULL_SCAN_INTERVAL) == 0
        use_roi = (
            last_hit is not None
            and lost_cnt < MAX_TRACK_LOST
            and not force_full
        )

        if use_roi:
            roi = build_roi_from_hit(last_hit)
            levels = select_levels(pyramid, last_hit.get("scale"), neighbor_only=True)
            hit = match_armor(small_gray, pyramid, roi=roi, levels=levels)
            if hit is None and levels is not pyramid:
                hit = match_armor(small_gray, pyramid, roi=roi, levels=None)
            if hit is None:
                hit = match_armor(small_gray, pyramid, roi=None, levels=None)
        else:
            hit = match_armor(small_gray, pyramid, roi=None, levels=None)

        vx, vy, valid = 0.0, 0.0, False

        if hit is not None:
            last_hit = hit
            lost_cnt = 0

            pose = estimate_pose(hit)
            # 回写中心/距离，供后续扩展或调试读取
            hit["cx_px"] = pose["cx_px"]
            hit["cy_px"] = pose["cy_px"]
            hit["x_m"] = pose["x_m"]
            hit["y_m"] = pose["y_m"]
            hit["z_m"] = pose["z_m"]

            cx_i = int(pose["cx_px"])
            cy_i = int(pose["cy_px"])
            x0, y0 = int(hit["x0"]), int(hit["y0"])
            x1, y1 = int(hit["x1"]), int(hit["y1"])
            cv2.rectangle(img_bgr, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.drawMarker(img_bgr, (cx_i, cy_i), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
            cv2.line(img_bgr, (cx_i, cy_i), (int(CX0), int(CY0)), (0, 255, 255), 1)

            vx_new, vy_new = calculate_speed(pose)
            smooth_vx = a * smooth_vx + b * vx_new
            smooth_vy = a * smooth_vy + b * vy_new
            vx, vy, valid = smooth_vx, smooth_vy, True
        else:
            lost_cnt += 1
            if lost_cnt >= MAX_TRACK_LOST:
                last_hit = None
            smooth_vx = 0.0
            smooth_vy = 0.0

        cv2.drawMarker(img_bgr, (int(CX0), int(CY0)), (255, 128, 0), cv2.MARKER_CROSS, 10, 1)

        frame = pack_avc1(seq, vx, vy, valid, max_mps=MAX_SPEED)
        if uart_dev is not None:
            uart_dev.write(frame)
        seq = (seq + 1) & 0xFFFFFFFF

        disp.show(image.cv2image(img_bgr, bgr=True, copy=False))
        frame_cnt += 1

        left = FRAME_MS - int(time.ticks_diff(time.ticks_ms(), t0))
        if left > 0:
            time.sleep_ms(left)


if __name__ == "__main__":
    main()
