"""
调试：检查测距 Z 与左右判断（不开车、默认不发 UART）

用法（MaixVision）：
  与 main.py / avc1_protocol.py / target1.jpg 同目录，运行本文件。
  板面尽量平行镜头；用卷尺量真实距离，对照屏上 Z。

屏上：
  绿框 / 黄线：目标中心 → 画面中心
  竖虚线：左右死区；横条文字：Z、X、判定、建议 vx/vy
  标定：若填了 MEASURED_Z_M>0，会显示建议 FOCAL_PX

不控制底盘。确认准了再回 main.py 改 FOCAL_PX / 死区 / 增益。
"""
import os
from maix import camera, display, image, time, app
import cv2

# ======== 与 main.py 保持一致（改标定时两边一起改）=======
CAM_W, CAM_H = 320, 240
MATCH_W, MATCH_H = 40, 30

TEMPLATE_CANDIDATES = (
    # 统一用板上 /root 的模板
    "/root/target1.jpg",
    "target1.jpg",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "target1.jpg"),
)

MATCH_MIN_SCORE = 0.62
SCALE_MIN = 0.3
SCALE_MAX = 0.9
SCALE_STEP = 0.05
TEMPLATE_BASE_MAX_SIDE = 96

ARMOR_W_M = 0.08
ARMOR_H_M = 0.08
FOCAL_PX = 277.0
Z_TARGET_M = 0.2
X_DEAD_M = 0.02
Z_DEAD_M = 0.04
KP_LAT = 1.2
KP_RANGE = 0.9
MAX_SPEED = 0.8

ROI_EXPAND = 1.8
MAX_TRACK_LOST = 3
FULL_SCAN_INTERVAL = 8
SCALE_NEIGHBOR = 2
SCORE_PRUNE = MATCH_MIN_SCORE * 0.8

# 卷尺量到的真实距离（米）。>0 时显示建议焦距：f = w_px * D / 0.08
MEASURED_Z_M = 0.0

# 调试输出
SHOW_HZ = 25
PRINT_EVERY_MS = 500
FRAME_MS = max(1, int(1000 / SHOW_HZ))

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
        raise FileNotFoundError("no target1.jpg")
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
        raise RuntimeError("尺度金字塔为空")
    print("debug_pose template=%s levels=%d f=%.1f Zt=%.2f" % (path, len(pyramid), FOCAL_PX, Z_TARGET_M))
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
    return (int(cx - rw * 0.5), int(cy - rh * 0.5), int(rw), int(rh))


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
    w_px = max(float(hit["bw"]), 1.0)
    h_px = max(float(hit["bh"]), 1.0)
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
        "z_w": z_w,
        "z_h": z_h,
        "w_px": w_px,
        "h_px": h_px,
    }


def judge_lr(x_m):
    if abs(x_m) < X_DEAD_M:
        return "CENTER", 0
    if x_m > 0:
        return "RIGHT", -1  # 板在右 → 车应向右 → vy<0
    return "LEFT", 1


def judge_range(z_m):
    ez = z_m - Z_TARGET_M
    if abs(ez) < Z_DEAD_M:
        return "HOLD", 0
    if ez > 0:
        return "FAR", 1  # 太远 → 前进 vx>0
    return "NEAR", -1


def suggest_speed(pose):
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
    vx = max(-MAX_SPEED, min(MAX_SPEED, vx))
    vy = max(-MAX_SPEED, min(MAX_SPEED, vy))
    return vx, vy


def x_dead_to_px(z_m):
    """当前距离下，X_DEAD_M 对应的半宽像素。"""
    return X_DEAD_M * FOCAL_PX / max(z_m, 0.05)


def draw_hud(img_bgr, hit, pose):
    cv2.drawMarker(img_bgr, (int(CX0), int(CY0)), (255, 128, 0), cv2.MARKER_CROSS, 12, 1)
    # 画面中线
    cv2.line(img_bgr, (int(CX0), 0), (int(CX0), CAM_H - 1), (80, 80, 80), 1)

    if hit is None or pose is None:
        cv2.putText(
            img_bgr,
            "NO TARGET",
            (6, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )
        cv2.putText(
            img_bgr,
            "hold plate front-on",
            (6, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 0, 255),
            1,
        )
        return

    x0, y0 = int(hit["x0"]), int(hit["y0"])
    x1, y1 = int(hit["x1"]), int(hit["y1"])
    cx_i = int(pose["cx_px"])
    cy_i = int(pose["cy_px"])
    cv2.rectangle(img_bgr, (x0, y0), (x1, y1), (0, 255, 0), 2)
    cv2.drawMarker(img_bgr, (cx_i, cy_i), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
    cv2.line(img_bgr, (cx_i, cy_i), (int(CX0), int(CY0)), (0, 255, 255), 1)

    # 左右死区（随当前 Z 变宽/变窄）
    half = int(x_dead_to_px(pose["z_m"]))
    cv2.line(img_bgr, (int(CX0) - half, 0), (int(CX0) - half, CAM_H - 1), (0, 180, 0), 1)
    cv2.line(img_bgr, (int(CX0) + half, 0), (int(CX0) + half, CAM_H - 1), (0, 180, 0), 1)

    lr, _ = judge_lr(pose["x_m"])
    rg, _ = judge_range(pose["z_m"])
    vx, vy = suggest_speed(pose)
    dx_px = pose["cx_px"] - CX0

    line1 = "Z:%.2fm X:%+.2fm" % (pose["z_m"], pose["x_m"])
    line2 = "LR:%s  RNG:%s" % (lr, rg)
    line3 = "dx:%+.0fpx w:%.0f sc:%.2f" % (dx_px, pose["w_px"], hit["score"])
    line4 = "cmd vx:%+.2f vy:%+.2f" % (vx, vy)
    # vy>0 表示建议向左平移
    line5 = "move: %s  %s" % (
        "LEFT" if vy > 0.02 else ("RIGHT" if vy < -0.02 else "—"),
        "FWD" if vx > 0.02 else ("BACK" if vx < -0.02 else "—"),
    )

    y = 16
    for t, col in (
        (line1, (0, 255, 255)),
        (line2, (0, 255, 0) if lr == "CENTER" and rg == "HOLD" else (0, 200, 255)),
        (line3, (200, 200, 200)),
        (line4, (255, 200, 0)),
        (line5, (255, 128, 0)),
    ):
        cv2.putText(img_bgr, t, (4, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
        y += 16

    if MEASURED_Z_M and MEASURED_Z_M > 0.05:
        f_s = pose["w_px"] * MEASURED_Z_M / ARMOR_W_M
        err_z = pose["z_m"] - MEASURED_Z_M
        cv2.putText(
            img_bgr,
            "Dtrue:%.2f Zerr:%+.2f f*=%.0f" % (MEASURED_Z_M, err_z, f_s),
            (4, CAM_H - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 128),
            1,
        )
    else:
        cv2.putText(
            img_bgr,
            "set MEASURED_Z_M to calib f",
            (4, CAM_H - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (120, 120, 120),
            1,
        )


def main():
    pyramid = load_template_pyramid()
    try:
        cam = camera.Camera(CAM_W, CAM_H, image.Format.FMT_BGR888)
    except Exception:
        cam = camera.Camera(CAM_W, CAM_H)
    disp = display.Display()

    last_hit = None
    lost_cnt = 0
    frame_cnt = 0
    last_print = time.ticks_ms()

    while not app.need_exit():
        t0 = time.ticks_ms()
        img = cam.read()
        img_bgr = image.image2cv(img, ensure_bgr=True, copy=False)
        small_bgr = cv2.resize(img_bgr, (MATCH_W, MATCH_H), interpolation=cv2.INTER_AREA)
        small_gray = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2GRAY)

        force_full = (frame_cnt % FULL_SCAN_INTERVAL) == 0
        use_roi = last_hit is not None and lost_cnt < MAX_TRACK_LOST and not force_full

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

        pose = None
        if hit is not None:
            last_hit = hit
            lost_cnt = 0
            pose = estimate_pose(hit)
        else:
            lost_cnt += 1
            if lost_cnt >= MAX_TRACK_LOST:
                last_hit = None

        draw_hud(img_bgr, hit, pose)

        now = time.ticks_ms()
        if pose is not None and time.ticks_diff(now, last_print) >= PRINT_EVERY_MS:
            lr, _ = judge_lr(pose["x_m"])
            rg, _ = judge_range(pose["z_m"])
            vx, vy = suggest_speed(pose)
            msg = (
                "Z=%.3f X=%+.3f dx=%+.1f w=%.1f score=%.2f | %s %s | vx=%+.2f vy=%+.2f"
                % (
                    pose["z_m"],
                    pose["x_m"],
                    pose["cx_px"] - CX0,
                    pose["w_px"],
                    hit["score"],
                    lr,
                    rg,
                    vx,
                    vy,
                )
            )
            if MEASURED_Z_M > 0.05:
                msg += " | f_sugg=%.1f" % (pose["w_px"] * MEASURED_Z_M / ARMOR_W_M)
            print(msg)
            last_print = now

        disp.show(image.cv2image(img_bgr, bgr=True, copy=False))
        frame_cnt += 1
        left = FRAME_MS - int(time.ticks_diff(time.ticks_ms(), t0))
        if left > 0:
            time.sleep_ms(left)


if __name__ == "__main__":
    main()
