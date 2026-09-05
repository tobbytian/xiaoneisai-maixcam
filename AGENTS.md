# AGENTS.md — 校内赛 / MaixCAM 装甲板视觉

## Purpose

School-competition **MaixCAM (MaixPy)** workspace. Core path: **template-match armor on device**, compute body velocity (`+x` forward, `+y` left), send **AVC1** on UART1 continuously at 20–50 Hz.

## Primary files

| File | Role |
|------|------|
| **`main.py`** | **板端主程序**：matchTemplate + 8cm 针孔测距/中心 + 米制跟随 + UART AVC1 |
| **`debug_pose.py`** | 测距/左右调试 HUD + 串口日志；默认无 UART |
| **`target1.jpg`** | 装甲模板（脚本同目录或板上 `/root/target1.jpg`） |
| **`avc1_protocol.py`** | AVC1 **唯一**打包/校验/黄金向量（PC + 板端 import） |
| `test_avc1_pack.py` | PC 自检入口 → `avc1_protocol.selftest` |
| `README.md` | 接线、联调顺序、调参 |

## Runtime

- **Device**: 板端 / MaixVision 跑 **`main.py`**；需同目录 `avc1_protocol.py` + 模板。`maix` + OpenCV。
- **PC**: `python avc1_protocol.py`（stdlib only）。
- 预览不发串口：`main.py` 里 `SEND_UART = False`。

## Armor pipeline (`main.py`)

1. 加载 `target1.jpg` → 灰度底座 → 尺度金字塔  
2. 帧 320×240 → 缩 `MATCH_W×H` 灰度 → `TM_CCOEFF_NORMED`  
3. 锁定后 ROI + 尺度邻域；失败全图回退  
4. 命中：`estimate_pose` → `(cx,cy)` + `Z=f*0.08/w_px` + 相机系 `X,Y`；米制 P 控制对中/跟距 → 平滑 `vx/vy`，`valid=1`  
5. 丢失：`valid=0, vx=vy=0`，**仍发帧且 seq++**  
6. `pack_avc1` 仅来自 `avc1_protocol`

**Tune order**（仅顶部常量）:

1. 模板路径  
2. `MATCH_MIN_SCORE` / `SCALE_*`  
3. **`FOCAL_PX` 标定**（`w_px * D / 0.08`）；`ARMOR_*_M=0.08`  
4. `Z_TARGET_M`、`X_DEAD_M`、`Z_DEAD_M`、`KP_LAT`、`KP_RANGE`  
5. `ROI_EXPAND` / `MAX_TRACK_LOST` / `FULL_SCAN_INTERVAL`  
6. `SEND_HZ`（20–50，默认 30）

## AVC1 contract (do not drift)

- 24 B LE，`magic=0x31435641`，csum seed `0xA5A51234`，clamp ±0.8 m/s  
- UART1 A19/A18 → `/dev/ttyS1` 115200；TX→C 板 **PG9**，共地  
- 持续发；C 板 200 ms 超时；auto 需圆圈解锁 + 方块 + **按住 L1**  
- 黄金向量只维护在 `avc1_protocol.GOLDEN_CASES`

Docs:  
https://github.com/shiuhou/pnx_newtemplate_F4/blob/feat/ps2-auto-vision/docs/vision-auto-chassis-interface.md  
https://github.com/shiuhou/pnx_newtemplate_F4/blob/feat/ps2-auto-vision/docs/vision-auto-chassis-quick-start.md  
https://wiki.sipeed.com/maixpy/doc/zh/index.html

## Conventions

- 协议改动只动 `avc1_protocol.py`，再跑自检  
- 阈值集中在 `main.py` 顶部  
- 中文注释保持有用；不引入 YOLO/电赛模型 unless 明确改需求  
- 不在本仓改 C 板固件；实车/烧录属电控授权范围  

## Quick commands

```bash
python avc1_protocol.py
python test_avc1_pack.py
```

On-device: **`main.py`**（MaixVision 或板端默认入口）。
