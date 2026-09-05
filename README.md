调试指南参见https://github.com/tobbytian/xiaoneisai-maixcam/edit/main/TUNING.md

# 校内赛 · MaixCAM 装甲板视觉（AVC1）

MaixCAM 端识别白色装甲板模板，计算车体平移速度，经 UART 向 DJI C 板发送 **AVC1** 帧。  
电控/底盘侧见参考分支，本仓库只负责视觉与协议打包。

## 文件

| 文件 | 说明 |
|------|------|
| `main.py` | **板端主程序**（匹配 + 8cm 测距/中心 + 跟随 + 串口） |
| `debug_pose.py` | **测距/左右调试**（只显示+打印，默认不发串口、不开车） |
| `target1.jpg` | 装甲板模板（与脚本同目录，或板上 `/root/target1.jpg`） |
| `avc1_protocol.py` | AVC1 打包/解包/黄金向量（PC 与板端共用） |
| `test_avc1_pack.py` | PC 一键自检入口 |

## 接线（V1 单向）

| MaixCAM Pro | C 板 F407 | 说明 |
|-------------|-----------|------|
| UART1 TX **A19** | **PG9** USART6_RX | 命令 |
| GND | GND | 必须共地 |
| UART1 RX A18 | （可不接） | V1 无 ACK |

- 电平：**3.3 V TTL**，**115200 8N1**
- 不要接 RS-232；不要与 PS2/裁判串口复用

## 坐标与帧率

- **+vx**：车头向前（m/s）
- **+vy**：车体向左（m/s）
- 无 wz；C 板 auto 固定 `wz=0`
- 逐轴限幅 **±0.8 m/s**
- **20–50 Hz** 持续发送（默认 30 Hz）；**每帧 seq+1**，丢失目标时发 `valid=0, vx=vy=0`
- C 板 **>200 ms** 无合法新帧 → 停车

## 测距与对中（8×8 cm 装甲板）

针孔模型（分辨率 `CAM_W×CAM_H=320×240`）：

- 距离：`Z = FOCAL_PX * 0.08 / w_px`（宽、高各算一次取平均）
- 中心像素：`(cx_px, cy_px)` = 匹配框中心
- 相机系位置（米）：`X = (cx - cx0) * Z / f`，`Y = (cy - cy0) * Z / f`（X 右，Z 前）
- 控制：消掉横向 `X`、把 `Z` 拉到 `Z_TARGET_M`（默认 0.5 m），输出 `vx/vy`

**标定 `FOCAL_PX`（重要）**：板面平行镜头，卷尺量真实距离 `D`（m），读绿框宽度 `w_px`：

```text
FOCAL_PX = w_px * D / 0.08
```

写入 `main.py` 顶部。出厂粗估按水平 FOV≈60° → `f≈277`（320 宽），未标定会有比例误差。

## 板端运行

1. 将 `main.py`、`avc1_protocol.py`、`target1.jpg` 放到板子同一目录（或模板在 `/root/target1.jpg`）
2. MaixVision 打开 **`main.py`** 运行（或板端默认启动 `main.py`）
3. 屏上绿框 = 锁定；准星连线指向板中心；无框时仍在发停车帧
4. 仅预览、不发串口：把 `SEND_UART = False`

### 先调测距/左右：`debug_pose.py`

1. 同目录再放 `debug_pose.py`，MaixVision 运行它（**不发 UART**）
2. 板正对镜头，看屏上 `Z`、`X`、`LR`（LEFT/CENTER/RIGHT）、`RNG`（FAR/HOLD/NEAR）
3. 卷尺量真实距离 `D`，在文件顶部设 `MEASURED_Z_M = D`，屏底会给 **建议 `FOCAL_PX`**
4. 把建议焦距写回 `debug_pose.py` 与 **`main.py`** 的 `FOCAL_PX` 后，再跑主程序

### 调参（`main.py` 顶部）

1. `MATCH_MIN_SCORE`：匹配阈值  
2. `SCALE_MIN/MAX/STEP`：远近尺度  
3. `FOCAL_PX`：按上面公式标定；`ARMOR_W_M/H_M=0.08`  
4. `Z_TARGET_M`、`X_DEAD_M`、`Z_DEAD_M`、`KP_LAT`、`KP_RANGE`：对中/跟距手感  
5. `ROI_*` / `FULL_SCAN_INTERVAL`：跟踪加速（非协议字段）

## C 板联调顺序（安全）

参考 quick-start，实车前建议：

1. 上电默认锁定手动；**圆圈解锁** → **方块进 auto**
2. 未按 **L1** 时发 `valid=1` 低速帧 → 车应仍静止  
3. 发 `valid=0` → 静止  
4. 安全确认后：**按住 L1**，单轴 ≤0.1 m/s 测前/后/左/右（不应自转）  
5. 松 L1 / 停发 >200 ms / 拔线 → 应停车  
6. 重启 Maix 后 seq 可从 0 再计数（超时后 C 板重建基线）

视觉组不改 C 板固件；烧录与实车需电控授权与安全措施。

## PC 自检

```bash
python avc1_protocol.py
# 或
python test_avc1_pack.py
```

## 参考

- 接口与快速开始：  
  https://github.com/shiuhou/pnx_newtemplate_F4/tree/feat/ps2-auto-vision  
- MaixPy OpenCV：https://wiki.sipeed.com/maixpy/doc/zh/vision/opencv.html  
- MaixPy UART：https://wiki.sipeed.com/maixpy/doc/zh/peripheral/uart.html  
- MaixPy 相机：https://wiki.sipeed.com/maixpy/doc/zh/vision/camera.html  
