# 校内赛 MaixCAM 装甲板视觉

板端用模板匹配找白色装甲板，算出车体平移速度，走 UART1 按 AVC1 协议发给 DJI C 板。接车和底盘逻辑在电控的参考分支里，这个仓库只管视觉和协议打包。

## 文件

| 文件 | 作用 |
|------|------|
| `main.py` | 板端主程序，模板匹配 + 测距/对中 + 跟随 + 串口 |
| `debug_pose.py` | 调测距和左右判断用，只显示和打印，默认不发串口 |
| `target1.jpg` | 装甲板模板，和脚本同目录，板上也可以放 `/root/target1.jpg` |
| `avc1_protocol.py` | AVC1 打包/解包/黄金向量，PC 和板端共用一份 |
| `test_avc1_pack.py` | PC 自检入口 |
| `TUNING.md` | 调参手册，按症状查 |

## 接线（V1 单向）

| MaixCAM Pro | C 板 F407 | 说明 |
|-------------|-----------|------|
| UART1 TX A19 | PG9（USART6_RX） | 命令 |
| GND | GND | 必须共地 |
| UART1 RX A18 | 可以不接 | V1 没有 ACK |

电平 3.3 V TTL，115200 8N1。别接 RS-232，也别和 PS2、裁判系统的串口混用。

## 坐标和帧率

+vx 车头向前，+vy 车体向左，单位 m/s，逐轴限幅 ±0.8。没有 wz，C 板 auto 固定 wz=0。

发送频率保持在 20 到 50 Hz，默认 30。每帧 seq 加一；跟丢目标就发 valid=0、vx=vy=0 的帧，不能停发。C 板超过 200 ms 收不到合法新帧会停车。

## 测距和对中

装甲板按 8×8 cm 实尺算，针孔模型，分辨率 320×240。

- 距离 `Z = FOCAL_PX * 0.08 / w_px`，宽和高各算一次再取平均
- 中心像素取匹配框中心
- 相机系位置 `X = (cx - cx0) * Z / f`，`Y = (cy - cy0) * Z / f`，X 朝右，Z 朝前
- 控制上就是消掉横向 X，把 Z 拉到 `Z_TARGET_M`（现在设的 0.2 m），输出 vx/vy

`FOCAL_PX` 要标定。板面平行镜头，卷尺量真实距离 D（米），读屏上绿框的宽度 w_px，然后算

```text
FOCAL_PX = w_px * D / 0.08
```

结果写进 `main.py` 顶部。没标过先用出厂粗估，水平 FOV 按 60° 算 f 大约 277（320 像素宽），没标定距离会有比例误差。

## 板上跑起来

1. `main.py`、`avc1_protocol.py`、`target1.jpg` 放同一目录（模板也可以只放 `/root/target1.jpg`）
2. MaixVision 打开 `main.py` 运行，或者设成板上默认启动
3. 屏上绿框表示锁定，准星连线指向板中心；没有框也照常在发停车帧
4. 只想预览不想发串口，把 `SEND_UART = False`

### 先跑 `debug_pose.py`

上车之前先跑这个，确认测距和左右判断是对的。它不发串口，车不会动。

1. 和主程序放同一目录，MaixVision 运行
2. 板正对镜头，看屏上 `Z`、`X`、`LR`（LEFT/CENTER/RIGHT）、`RNG`（FAR/HOLD/NEAR）
3. 卷尺量真实距离 D，填到文件顶部 `MEASURED_Z_M = D`，屏底会给出建议焦距
4. 把建议值写回 `debug_pose.py` 和 `main.py` 的 `FOCAL_PX`，再跑主程序

### 调参顺序

都在 `main.py` 顶部，按这个顺序来。

1. `MATCH_MIN_SCORE`，匹配门槛
2. `SCALE_MIN/MAX/STEP`，远近尺度
3. `FOCAL_PX` 标定，`ARMOR_W_M/H_M` 保持 0.08
4. `Z_TARGET_M`、`X_DEAD_M`、`Z_DEAD_M`、`KP_LAT`、`KP_RANGE`，对中和跟距的手感
5. `ROI_*`、`FULL_SCAN_INTERVAL`，跟踪加速
6. `SEND_HZ`，20 到 50，默认 30

车有什么具体毛病，去 [TUNING.md](TUNING.md) 按症状查。

## C 板联调（先安全后动车）

参考 quick-start 文档，实车前按这个顺序来。

1. 上电默认手动锁定，圆圈解锁，方块进 auto
2. 不按 L1，发 valid=1 低速帧，车应该不动
3. 发 valid=0，也应该不动
4. 确认没问题后按住 L1，单轴 0.1 m/s 以内分别测前后左右，车不应该自转
5. 松开 L1、停发超过 200 ms、拔线，这三种情况都应该停车
6. 重启 Maix 后 seq 从 0 重新计数没问题，超时后 C 板会重建基线

C 板固件不归视觉组改，烧录和实车要电控授权，做好安全措施。

## PC 自检

```bash
python avc1_protocol.py
# 或
python test_avc1_pack.py
```

输出对上接口文档的三组黄金向量，就说明打包没问题。

## 参考

- 接口与快速开始 https://github.com/shiuhou/pnx_newtemplate_F4/tree/feat/ps2-auto-vision
- MaixPy OpenCV https://wiki.sipeed.com/maixpy/doc/zh/vision/opencv.html
- MaixPy UART https://wiki.sipeed.com/maixpy/doc/zh/peripheral/uart.html
- MaixPy 相机 https://wiki.sipeed.com/maixpy/doc/zh/vision/camera.html
