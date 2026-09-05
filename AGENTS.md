# AGENTS.md（校内赛 / MaixCAM 装甲板视觉）

校内赛 MaixCAM (MaixPy) 仓库。核心路线是板端模板匹配找装甲板，算车体平移速度（+x 向前，+y 向左），通过 UART1 以 20-50 Hz 持续发 AVC1 帧。

## 主要文件

| 文件 | 作用 |
|------|------|
| `main.py` | 板端主程序，matchTemplate + 8cm 针孔测距/对中 + 米制跟随 + UART AVC1 |
| `debug_pose.py` | 测距/左右调试 HUD 和串口日志，默认不发 UART |
| `target1.jpg` | 装甲板模板，脚本同目录或板上 `/root/target1.jpg` |
| `avc1_protocol.py` | AVC1 打包/校验/黄金向量的唯一来源，PC 和板端共用 |
| `test_avc1_pack.py` | PC 自检入口，调 `avc1_protocol.selftest` |
| `README.md` | 接线、运行、联调顺序 |
| `TUNING.md` | 症状导向的调参手册 |

## 运行方式

- 板端用 MaixVision 跑 `main.py`，需要同目录有 `avc1_protocol.py` 和模板，依赖 `maix` 和 OpenCV
- PC 上 `python avc1_protocol.py` 直接自检，只用标准库
- 预览不想发串口，把 `main.py` 里 `SEND_UART` 设为 False

## main.py 流程

1. 加载 `target1.jpg`，转灰度，建尺度金字塔
2. 每帧 320×240 缩到 `MATCH_W×H` 灰度图，跑 `TM_CCOEFF_NORMED`
3. 锁定后只在 ROI 加尺度邻域里找，失败回退全图
4. 命中后 `estimate_pose` 给出 `(cx,cy)`、`Z=f*0.08/w_px` 和相机系 `X,Y`，米制 P 控制对中和跟距，平滑后输出 `vx/vy`，`valid=1`
5. 丢失时发 `valid=0, vx=vy=0`，帧照发，seq 照加
6. `pack_avc1` 只从 `avc1_protocol` 拿

## 调参顺序（只动顶部常量）

1. 模板路径
2. `MATCH_MIN_SCORE` / `SCALE_*`
3. `FOCAL_PX` 标定，`w_px * D / 0.08`，`ARMOR_*_M` 保持 0.08
4. `Z_TARGET_M`、`X_DEAD_M`、`Z_DEAD_M`、`KP_LAT`、`KP_RANGE`
5. `ROI_EXPAND` / `MAX_TRACK_LOST` / `FULL_SCAN_INTERVAL`
6. `SEND_HZ`，20-50，默认 30

## AVC1 契约（不要改）

- 24 字节小端，`magic=0x31435641`，校验种子 `0xA5A51234`，逐轴限幅 ±0.8 m/s
- UART1 用 A19/A18，设备 `/dev/ttyS1`，115200；TX 接 C 板 PG9，共地
- 持续发帧，C 板 200 ms 超时；进 auto 要圆圈解锁、方块切换，再按住 L1
- 黄金向量只维护在 `avc1_protocol.GOLDEN_CASES`

接口文档在下面两个链接。
https://github.com/shiuhou/pnx_newtemplate_F4/blob/feat/ps2-auto-vision/docs/vision-auto-chassis-interface.md
https://github.com/shiuhou/pnx_newtemplate_F4/blob/feat/ps2-auto-vision/docs/vision-auto-chassis-quick-start.md

MaixPy 文档在 https://wiki.sipeed.com/maixpy/doc/zh/index.html

## 约定

- 协议改动只动 `avc1_protocol.py`，改完跑自检
- 阈值集中在 `main.py` 顶部
- 中文注释保持有用；除非明确改需求，不引入 YOLO 或电赛模型
- 不在这个仓改 C 板固件，实车和烧录归电控管

## 常用命令

```bash
python avc1_protocol.py
python test_avc1_pack.py
```

板端入口是 `main.py`，MaixVision 或板上默认启动都行。
