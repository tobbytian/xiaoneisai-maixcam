"""PC 侧 AVC1 打包自检（无需 Maix）。委托 avc1_protocol 黄金向量。"""
from avc1_protocol import selftest

if __name__ == "__main__":
    raise SystemExit(0 if selftest() else 1)
