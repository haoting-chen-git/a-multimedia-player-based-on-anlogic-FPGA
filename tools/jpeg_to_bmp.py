# -*- coding: utf-8 -*-
r"""
jpg/png -> TF 卡可读的 24 位非压缩 BMP 转换工具
=================================================================
用于把电脑上的照片转成 40_player_release 播放器能直接读取的 BMP：

    * 24 位色（BI_RGB）、不压缩、自底向上
    * 宽度为 4 的倍数（bmp_read.v 的硬性检查）
    * 等比缩放到不超过 640x480，不放大、不裁剪、不拉伸
      （播放器自身还会做 letterbox，所以 16:9 / 3:4 素材都不会变形）

用法：
    python jpeg_to_bmp.py <输入目录> <输出目录> [最大宽] [最大高]

例：
    python jpeg_to_bmp.py ..\src\pic ..\tf_card 640 480
"""

import os
import sys
import glob
from PIL import Image

MAX_W = 640
MAX_H = 480


def fit_size(w, h, max_w, max_h):
    """等比缩放到 max_w x max_h 以内，宽度向下取 4 的倍数。"""
    scale = min(max_w / w, max_h / h, 1.0)
    nw = int(w * scale + 0.5)
    nh = int(h * scale + 0.5)
    nw -= nw % 4                     # BMP 要求宽度 4 的倍数
    nw = max(nw, 4)
    nh = max(nh, 1)
    return nw, nh


def convert(src, dst, max_w=MAX_W, max_h=MAX_H):
    im = Image.open(src)
    im = im.convert("RGB")           # 丢掉 alpha / 调色板，保证 24 位
    nw, nh = fit_size(im.width, im.height, max_w, max_h)
    if (nw, nh) != im.size:
        # LANCZOS 比最近邻更适合缩小时保留细节；播放器内部缩放在 640 像素
        # 宽度上做最近邻，所以这里先把图缩到位，等于做了一次"高质量预缩放"。
        im = im.resize((nw, nh), Image.LANCZOS)
    im.save(dst, format="BMP")
    return nw, nh, os.path.getsize(dst)


def main():
    inp = sys.argv[1] if len(sys.argv) > 1 else r"..\src\pic"
    out = sys.argv[2] if len(sys.argv) > 2 else r"..\tf_card"
    max_w = int(sys.argv[3]) if len(sys.argv) > 3 else MAX_W
    max_h = int(sys.argv[4]) if len(sys.argv) > 4 else MAX_H

    os.makedirs(out, exist_ok=True)
    files = []
    for ext in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
        files += glob.glob(os.path.join(inp, ext))
    files.sort()

    for i, f in enumerate(files, 1):
        dst = os.path.join(out, "IMG%d.BMP" % i)   # 8.3 短文件名，兼容 FAT 扫描
        nw, nh, size = convert(f, dst, max_w, max_h)
        print("%-46s -> %-10s %4dx%-4d %7.1f KB"
              % (os.path.basename(f), os.path.basename(dst), nw, nh, size / 1024.0))
    print("\n完成：%d 张 -> %s" % (len(files), os.path.abspath(out)))


if __name__ == "__main__":
    main()
