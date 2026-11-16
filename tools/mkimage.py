#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mkimage.py — DHTB 镜像打包/解包/校验

展锐 DHTB 头结构（实测自 fdl2-device.bin，66,544 B）：

    偏移 0x000  "DHTB"            magic
    偏移 0x004  u32  version = 1
    偏移 0x030  u32  payload_size
    偏移 0x034 ~ 0x1FF  全 0
    偏移 0x200  ← 负载起点

⚠️ 本机这颗 FDL2 的 DHTB 头里没有签名/哈希/校验和被使用（除 magic/version/size 外全 0），
   与「EFUSE 未熔丝」互相印证。本脚本按此格式原样复现。

用法：
    python mkimage.py info   fdl2.bin            # 打印头部字段
    python mkimage.py unpack fdl2.bin out.raw    # 拆出负载
    python mkimage.py pack   in.raw out.bin      # 打上 DHTB 头
    python mkimage.py selftest fdl2.bin          # 解包再打包，比对 md5
"""

import sys
import os
import struct
import hashlib

HDR_MAGIC = b"DHTB"
HDR_VER = 1
HDR_SIZE = 0x200
OFF_VER = 0x04
OFF_SIZE = 0x30


def md5(b):
    return hashlib.md5(b).hexdigest()


def parse_header(data):
    if len(data) < HDR_SIZE:
        raise ValueError("文件太小，不足 %d 字节的头" % HDR_SIZE)
    if data[:4] != HDR_MAGIC:
        raise ValueError("magic 不是 DHTB，实际 %r" % data[:4])
    ver = struct.unpack("<I", data[OFF_VER:OFF_VER + 4])[0]
    psize = struct.unpack("<I", data[OFF_SIZE:OFF_SIZE + 4])[0]
    actual = len(data) - HDR_SIZE
    return {
        "magic": data[:4].decode("ascii", "replace"),
        "version": ver,
        "payload_size": psize,
        "actual_payload": actual,
        "consistent": psize == actual,
        "trailing": actual - psize,
        "hdr_nonzero": [(i, data[i]) for i in range(HDR_SIZE)
                        if data[i] != 0
                        and not (0 <= i < 4)
                        and not (OFF_VER <= i < OFF_VER + 4)
                        and not (OFF_SIZE <= i < OFF_SIZE + 4)],
    }


def cmd_info(path):
    data = open(path, "rb").read()
    h = parse_header(data)
    print("文件      : %s" % path)
    print("总大小    : %d (0x%X)" % (len(data), len(data)))
    print("magic     : %s" % h["magic"])
    print("version   : %d" % h["version"])
    print("payload   : %d (0x%X)  声明" % (h["payload_size"], h["payload_size"]))
    print("           %d (0x%X)  实际" % (h["actual_payload"], h["actual_payload"]))
    print("一致      : %s" % ("是" if h["consistent"] else "否 ⚠️"))
    if h["trailing"]:
        print("尾部多余  : %d 字节" % h["trailing"])
    print("头部其他非零字节: %s" % (h["hdr_nonzero"] or "无（无签名/无校验和）"))
    print("md5       : %s" % md5(data))
    return 0


def cmd_unpack(src, dst):
    data = open(src, "rb").read()
    h = parse_header(data)
    payload = data[HDR_SIZE:HDR_SIZE + h["payload_size"]]
    open(dst, "wb").write(payload)
    print("已拆出负载: %s  %d 字节  md5=%s" % (dst, len(payload), md5(payload)))
    return 0


def build(payload, version=HDR_VER):
    hdr = bytearray(HDR_SIZE)
    hdr[0:4] = HDR_MAGIC
    struct.pack_into("<I", hdr, OFF_VER, version)
    struct.pack_into("<I", hdr, OFF_SIZE, len(payload))
    return bytes(hdr) + payload


def cmd_pack(src, dst):
    payload = open(src, "rb").read()
    img = build(payload)
    open(dst, "wb").write(img)
    print("已打包    : %s  %d 字节  md5=%s" % (dst, len(img), md5(img)))
    print("负载      : %d 字节  md5=%s" % (len(payload), md5(payload)))
    return 0


def cmd_selftest(path):
    data = open(path, "rb").read()
    h = parse_header(data)
    payload = data[HDR_SIZE:HDR_SIZE + h["payload_size"]]
    rebuilt = build(payload, h["version"])

    ok = True
    if rebuilt != data:
        ok = False
        # 找出第一处差异，便于定位
        n = min(len(rebuilt), len(data))
        diff = next((i for i in range(n) if rebuilt[i] != data[i]), None)
        print("❌ 重建结果与原文件不一致")
        if diff is not None:
            print("   首个差异 @0x%X: 原=%02X 重建=%02X"
                  % (diff, data[diff], rebuilt[diff]))
        print("   原 md5  = %s" % md5(data))
        print("   重建 md5= %s" % md5(rebuilt))
        print("   长度    原=%d 重建=%d" % (len(data), len(rebuilt)))
    else:
        print("✅ round-trip 完全一致")
        print("   md5 = %s" % md5(data))
        print("   负载 %d 字节，头 %d 字节" % (len(payload), HDR_SIZE))

    print("\n头部字段复核:")
    print("   magic=%s version=%d payload=%d"
          % (h["magic"], h["version"], h["payload_size"]))
    print("   其他非零字节: %s" % (h["hdr_nonzero"] or "无"))
    return 0 if ok else 1


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    cmd = argv[1]
    if cmd == "info":
        return cmd_info(argv[2])
    if cmd == "unpack" and len(argv) >= 4:
        return cmd_unpack(argv[2], argv[3])
    if cmd == "pack" and len(argv) >= 4:
        return cmd_pack(argv[2], argv[3])
    if cmd == "selftest":
        return cmd_selftest(argv[2])
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
