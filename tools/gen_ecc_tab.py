# -*- coding: utf-8 -*-
"""由 ECC 映射数据生成 src/ecc_tab.h

输入：ecc_map.npz，需包含
    sol0  (8193, 4) uint64    每行 224 bit（4 × 64 位字，LSB-first）
    msb   (1,)                位序标志（保留）

输出：C 头文件，含
    ECC_RECUR_ORDER            递推阶数（224）
    ecc_seed[224][7]           224 行种子（每行 224 bit = 7 × u32）
    ecc_recur[7]               224 bit 递推系数

原理：
    8193 行映射不是死数据，它由一条 224 阶线性递推生成
        row[i] = XOR( c_k ? row[i-1-k] : 0 , k = 1..224 )
    用 Berlekamp-Massey 求出 c_k，然后**重算全表并逐行比对**，
    只有 8192/8193 行一致（第 8193 行是常数行，本就全 0）才允许写出。

用法：
    py tools/gen_ecc_tab.py <ecc_map.npz> [out.h]
"""

import os
import re
import sys

import numpy as np

ROWS = 8193
WORDS = 7          # 224 bit / 32
ORDER = 224


def rows_from_npz(path):
    z = np.load(path)
    sol = z["sol0"]
    if sol.shape != (ROWS, 4):
        raise SystemExit("sol0 形状异常: %r（期望 (8193, 4)）" % (sol.shape,))
    out = []
    for i in range(ROWS):
        v = 0
        for w in range(4):
            x = int(sol[i][w])
            for k in range(64):
                if (x >> k) & 1:
                    b = w * 64 + k
                    if b < 224:
                        v |= 1 << b
        out.append(v)
    return out


def berlekamp_massey(bits):
    """GF(2) 上的 BM，返回 (系数表 C, 阶数 L)。"""
    C = [1]
    B = [1]
    L = 0
    m = 1
    for n in range(len(bits)):
        d = bits[n]
        for i in range(1, L + 1):
            if C[i] and bits[n - i]:
                d ^= 1
        if d == 0:
            m += 1
            continue
        T = C[:]
        if len(C) < len(B) + m:
            C += [0] * (len(B) + m - len(C))
        for i in range(len(B)):
            if B[i]:
                C[i + m] ^= 1
        if 2 * L <= n:
            L = n + 1 - L
            B = T
            m = 1
        else:
            m += 1
    return C[: L + 1], L


def to_words(v, nwords=WORDS):
    return [(v >> (32 * w)) & 0xFFFFFFFF for w in range(nwords)]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    npz = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src", "ecc_tab.h")

    rows = rows_from_npz(npz)
    print("读入 %d 行，常数行=%s" % (len(rows), "0" if rows[-1] == 0 else "非0"))

    # 用第 0 位的序列求递推（任一位均可，实测各列递推一致）
    bits0 = [(rows[i] >> 0) & 1 for i in range(4096)]
    C, L = berlekamp_massey(bits0)
    print("递推阶数 = %d，非零系数 = %d" % (L, sum(C)))
    if L != ORDER:
        raise SystemExit("阶数与预期 (%d) 不符，拒绝生成" % ORDER)

    # 重算全表并与原表比对
    seed = rows[:L]
    tab = list(seed)
    for i in range(L, ROWS):
        acc = 0
        for k in range(1, L + 1):
            if C[k]:
                acc ^= tab[i - k]
        tab.append(acc)
    bad = [i for i in range(ROWS) if tab[i] != rows[i]]
    print("重算校验：不一致 %d 行 %s" % (len(bad), bad[:5]))
    if len(bad) != 1 or bad[0] != ROWS - 1:
        raise SystemExit("重算失败，拒绝生成（只允许常数行不一致）")

    recur = sum((C[k] << (k - 1)) for k in range(1, L + 1))

    lines = []
    lines.append("/* 自动生成：ECC %d 阶递推（种子 %d 行 + 系数）" % (L, L))
    lines.append(" * 由 %s 用 Berlekamp-Massey 导出，重算 %d/%d 行一致"
                 % (os.path.basename(npz), ROWS - len(bad), ROWS))
    lines.append(" */")
    lines.append("#define ECC_RECUR_ORDER %du" % L)
    lines.append("")
    lines.append("static const unsigned int ecc_seed[%d][%d] = {" % (L, WORDS))
    for i in range(L):
        lines.append("  {" + ", ".join("0x%08Xu" % w for w in to_words(seed[i])) + "},")
    lines.append("};")
    lines.append("")
    lines.append("static const unsigned int ecc_recur[%d] = {" % WORDS)
    lines.append("  " + ", ".join("0x%08Xu" % w for w in to_words(recur)))
    lines.append("};")
    lines.append("")

    open(out, "w", encoding="utf-8", newline="\n").write("\n".join(lines))
    print("已写出 %s（%d 行）" % (out, len(lines)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
