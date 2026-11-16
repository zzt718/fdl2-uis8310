#!/usr/bin/env bash
# =============================================================================
# build.sh — fdl2-uis8310 构建脚本
#
# 用法：
#   ./build.sh                # 构建 → build/fdl2.bin
#   CROSS=/path/to/bin/arm-none-eabi- ./build.sh    # 指定工具链前缀
#
# 产物：
#   build/fdl2.bin    带 DHTB 头的完整镜像（可直接替换发布包里的 fdl2.bin）
#   build/fdl2.elf    带符号的 ELF（仅本地调试用，不随发布物分发）
#
# 依赖：arm-none-eabi-gcc（ARM GNU Toolchain）、python3
#
# 期望结果：build/fdl2.bin 大小 27716 字节，md5 7e47163dac221265bfbfc243a9e1677a
#           （与本文档记录的上游发布版逐字节一致；若不一致，先查工具链版本）
# =============================================================================

set -u

ROOT="$(cd "$(dirname "$0")" && pwd)"
SRC="$ROOT/src"
OUT="$ROOT/build"
PREFIX="${CROSS:-arm-none-eabi-}"
CC="${PREFIX}gcc"
OBJCOPY="${PREFIX}objcopy"
SIZE="${PREFIX}size"
PY="${PYTHON:-python}"

if ! command -v "$CC" >/dev/null 2>&1; then
    cat <<'EOF'
[!] 未找到 arm-none-eabi-gcc

安装方式（任选其一）：

  A. ARM 官方
     https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
     选 "arm-none-eabi" → 对应平台压缩包，解压后把 bin 目录加进 PATH

  B. xPack 版（免安装）
     https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases

  C. MSYS2
     pacman -S mingw-w64-x86_64-arm-none-eabi-gcc

已安装但路径不同，可用环境变量指定前缀：
     CROSS=/path/to/bin/arm-none-eabi- ./build.sh
EOF
    exit 1
fi

mkdir -p "$OUT"

# 说明：
#   -mcpu=cortex-a55  这颗 SoC 的核（UIS8310）
#   -marm             全程 ARM 指令（不是 Thumb）
#   -Os               以体积优先（最终镜像 ~27 KB）
#   -ffreestanding    裸机，不链接 libc
CFLAGS=(
    -mcpu=cortex-a55
    -marm
    -mfloat-abi=softfp
    -mfpu=neon
    -Os
    -ffreestanding
    -fno-builtin
    -fno-common
    -fno-stack-protector
    -fno-pic
    -ffunction-sections
    -fdata-sections
    -Wall -Wextra
    -Wno-unused-parameter
    -I"$SRC"
)

LDFLAGS=(
    -nostdlib
    -nostartfiles
    -T "$SRC/linker.ld"
    -Wl,--build-id=none
    -Wl,--gc-sections
)

SRCS=(
    "$SRC/start.S"
    "$SRC/main.c"
)

echo "[1/4] 编译"
for f in "${SRCS[@]}"; do
    o="$OUT/$(basename "${f%.*}").o"
    echo "      $(basename "$f")"
    "$CC" "${CFLAGS[@]}" -c "$f" -o "$o" || exit 1
done

echo "[2/4] 链接"
"$CC" "${CFLAGS[@]}" "${LDFLAGS[@]}" "$OUT"/*.o -o "$OUT/fdl2.elf" || exit 1

echo "[3/4] 提取二进制"
"$OBJCOPY" -O binary "$OUT/fdl2.elf" "$OUT/fdl2.raw" || exit 1

echo "[4/4] 打 DHTB 头"
"$PY" "$ROOT/tools/mkimage.py" pack "$OUT/fdl2.raw" "$OUT/fdl2.bin" || exit 1

echo
echo "================ 结果 ================"
"$SIZE" "$OUT/fdl2.elf" 2>/dev/null || true
echo
"$PY" "$ROOT/tools/mkimage.py" info "$OUT/fdl2.bin"
echo
echo "⚠️ 刷机前务必自行备份（BOOT1 里有原厂 FDL2，是最后的救砖手段）"
