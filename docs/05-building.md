# 05 · 构建、镜像格式与烧写

## 1. 工具链

| 项 | 要求 |
|---|---|
| 编译器 | `arm-none-eabi-gcc` |
| 验证版本 | **GCC 15.2.1**（xPack / ARM 官方均可） |
| Python | 3.8+（仅打包镜像用） |

```bash
./build.sh
# 或指定前缀
CROSS=/opt/arm-gnu-toolchain/bin/arm-none-eabi- ./build.sh
```

### 1.1 编译选项（`build.sh` 里）

| 选项 | 原因 |
|---|---|
| `-mcpu=cortex-a55` `-marm` | 这颗 SoC 的核；**ARM 状态**运行（不是 Thumb） |
| `-Os` | 体积优先，最终镜像 ~27 KB |
| `-ffreestanding -fno-builtin` | 裸机，没有 libc |
| `-fno-common` `-fno-stack-protector` `-fno-pic` | 裸机必需 |
| `-ffunction-sections -fdata-sections` + `--gc-sections` | 丢掉没用的函数 |
| `-nostdlib -nostartfiles -T src/linker.ld` | 不要启动文件，自己指定布局 |

### 1.2 可复现性

```
build/fdl2.bin   27 144 字节
md5              7bce7f456e2f2ce92a846ee725efe0e1
```

**同版本工具链应当逐字节一致。** 若不一致，先确认 GCC 版本；再确认没有改过 `src/`。

## 2. 镜像格式：DHTB

文件 = **512 字节头 + 负载**：

| 偏移 | 长度 | 内容 |
|---|---|---|
| `0x000` | 4 | magic `"DHTB"` |
| `0x004` | 4 | 版本 = `1` |
| `0x030` | 4 | **负载长度** = 文件长度 − `0x200` |
| 其余 | — | 全 0 |
| `0x200` | — | 代码 / 数据（链接基址 `0x80100200`） |

**没有校验和、没有签名。** 好处是改完不用重算；坏处是没有任何保护——写坏的镜像设备会直接执行。

`tools/mkimage.py`：

```bash
py tools/mkimage.py pack  build/fdl2.raw  build/fdl2.bin    # 打头
py tools/mkimage.py info  build/fdl2.bin                    # 校验查看
py tools/mkimage.py unpack build/fdl2.bin build/fdl2.raw    # 去头
```

`info` 会校验"声明的负载长度 == 实际长度"，并把 md5 打出来。

## 3. 烧写

### 3.1 用厂家的刷机包（推荐，最省事）

1. 把 `build/fdl2.bin` 复制进发布包，覆盖同名文件；
2. 设备进下载模式：**拔 USB → 长按电源 10 秒 → 按住 RESET 不放 → 插 USB → 松手**；
3. 双击原来的 `一键刷机.bat`。

> 初次使用建议只替换 `fdl2.bin`，**其它文件一个都别动**——这样出问题容易回退。

### 3.2 用自己的脚本（WinUSB / libusb）

需要先把设备驱动换成 **WinUSB**（例如用 Zadig），因为 `libusb` 与厂商的 COM 驱动互斥。

流程与厂家工具一致：

```
加载 FDL1 → 加载 FDL2 → CONNECT → 按分区 START/MIDST/END → RESET
```

> 注意：本项目的 USB 层**在主机断开一次后不会重新武装端点**，
> 所以每次测试前都要重新插拔（见 `docs/06-limitations.md`）。

### 3.3 写完后

`0x05 NORMAL_RESET` 在这颗平台上被实现为**关机**：

```
刷完 → 设备自动关机（灯全灭）→ 手动按电源 → 正常进系统
```

这不是 bug，是本平台没打通"自动重启"（见 `docs/06-limitations.md`）。

## 4. 救砖

只要不是把 BOOT0/BOOT1 写坏，**都能救**：

1. 设备进下载模式（按住 RESET 插 USB）；
2. 用**原厂完整包**重刷（不要用自己 dump 的镜像）；
3. 如果连 `VID_1782` 都枚举不出来：继续按住 RESET 上电，再试。

> 建议：第一次动手前，先用原厂工具把三个分区各读出来存一份（`read_part`），
> 那是你最后的退路。

## 5. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 编译出来的 md5 不是 `7e47163d…` | 工具链版本不同 / 改过源码 | 对比 GCC 版本；`git status` 看有没有改动 |
| 工具报 `FDL2 Exec timeout` | 没有回 `0x96 + DA_INFO`，或回了两帧 | 检查 `main()` 第 6 步 |
| 卡在 `Writing Part`、无任何输出 | 某个 16 KB 帧超过了 1000 ms | 检查是不是把重活塞进了写页路径 |
| 写完不关机 | `0x05` 的等待/写入顺序不对 | 见 `p0_poweroff()` 的三步 |
| 设备不枚举 | USB 初始化寄存器写错 / MMU 映射错 | 断电重进；确认 MMIO 段被映射成 strongly-ordered 且 AP=11 |
| 刷完起不来、但能进下载模式 | 分区内容错（OOB/ECC 或数据不对） | 用原厂包重刷，再逐步定位 |

## 6. 构建产物说明

| 文件 | 是否随发布物分发 |
|---|---|
| `build/fdl2.bin` | ✅ 唯一需要分发的文件 |
| `build/fdl2.elf` | ❌ 含符号表，仅本地调试 |
| `build/fdl2.raw` | ❌ 中间产物（无 DHTB 头） |
| `build/*.o` | ❌ |
