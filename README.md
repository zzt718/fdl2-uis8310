# fdl2-uis8310

面向 **展锐 UIS8310** 平台的**自研 FDL2**（Flash Download Loader 2）：一个可以在裸机上完成
NAND 擦除 / 写入 / 读取、并通过 BSL 协议与 PC 刷机工具通信的最小实现。

> **English**: An independent, from-scratch reimplementation of the FDL2 stage for Spreadtrum
> UIS8310 based devices. It initialises the NAND controller, implements the BSL command set
> (write / read / erase / reset), computes ECC in software, and brings up its own USB device
> stack — so it does not depend on any internal symbol of the boot ROM or of a specific FDL1.
>
> 27 716 bytes, 100 % self-contained, no libc, no vendor binary.

---

## 1. 这是什么

设备进下载模式时，PC 侧工具会分两级加载固件：

```
PC 工具  ──USB──>  BootROM  ──>  FDL1（厂家的引导下载器）  ──>  FDL2（真正干活的）
```

**FDL2 是唯一真正碰闪存的那一级**：擦块、写页、读页、按 BSL 协议应答主机。

本项目就是这一级的一个**独立实现**，交付形态仍是**一个 `fdl2.bin` 文件**：

- 放进厂家发布包的 `dl/` 同级目录，覆盖同名文件即可；
- 刷机工具、FDL1、驱动、bat 全部不用改；
- 也可以被自己写的刷机脚本直接加载（本项目用 `libusb` / WinUSB 验证过）。

## 2. 适用与不适用

| 项 | 说明 |
|---|---|
| **SoC** | 展锐 **UIS8310**（Cortex-A55，ARM 32 位状态运行） |
| **闪存** | **UNIM（至讯创新）UM19A8LISW**：SPI-NAND / SLC / 32 MB / 页 2048 / 块 128 KB（64 页）/ 备用区 78 B |
| **设备** | 蜂助手 MIBOX-660X 系列（`EM40` 电池款、`EM313A` 充电宝款） |
| **分区** | 只认 3 个：`KernelImg`（块 79–134）/ `UserImg`（135–203）/ `PreloadUdisk`（210–247） |
| **不适用** | 其它 SoC、其它几何参数的闪存、其它分区布局 —— 见 [docs/06-limitations.md](docs/06-limitations.md) |

> **请先确认型号与固件版本再刷。** 本实现不校验 Flash ID、不解析分区表，
> 它是"给已知机型用的专用工具"，不是通用刷机器。

## 3. 为什么会有这个项目

原厂 FDL2 是**闭源二进制**：不能审计、不能改、不能适配新料、出问题只能靠猜。
当需要（a）完全掌控刷写流程、（b）在没有任何官方支持的情况下排障、
（c）把这套能力长期维护下去时，唯一可行的办法是**自己写一个**。

本项目通过**逆向分析确定硬件接口**（寄存器、命令格式、时序），代码全部重写，
不包含、也不依赖任何第三方二进制代码；ECC 部分由**设备实测数据反推**得到（见 `docs/03-ecc.md`）。

## 4. 快速开始

### 4.1 构建

```bash
# 依赖 arm-none-eabi-gcc（ARM GNU Toolchain 或 xPack 版）与 python3
export PATH="/path/to/arm-none-eabi/bin:$PATH"
./build.sh
```

产物：

```
build/fdl2.bin   27 716 字节   md5 7e47163dac221265bfbfc243a9e1677a
```

**这个 md5 是构建可复现的校验点**：同样的工具链（GCC 15.2.1）应得到逐字节相同的结果。

### 4.2 刷写

把 `build/fdl2.bin` 覆盖到发布包里的 `fdl2.bin`，然后按原来的方式刷机。

> ⚠️ **刷机前请自行备份**。BOOT1 里通常还留着原厂 FDL2，那是最可靠的救砖手段。

### 4.3 验证

刷完正常应当：三分区写完 → 设备**自动关机** → **手动按电源**开机 → 正常进系统。

（本实现把 BSL `0x05 NORMAL_RESET` 实现为"关机"而不是"重启"，原因见
[docs/06-limitations.md](docs/06-limitations.md#1-reset-只能关机不能自动重启)。）

## 5. 功能清单

| 能力 | 状态 | 说明 |
|---|---|---|
| USB 设备层（自带） | ✅ | 控制器 `0x20200000`，EP `0x85`(IN)/`0x06`(OUT)，VID:PID `1782:4D00`，**不依赖 FDL1 内部函数** |
| BSL 帧收发（HDLC） | ✅ | `7E` 定界、`7D5E/7D5D` 转义、16 位反码校验 |
| `0x00` CONNECT / `0x04` EXEC | ✅ | exec 时回 `0x96 + DA_INFO`（声明 NAND 存储类型） |
| `0x01` START_DATA | ✅ | 支持 **8 字节地址式** 与 **76 字节分区名式**（UTF-16）两种载荷 |
| `0x02` MIDST_DATA | ✅ | 流式攒页，收满 2048 就写一页；单帧最大 **16 KB** |
| `0x03` END_DATA | ✅ | 尾页补 `0xFF`，并补写暂存的块内页 0/1 |
| `0x06` READ_FLASH | ✅ | 单次最多 16 KB，支持跨页拼接 |
| `0x0A` ERASE_FLASH | ✅ | 支持地址式 / 分区名式 |
| `0x05` NORMAL_RESET | ⚠️ | 实现为**关机**（本平台没打通"重启"） |
| 软件 ECC | ✅ | 78 字节 OOB，2 段 × 39 字节，224 位校验，运行时展开查表 |
| MMU + D/I-cache | ✅ | 1 MB 段映射；写页 3.58 ms/页（不开 cache 是 35 ms/页） |
| 自定义诊断命令 | ✅ | `0xE1`–`0xFF` 一批（读寄存器、读页、读 OOB、设参数…）|

## 6. 目录结构

```
.
├── build.sh               构建脚本（一条命令出镜像）
├── src/
│   ├── start.S            启动汇编（0x80100200 入口、栈、BSS 清零、跳 main）
│   ├── main.c             全部实现（约 4600 行，文件头有分区索引）
│   ├── fdl2.h             类型、寄存器宏、命令码
│   ├── ecc_tab.h          ECC 递推表（生成物，224 行种子 + 系数）
│   └── linker.ld          链接脚本（基址 0x80100000，代码从 +0x200 起）
├── tools/
│   ├── mkimage.py         打 / 拆 DHTB 头
│   └── gen_ecc_tab.py     由 ecc_map.npz 生成 src/ecc_tab.h
└── docs/
    ├── 01-architecture.md  总体架构、启动流程、内存布局
    ├── 02-nand-controller.md  NFI 初始化与读/写/擦（含四个致命参数）
    ├── 03-ecc.md           OOB 结构、ECC 映射与递推压缩
    ├── 04-usb-bsl.md       USB 设备层与 BSL 协议、时间预算
    ├── 05-building.md      构建细节、镜像格式、烧写与救砖
    ├── 06-limitations.md   已知缺陷与未实现项（如实）
    └── 07-porting.md       移植到别的 SoC / 闪存要改什么
```

## 7. 从哪开始读

| 你想做什么 | 先看 |
|---|---|
| 搞懂整体怎么跑起来 | [docs/01-architecture.md](docs/01-architecture.md) |
| 改 NAND 读写、排查写坏页 | [docs/02-nand-controller.md](docs/02-nand-controller.md) |
| 搞懂 ECC / OOB | [docs/03-ecc.md](docs/03-ecc.md) |
| 接自己的刷机工具 | [docs/04-usb-bsl.md](docs/04-usb-bsl.md) |
| 编译、烧写、救砖 | [docs/05-building.md](docs/05-building.md) |
| 知道它有哪些坑 | [docs/06-limitations.md](docs/06-limitations.md) |
| 换芯片 / 换机型 | [docs/07-porting.md](docs/07-porting.md) |

## 8. 已测试

| 平台 | 结果 |
|---|---|
| 蜂助手 MIBOX-660X（充电宝款，`EM313A`） | ✅ 三分区完整刷写（19.24 MB / 42 秒）+ 开机正常 |
| 自研刷机脚本（WinUSB） | ✅ 读 19.24 MB / 11.8 s，写 19.24 MB / 35.3 s |
| 整盘 4376 页 ECC 对拍 | ✅ 与设备实测 OOB 逐页一致 |

未在其它机型 / 其它闪存上验证。**请不要默认它能用在别的设备上。**

## 9. 已知限制（摘要）

- 参数**硬编码**：页 2048 / 64 页每块 / OOB 78 B，换闪存即失效；
- 只支持 3 个分区，不解析分区表、**不做 FTL / 坏块管理**；
- `reset` 只能关机，不能自动重启；
- 主机断开一次后，USB 端点不会重新武装（需重新插拔）；
- 没有串口输出，排障只能靠 USB 诊断命令。

完整清单（含原因与可能的修法）见 [docs/06-limitations.md](docs/06-limitations.md)。

## 10. 参与贡献

欢迎 fork。改动前请先看 `docs/06-limitations.md` 与各文档里的"踩坑记录"——
里面很多条是**实测证伪**的结论，重复试一遍只会浪费你的时间。

提交 PR 时请附：

1. 改了哪一层（USB / BSL / NAND / ECC / 参数）；
2. **构建后的 `md5sum build/fdl2.bin`**（便于判断是否影响了已有路径）；
3. 上机验证条件（机型、板号、闪存、刷了哪些分区）。

## 11. 许可与来源

- 本项目代码：**MIT**，见 [LICENSE](LICENSE)。
- 硬件接口（寄存器序列、命令格式、时序）通过逆向分析确定；**未复制任何第三方代码**。
- 参考过 `spreadtrum_flash` 项目中公开的 USB 控制器寄存器序列（该项目为 **Unlicense / 公有领域**），
  本项目的 USB 层在此基础上有较大改写（自带 HDLC 分帧、32 KB 收发缓冲、与 FDL1 解耦）。
- 本项目与原厂 `fdl2.bin` 无任何代码关系：前者 27 716 字节 / ARM 指令集 / GCC 编译，
  后者为闭源 Thumb 二进制。

## 12. 免责声明

刷机会改动设备分区。**使用本工具造成的一切后果由使用者自负**，作者不提供任何担保。
请务必先备份、先在小批量设备上验证，并保留原厂 FDL2 作为救砖手段。
