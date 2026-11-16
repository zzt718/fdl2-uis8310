# 01 · 总体架构

## 1. 运行上下文：FDL2 是被"塞进去再跳过来"的

```
① 设备进下载模式（按住 RESET 插 USB）→ 芯片内 BootROM 枚举 VID:PID = 1782:4D00
② PC 工具把 FDL1 下载到 SRAM 0x00006200，跳过去
③ FDL1 建立基本环境，接收 FDL2：把 fdl2.bin 原样放到 0x80100000
④ FDL1 执行 EXEC_DATA 后**不再回应答**，直接跳到 0x80100200
⑤ 我们的 start.S 从这里开始跑
```

关键点：

- **FDL2 必须自己补发 EXEC 的应答**（`0x96` + DA_INFO），否则主机的 `send_and_check()`
  只等 `0x80`，会报 `FDL2 Exec timeout`。
- 文件前 0x200 字节是 **DHTB 头**（magic / 版本 / 负载长度），代码从 `+0x200` 开始，
  所以链接脚本的起点是 `0x80100200`。
- 加载后设备**不重新枚举**：USB 控制器是 FDL2 自己重新初始化的（见 `docs/04-usb-bsl.md`）。

## 2. 地址空间与内存布局

| 区域 | 地址 | 说明 |
|---|---|---|
| 栈 | `0x80000000`–`0x8000FFFF` | 64 KiB，SP 初值 `0x80010000`（向下生长） |
| DHTB 头 | `0x80100000`–`0x801001FF` | 512 B，由 `tools/mkimage.py` 打包时生成 |
| 代码 / 数据 | `0x80100200` 起 | `.text` + `.rodata`（含 ECC 种子）+ `.data` |
| BSS | `0x80106C44`–`0x801615C0` | 约 366 KB，其中 `ecc_tab`（229 KB）占大头 |
| `OBUF` | `0x80176000` | OOB 缓冲区（DMA） |
| `XBUF` | `0x80176400` | 备用区 |
| `DBUF` | `0x80298820` | 页缓冲区（2048 B，DMA 目标） |

> 这些固定地址不是随便挑的：它们取自该平台工程惯用的缓冲区布局，
> 落在同一段可用 RAM 内、且互不重叠（已验证，见 `docs/06-limitations.md` 的"内存余量"一节）。

实际占用（`arm-none-eabi-size build/fdl2.elf`）：

```
text 27 140   data 64   bss 366 016
```

## 3. 启动顺序（顺序本身有强约束）

```
start.S
 ├─ 保存 BROM 传来的 r0（存 __boot_arg，必须在 .data 里，否则被清 BSS 冲掉）
 ├─ 关 I-cache
 ├─ 放开 CP10/CP11 与 FPU
 ├─ 进 SVC、关中断
 ├─ SP = __stack_top (0x80010000)
 ├─ 清 BSS
 └─ bl main
```

`main()` 的前六步，**顺序不能换**：

| 步 | 动作 | 为什么必须在这个位置 |
|---|---|---|
| 1 | `mmu_init()` | 之后所有 MMIO / RAM 访问都走 MMU。**必须先建映射再碰任何外设** |
| 2 | `usb_init()` | 自带 USB 层要重新配置端点；晚了会错过主机重连窗口 |
| 3 | `nand_ensure()` | NAND 上电 + PHY/NFI 初始化（只做一次） |
| 4 | `flash_read_page(0)` | **启动自预热**：加载后第一个 NAND 操作可能丢一拍，用一次只读页读把它消化掉 |
| 5 | `ecc_build_pub()` | 把 8193 行 ECC 表展开到 RAM。**必须赶在回 ACK 之前**，否则第一个 16 KB 帧会超时（主机只等 1000 ms） |
| 6 | 回 `0x96 + DA_INFO` | 告诉主机"我是 NAND 存储"，否则工具会按 eMMC 处理 |

然后进入 `for(;;)` 的 BSL 命令循环。

> 第 5 步曾经放在写页路径里（`ecc_fill_oob` 第一句），结果厂家工具写第一个 16 KB 帧就超时。
> 这是一次真实的排障结论，见 `docs/04-usb-bsl.md#2-时间预算1000-ms-是硬约束`。

## 4. main.c 的模块划分

```
┌ 1. 基础工具      mem_copy / mem_set / be32 / put_be32 / add_cs
├ 2. 电源与复位    p0_apb_wr / p0_poweroff            ← 只写 APB/PMIC 寄存器
├ 3. cache 维护    cache_inv / cache_clean / cache_inv2
├ 4. NFI 控制器    nfc / nf_wait / phy_init / nfi_init / nf_one / nf_exec
├ 5. ECC           ecc_build / ecc_fill_oob
├ 6. USB 设备层    usb_init / usb_ep_config / usb_send_raw / usb_recv_frame
├ 7. BSL 应答      send_resp
├ 8. NAND 抽象     nand_power_on / flash_read_page / flash_write_page
│                  wr_page_now / flush_hold / flash_prep_row
├ 9. MMU           mmu_init
└ 10. 主循环       main
```

各层之间的依赖是单向的：`main → 命令处理 → NAND 抽象 → nf_exec → nf_one → NFI 寄存器`；
USB 层与 NAND 层互不知道对方存在，中间只靠 `p_rx` / `p_tx` 两个缓冲区。

## 5. 一次刷机的完整命令流

```
主机                                    固件
────────────────────────────────────────────────────────────────
CONNECT (0x00)                    →    ACK
START_DATA (0x01)  分区名/地址      →    ACK，初始化写任务
  MIDST_DATA (0x02) 16 KB × N      →    ACK（每帧 8 页）
  ...                                   攒满 2048 就写一页
END_DATA (0x03)                   →    ACK，补尾页 + 补写页 0/1
（下一个分区重复）
EXEC_DATA (0x04)                  →    （FDL1 阶段，不会到我们）
NORMAL_RESET (0x05)               →    ACK → 关机
```

写一个分区时，固件内部：

```
MIDST_DATA 字节流
   ↓ 逐字节进 DBUF（g_wr_fill 计数）
凑满 2048 字节
   ↓ wr_page_now()
      ├─ ECC：ecc_fill_oob(DBUF → OBUF)      （主机没给 OOB 时）
      ├─ 块首：flash_erase_block()
      ├─ 页 0/1：先扣下（g_hold），等块内其他页写完再补
      └─ 4 个 512 字节扇区，每扇区 LOAD + EXECUTE
```

## 6. 写任务的状态机

| 变量 | 含义 |
|---|---|
| `g_wr_active` | 是否处在"写分区"模式（否则 `MIDST_DATA` 当 RAM 载入） |
| `g_wr_base` | 起始**线性页号** = `blk0 × 64` |
| `g_wr_n` | 已写完的页数 |
| `g_wr_fill` | `DBUF` 里已攒的字节数（0–2047） |
| `g_wr_blk` | 当前已擦除的块号；`0xFFFFFFFF` 表示还没擦 |
| `g_hold[2]` / `g_hold_has` | 被扣下的块内页 0/1 的内容与标志 |

**页 0/1 必须最后写**是硬约束（program disturb，写后面的页会扰动块首两页），
由 `flush_hold()` 在"块写满"或"`END_DATA` 收尾"时补写。

## 7. 三个关键设计决策

| 决策 | 原因 |
|---|---|
| **自带 USB 层**，不调用 FDL1 留下的函数 | FDL1 里的发送/接收函数是**绝对地址**（换一个 FDL1 就崩），且单次接收上限 12 800 B，吃不下 16 KB 帧 |
| **软件算 ECC** | 本平台控制器硬件 ECC 的两条路都实测失败（见 `docs/03-ecc.md`），软件算可逐页验证 |
| **开 MMU + D/I-cache** | 不开时 229 KB 的 ECC 表每次访问都直通 DDR（~460 ns/次），写页 35 ms/页；开后 3.58 ms/页 |

这三条都不是"优化"，是**能不能用**的问题，详见对应文档。
