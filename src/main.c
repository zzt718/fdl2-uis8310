/*
 * =============================================================================
 * main.c — UIS8310 自研 FDL2 主实现
 *
 * 文件结构（自上而下，按函数名索引）：
 *
 * 1. 基础工具 mem_copy / mem_set / be32 / put_be32 / add_cs
 * 2. 电源与复位 p0_apb_wr / p0_关机
 * 3. cache 维护 cache_inv / cache_clean / cache_inv2
 * 4. NFI 控制器 nfc / nf_wait / phy_init / nfi_init / nf_one / nf_exec
 * 5. ECC ecc_build / ecc_fill_oob
 * 6. USB 设备层 usb_init / usb_ep_config / usb_send_raw / usb_recv_frame
 * usb_handler / usb_sys_events / usb_getbyte
 * 7. BSL 应答 send_resp
 * 8. NAND 抽象 nand_power_on / flash_read_page / flash_write_page
 * wr_page_now / flush_hold / flash_prep_row
 * 9. MMU mmu_init
 * 10. 主循环 main（BSL 命令分发 + 自定义诊断命令）
 *
 * 阅读建议：
 * docs/01-architecture.md → docs/02-nand-controller.md → docs/03-ecc.md
 * → docs/04-usb-bsl.md → 再回来看本文件
 * =============================================================================
 */

#include "fdl2.h"

u8 g_rx[RX_BUF_SIZE];
u8 g_tx[TX_BUF_SIZE];
u8 g_xfer[XFER_BUF_SIZE];

/*
 * BROM 跳转到 FDL2 时传进来的 r0（由 start.S 保存）。
 * 必须放 .data，不能放 .bss —— 启动代码会清 BSS，会把值冲掉。
 */
u32 __boot_arg __attribute__((section(".data"))) = 0;

/* ---------------------------------------------------------------- 工具 */
void mem_copy(void *dst, const void *src, u32 n)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    while (n--)
 *d++ = *s++;
}

void mem_set(void *dst, u8 v, u32 n)
{
    u8 *d = (u8 *)dst;
    while (n--)
 *d++ = v;
}

/* ---------------------------------------------------------------- 入口 */

/*
 * ================================================================
 * P0 探针版（-DP0_PROBE）
 *
 * 目的：在没有串口、没有 USB 驱动的情况下，证明「自研二进制能被
 * FDL1 加载并执行」。
 *
 * 可观测信号：设备主动关机 → USB 设备从系统消失。
 *
 * 路径全部来自控制器关机流程，已实测：
 * 1. 轮询 0x40608E14 的 bit3，直到清零
 * 2. 读 0x40608C20，置 bit0，写回
 * 3. 写之前等 0x40600000+0x30 的 bit11（busy）清零
 *
 * 安全性：这条路径就是控制器关机 用的，不碰 flash、不碰引导区。
 * ================================================================
 */



static int p0_apb_wr(u32 addr, u32 val)
{
    u32 n = 1000;
    while ((*(volatile u32 *)0x40600030u & (1u << 11)) && --n)
        ;
    if (n == 0)
        return -1;
 *(volatile u32 *)addr = val;
    return 0;
}

static void p0_poweroff(void)
{
    u32 n;
    for (n = 0; n < 2000000u; n++) {
        if (!(*(volatile u32 *)0x40608E14u & 0x8u))
            break;
    }
    p0_apb_wr(0x40608C20u, *(volatile u32 *)0x40608C20u | 1u);
    for (;;)
        ;
}





/*
 * ================================================================
 * P65：按控制器的**完整**寄存器序列读页（三档输出）
 *
 * 之前一直失败的原因（2026-10-31 复盘）：
 * 早期只实现了传输流程的中段，漏掉全部前置配置。三处致命错误：
 *
 * 1) 数据 DMA 描述符在 0x200/0x204，不是 0x210/0x214！
 * 0x200 = 0 0x204 = 数据缓冲区
 * 0x208/0x20C = OOB 描述符（无 OOB 时 0xFFFFFFFF）
 * 0x210/0x214 = 第三描述符（0 与 0x80176400）
 * ⇒ 我一直把 OOB 描述符当数据 DMA 用，所以数据只进了 FIFO
 * （0x19C），内存永远没被写。这完美解释 P49~P64 的全部现象。
 *
 * 2) 0x004 配置 = 0x8108 | 0x10 = 0x8118，不是 2。
 * 0x8108 = 基础值；|0x10 = "只有数据缓冲"；|0x30 = "数据+OOB"
 * 0x04 的高字节 = (页数 - 1)，单页时为 0。
 *
 * 3) 0x0F8 = 0x38 | 2 = 0x3A（不是 |= 2）。
 *
 * 完整序列（严格按指令顺序）：
 * FIFO_IDX = 0
 * 0x200=0 0x204=DBUF
 * cmd 0x32, col_hi, col_lo, 0x2200 (col = 页大小 = 2048)
 * 0x208=0xFFFFFFFF 0x20C=0xFFFFFFFF
 * cmd 0xE000
 * 0x004=0x8118 0x00C=0
 * 0x210=0 0x214=XBUF
 * cmd 0x10, row_hi, row_mid, row_lo, 0xE000, 0x0F, 0xC0, 0x1000, 0xE201, 0xF000
 * 清状态
 * 0x000=0 0x008=0x3000 0x0F8=0x3A 0x000=0
 * cache_inv(DBUF,2048) cache_inv(OBUF,64)
 * 0x000 |= 1
 * wait
 *
 * 三档输出（一次插拔区分三种结果）：
 * 复位时刻 含义
 * ~0.3s 没读到任何东西（DMA 仍没写）
 * ~1s 以上 有数据写入，但头不是分区魔数（行地址/页大小要调）
 * ~5s 以上 ★★★ 读到分区头（DHTB 或 f3f2f1f0），读页彻底通了
 * 50s 不复位 挂死
 * ================================================================
 */
#define NFI_BASE        0x21A00000u
#define NFI_START       (*(volatile u32 *)(NFI_BASE + 0x000u))
#define NFI_CFG         (*(volatile u32 *)(NFI_BASE + 0x004u))
#define NFI_LEN         (*(volatile u32 *)(NFI_BASE + 0x008u))
#define NFI_STAT        (*(volatile u32 *)(NFI_BASE + 0x010u))
#define NFI_ID          (*(volatile u32 *)(NFI_BASE + 0x19Cu))
#define NFI_FIFO        (NFI_BASE + 0x220u)
#define FIFO_IDX        (*(volatile u32 *)0x80173ED0u)

/* ★★★ 缓冲区地址（2026-11-03 定案，只认这一处，别再在下面重复 #define）
 *
 * DBUF = 0x80298820 —— 唯一实测能收到 DMA 数据的地址。
 * 2026-11-03 曾同时定义 0x80174000，后一个生效，导致脚本读错缓冲区、
 * 全部读回 0x00，白跑一轮。已删除重复定义。
 * OBUF/XBUF 沿用控制器的 0x80176000 / 0x80176400。*/
#define DBUF            0x80298820u
#define OBUF            0x80176000u
#define XBUF            0x80176400u

#define MAGIC_LZMA      0xF0F1F2F3u   /* f3 f2 f1 f0 */
#define MAGIC_DHTB      0x42544844u   /* "DHTB" */

static void nf_delay(u32 n) { while (n--) __asm__ volatile("nop"); }









static void cache_inv(u32 addr, u32 len)
{
    u32 a = addr & ~0x3Fu, e = addr + len + 0x3Fu;
    while (a < e) {
        __asm__ volatile("mcr p15, 0, %0, c7, c6, 1" :: "r"(a) : "memory");
        a += 64u;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

static void cache_clean(u32 addr, u32 len)
{
    u32 a = addr & ~0x3Fu, e = addr + len + 0x3Fu;
    while (a < e) {
        __asm__ volatile("mcr p15, 0, %0, c7, c10, 1" :: "r"(a) : "memory");
        a += 64u;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}




















/* ================================================================
 * P81：★ 列地址修正为 0（之前用 2048 = 备用区起始，越界导致引擎卡死）
 *
 * 关键发现（重读 分支）：
 * 8010984A ldrh r0,[r4,#0xc] ; 页大小 2048
 * 8010984C ldrh r1,[r4,#0x26] ; OOB 64
 * 8010984E add r0, r1 ; 2112
 * 80109850 muls r6, r0, r6 ; col = r6 * 2112
 * => r6=0 时 col = 0
 *
 * 之前误用另一分支（arg7!=0 路径）的公式：
 * col = 页大小 + r6*(页大小+OOB) = 2048 ← 备用区起始列，越界
 *
 * 完整序列（arg6=缓冲, arg7=0, arg4=0）：
 * 0x200=0 0x204=缓冲
 * cmd 0x32, 0x00, 0x00, 0x2200
 * 0x208=0xFFFFFFFF 0x20C=0xFFFFFFFF
 * cmd 0xE000
 * 0x04=0x8118 0x0C=0 0x210=0 0x214=XBUF
 * cmd 0x10, row>>16, row>>8, row, 0xE000, 0x0F, 0xC0, 0x1000, 0xE201, 0xF000
 * nf_clr ; 0x00=0 ; 0x08=0x3000 ; 0x0F8=0x3A ; 0x00=0
 * cache_inv ; 0x00|=1 ; wait
 * ================================================================ */

static void cache_inv2(u32 addr, u32 len)
{
    u32 a = addr & ~0x3Fu, e = addr + len + 0x3Fu;
    while (a < e) {
        __asm__ volatile("mcr p15, 0, %0, c7, c6, 1" :: "r"(a) : "memory");
        a += 64u;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}



static void nfc(u32 v)
{
    u32 i = FIFO_IDX;
    volatile u32 *q = (volatile u32 *)(NFI_FIFO + 4u * (i >> 1));
    if (i & 1u) *q = (*q & 0xFFFFu) | (v << 16);
    else        *q = (*q & 0xFFFF0000u) | (v & 0xFFFFu);
    FIFO_IDX = i + 1u;
}

/* ★★★ 状态位：从控制器逐条算出来（不要凭记忆）
 *
 * 80108A24 ldr.w r0,[ip,#0x10] ; ip = 0x21A00000 -> 读 0x21A00010
 * 80108A28 lsls r1, r0, #7 ; 结果 bit31 <- r0 的 bit24
 * 80108A2A bpl ... ; N=1 说明 bit24 置位
 * 80108A2C orr r4, r4, #1 ; 完成标志
 * 80108A30 lsls r0, r0, #4 ; 结果 bit31 <- r0 的 bit27
 * 80108A34 orr r4, r4, #8 ; 错误标志（置位立即退出）
 *
 * ⇒ 完成 = bit24 (0x01000000)，错误 = bit27 (0x08000000)。
 * 注意：不是 bit25 / bit28（lsls #7 移的是 7 位，31-7=24）。
 */
static int nf_wait(void)
{
    u32 n = 400000u;

    while (n--) {
        u32 st = NFI_STAT;
        if (st & 0x08000000u) {          /* bit27 错误 */
 NFI_START = 2u; /* ★ 控制器出错路径：中止命令 */
            NFI_STAT |= 0xF00u;
            return 5;
        }
        if (st & 0x01000000u) return 0;  /* bit24 完成 */
    }
 NFI_START = 2u; /* ★ 控制器超时路径：中止命令 */
    NFI_STAT |= 0xF00u;
    return 1;
}

static void phy_init(void)
{
 *(volatile u32 *)0x402E0000u |= 0x100000u;
 *(volatile u32 *)0x402E00B0u |= 0x1000u;
 *(volatile u32 *)0x402A0024u |= 0x1000u;
 *(volatile u32 *)0x402A0194u = 0u; *(volatile u32 *)0x402A0174u = 0u;
 *(volatile u32 *)0x402A0160u = 0u; *(volatile u32 *)0x402A015Cu = 0u;
 *(volatile u32 *)0x402A0184u = 0u; *(volatile u32 *)0x402A0164u = 0u;
 *(volatile u32 *)0x402A0158u = 0u; *(volatile u32 *)0x402A0154u = 0u;
 *(volatile u32 *)0x402A0180u = 0u; *(volatile u32 *)0x402A018Cu = 0u;
 *(volatile u32 *)0x402A0190u = 0u; *(volatile u32 *)0x402A0188u = 0u;
 *(volatile u32 *)0x402A017Cu = 0u; *(volatile u32 *)0x402A0168u = 0u;
 *(volatile u32 *)0x402A016Cu = 0u; *(volatile u32 *)0x402A0178u = 0u;
 *(volatile u32 *)0x402A0170u = 0u;
 *(volatile u32 *)0x402A0594u = 0x300000u; *(volatile u32 *)0x402A0574u = 0x300080u;
 *(volatile u32 *)0x402A0560u = 0x300000u; *(volatile u32 *)0x402A055Cu = 0x300000u;
 *(volatile u32 *)0x402A0584u = 0x300000u; *(volatile u32 *)0x402A0564u = 0x300000u;
 *(volatile u32 *)0x402A0554u = 0x300000u; *(volatile u32 *)0x402A0580u = 0x300000u;
 *(volatile u32 *)0x402A058Cu = 0x300000u; *(volatile u32 *)0x402A0590u = 0x300000u;
 *(volatile u32 *)0x402A0588u = 0x300000u; *(volatile u32 *)0x402A057Cu = 0x300000u;
 *(volatile u32 *)0x402A0568u = 0x300000u; *(volatile u32 *)0x402A056Cu = 0x300000u;
 *(volatile u32 *)0x402A0578u = 0x300000u; *(volatile u32 *)0x402A0570u = 0x300000u;
 *(volatile u32 *)0x402A0558u = 0x301000u;
}

/* ----------------------------------------------------------------
 * ★★★ NAND 初始化 —— 按控制器的工作方式实现
 *
 * 之前的版本漏了 4 处，其中两处致命：
 * 1. NFI[0x004] = 2 —— 完全没写
 * 2. NFI[0x008] = 0x3000 —— 完全没写
 * 3. NFI[0x0EC] = 0x00040002 —— 只写了中间的临时值 0xC80
 * （控制器先写 0xC80 占位，最后再覆盖成 0x00040002）
 * 4. ★ 芯片复位命令序列 0xFF / 0xE000 / 0x0F / 0xC0 /
 * 0x1000 / 0xE001 / 0xF000 + NFI[0x000]=1 + nf_wait
 * —— 完全没做。芯片从没被复位过，状态未知。
 *
 * 返回 0 = 成功。
 * ---------------------------------------------------------------- */
static int nfi_init(void)
{
 /* --- 14 个基线寄存器（字面量池 起） --- */
 *(volatile u32 *)(NFI_BASE + 0x014u) = 0x3A493146u; /* TIMING */
 *(volatile u32 *)(NFI_BASE + 0x018u) = 0x192A0000u; /* TIMING2 */
 *(volatile u32 *)(NFI_BASE + 0x034u) = 0x81000000u;
 *(volatile u32 *)(NFI_BASE + 0x038u) = 0x0003000Cu;
 *(volatile u32 *)(NFI_BASE + 0x0B0u) = 0x1004u;
 *(volatile u32 *)(NFI_BASE + 0x0B4u) = 0x1004u;
 *(volatile u32 *)(NFI_BASE + 0x0B8u) = 0x4013u;
 *(volatile u32 *)(NFI_BASE + 0x0BCu) = 0x00400010u;
 *(volatile u32 *)(NFI_BASE + 0x0DCu) = 6u;
 *(volatile u32 *)(NFI_BASE + 0x0E0u) = 0x100u;
 *(volatile u32 *)(NFI_BASE + 0x0E4u) = 0x100u;
 *(volatile u32 *)(NFI_BASE + 0x0E8u) = 0xBFu;
 *(volatile u32 *)(NFI_BASE + 0x0ECu) = 0xC80u; /* 临时值 */
 *(volatile u32 *)(NFI_BASE + 0x180u) = 0x680u;
 *(volatile u32 *)(NFI_BASE + 0x188u) = 3u;
 *(volatile u32 *)(NFI_BASE + 0x12Cu) = 0x548Au;

 /* --- 配置 / 长度（控制器起） --- */
    NFI_CFG = 2u;                                        /* CE=0，无 (ce<<9) */
    NFI_LEN = 0x3000u;
 *(volatile u32 *)(NFI_BASE + 0x0ECu) = 0x00040002u; /* ★ 最终值，覆盖 0xC80 */
 *(volatile u32 *)(NFI_BASE + 0x0F8u) |= 2u;

 /* --- ★ 芯片复位命令序列 --- */
    FIFO_IDX = 0u;
    nfc(0xFFu);      /* RESET */
    nfc(0xE000u);
    nfc(0x0Fu);
    nfc(0xC0u);
    nfc(0x1000u);
    nfc(0xE001u);
    nfc(0xF000u);

 NFI_STAT |= 0xF00u; /* = 控制器清状态 */
    NFI_START = 1u;
    return nf_wait();
}

/* ----------------------------------------------------------------
 * ★★★ 读一页 —— 按控制器的工作方式实现
 *
 * 关键认知（之前一直搞错的地方）：
 * 控制器那个传输函数有两个分支，判据是 arg6(buf) 是否为 0：
 * buf == 0 → （裸读分支，**从不设置描述符**，是死代码）
 * buf != 0 → （完整路径：设描述符A + 发 0x32 列地址前缀）
 * 真调用者 传的 buf = 0x80298820（非 0）⇒ 走的是完整路径。
 *
 * 完整路径的寄存器序列（逐条，地址均已核对）：
 * NFI[0x200] = 0 NFI[0x204] = buf ; 描述符A
 * NFI[0x208] = -1 NFI[0x20C] = -1 ; 描述符B 禁用
 * cmd 0x32, col_hi, col_lo, 0x2200 ; 0x2000=描述符A有效
 * cmd 0xE000
 * NFI[0x004] = cfg NFI[0x00C] = 0
 * NFI[0x210] = 0 NFI[0x214] = 1
 * cmd 0x10, row>>16, row>>8, row, 0xE000, 0x0F, 0xC0, 0x1000, 0xE201, 0xF000
 * 清状态 ; = STAT |= 0xF00
 * NFI[0x000] = burst ; ★ 是赋值，不是 |=
 * NFI[0x008] = 0x3000 | (psiz-1) | (lenhi << 24) ; ★ 少了 psiz-1
 * NFI[0x0F8] = 0x38 | 2 = 0x3A ; ★ 少了 0x38
 * NFI[0x000] = burst
 * cache_inv(buf, len)
 * NFI[0x000] = burst | 1
 * nf_wait()
 *
 * mode: 0 = 完整路径（带 0x32 前缀） 1 = 不带 0x32 前缀（对照）
 * ---------------------------------------------------------------- */

/* ================================================================
 * ★★★ 通用 NFI 执行器 —— 变量隔离
 *
 * cmdset：
 * 0 = 用 mode 位图自由组合（做二分定位用）
 * 1 = 控制器那套：cmd 0x0F,0xC0,0x1000,0xE000,0xF000
 * 2 = 芯片复位：cmd 0xFF,0xE000,0x0F,0xC0,0x1000,0xE001,0xF000
 * 3 = ReadID：cmd 0x9F,0,0x1000 ×4,0xE000,0xF000
 *
 * mode（仅 cmdset==0 有效）—— 读页路径独有的 7 个步骤，各自独立开关：
 * 0x01 F_DESC 写描述符 0x200/0x204/0x208/0x20C
 * 0x02 F_DMA3 写 0x00C=0 / 0x210=0 / 0x214=1
 * 0x04 F_LEN 0x008 = 0x3000 | (psiz-1) | (lenhi<<24)
 * 0x08 F_F8 0x0F8 = 0x3A
 * 0x10 F_START0 启动前把 NFI[0x000] 写 0 两次
 * 0x20 F_COL 发 0x32,col_hi,col_lo,0x2200 前缀
 * 0x40 F_RDCMD 用读页命令 0x10...（否则用 0x0F,0xC0,0x1000,0xE000,0xF000）
 *
 * startmode：
 * 0 = NFI[0x000] = 1 （裸写）
 * 1 = NFI[0x000] = (读回值 & ~1)|1 （读-改-写）
 * ================================================================ */
#define F_DESC   0x001u  /* 写描述符 0x200/0x204/0x208/0x20C */
#define F_DMA3   0x002u  /* 写 0x00C=0 / 0x210=0 / 0x214=1 */
#define F_LEN    0x004u  /* 0x008 = 0x3000 | (psiz-1) | (lenhi<<24) */
#define F_F8     0x008u  /* 0x0F8 = 0x3A */
#define F_START0 0x010u  /* 启动前把 NFI[0x000] 写 0 两次 */
#define F_COL    0x020u  /* 发 0x32,col_hi,col_lo,0x2200,0xE000 前缀 */
#define F_RDCMD  0x040u  /* （保留） */
#define F_DST    0x080u  /* 0x214 写 DBUF（而不是 1）*/
#define F_DSTB   0x100u  /* 描述符B 也指向 DBUF（而不是 -1 禁用）*/
#define F_NOECC  0x200u  /* NFI[0x0EC] = 0（关 ECC）*/
#define F_F8B    0x400u  /* 0x0F8 写 0x02（而不是 0x3A）*/
#define F_F8C 0x800u /* 0x0F8 写 0x403A（bit14，控制器用的）*/
#define F_START8 0x1000u /* 启动写 NFI[0]=8（bit3），不是 1 */
#define F_LEN2   0x2000u /* NFI[0x008] = psiz-1（不带 0x3000 基）*/
#define F_F82 0x4000u /* 0x0F8 |= 2（读-改-写，控制器ReadID 用的）*/
/* ★ 0x0F8 = 0x7A = 0x08 | 0x30 | 0x40 | 0x02
 * 控制器读页 ：基值 0x08（bit15=0 读方向）
 * | 0x30（双缓冲）| 0x40（dev[0x17]=1）| 2（使能）
 * ★ 之前一直用 0x3A —— 少了 bit6(0x40)，这就是读不出数据的直接原因之一 */
#define F_F87A   0x8000u
/* ★ 忠实模式：按 spec[64] 精确算出的三个寄存器值
 * NFI[0x00C] = (planes-1)<<24 | dev[0x1A]<<16 | dev[0x19] = 0x01000004
 * NFI[0x000] = burst = nf_map(dev[0x18]=0x10) = 0x2800，然后 |= 1
 * dev 已由 spec[64] 原始字节逐条映射算出，不再靠猜 */
#define F_FAITH  0x10000u
/* ★ 按扇区循环时用：描述符 A 指向 DBUF + col，而不是 DBUF。
 * 实测证明整页 2048 单次搬会卡死（NFI[0x004] 高字节 = 扇区数-1，
 * 与 0x3800/0x2800 的单扇区配置冲突），所以改成 4×512 循环，
 * 每轮必须让描述符指向缓冲区里的对应偏移。 */
#define F_DSTO   0x20000u
/* ★★★★★ 0x40000 = 把 lenhi 参数**原样**写进 NFI[0x008]（不位移）。
 *
 * 控制器写页 里 NFI[0x008] 是拼出来的三块：
 * · bits 0-15 : 0x3000 | (页大小-1) → 0x37FF
 * · bits 16-23 : (0x03A4 + dev[38]) → 常量池 的值 + dev[38]<<16
 * · bits 24-31 : dev[27] → `orr r0, r1, lsl #24`
 * 我们的 F_LEN 只做了 `(lenhi<<24)`，把中间那段 0x03A4 完全漏掉了。
 * 有了这个标志就能从主机侧一次扫完所有候选值，不必反复改固件重烧。 */
#define F_LENRAW 0x40000u
/* ★ 描述符 B 指向 OBUF（0x80176000，备用区缓冲区），而不是 DBUF。
 * 控制器写页的第二次 PROGRAM LOAD（0x34 + 0x4200）就是从描述符 B 取
 * 备用区字节：`mla r1, r5, r2, r8`，r8 = 调用方传的 spare 缓冲区。 */
#define F_DSTOB  0x80000u
/* ★ 事务开始前把 OBUF 前 512 字节填 0xFF（控制器默认 spare 就是全 FF）。 */
#define F_OBFF   0x100000u
/* ★ 不要用 0xAA 哨兵预填 DBUF 前 64 字节。**写页事务必须带这个标志**，
 * 否则会把要写入 flash 的页数据前 64 字节改成 0xAA。 */
#define F_NOMARK 0x200000u
/* ★★★ NFI[0x000] = 0x1001（控制器的启动值）。
 *
 * 控制器写页 -：
 * ldrb r0, [r4, #24] ; dev[0x18] = 扇区数
 * bl ; 查表：扇区数 → 字节数（4 → 0x1000）
 * ldr r1, [sp, #8]
 * orrs r0, r1
 * str r0, [sp, #8] ; 后面 str 进 NFI[0x000]
 * 然后 再 `orr r0, #1` ⇒ 最终启动值 **0x1001**。
 *
 * 我们一直写 1（burst=0），虽然能跑，但和控制器不是同一个传输模式。
 * 实现同样行为时必须用它。 */
#define F_STARTB 0x400000u
/* ★ 只设 ECC 配置寄存器 NFI[0x00C] = g_ecc_val（不碰启动值）*/
#define F_ECC    0x800000u
/* ★★★ 写 OOB（备用区）。OOB 布局见 docs/03-ecc.md：
 *
 * 控制器写页是**两次 PROGRAM LOAD**：
 * 主区 ：0x32 + 列地址 + 0x2200 （描述符 A 发 512 字节）
 * 备用区：0x34 + 列地址 + 0x4200 （描述符 B 发 512 字节）
 * OOB 数据是**软件准备好交给控制器**的（控制器由 r8 参数传入），
 * 不是控制器自动生成。
 *
 * 带这个标志时：
 * · 描述符 B 指向 OBUF（备用区缓冲区）
 * · NFI[0x210] = -1、NFI[0x214] = 0x80173400（控制器ECC 分支的写法）
 * · cmdset 46 在 EXECUTE 之前插入第二次 LOAD（0x34 + 2048 + 0x4200）
 */
#define F_OOBW   0x1000000u
/* ★ 描述符B 的「长度」写进 NFI[0x208]（而不是 0）。
 * 实测 0x4200 / 0x4240 都只搬 1 个字节 ⇒ 长度不在 FIFO 参数里。 */
#define F_BLEN   0x2000000u
/* ★★★ 写 OOB 专用长度：NFI[0x008] = 0x003F303F
 * bits 16-23 = 0x3F（备用区大小-1）+ 低 16 位 = 0x303F（64 字节）
 * 实测：只有这两个同时给，OOB 才会写满 64 字节。
 * 只给低 16 位 → 只写 1 字节；只给高位 → 主区也坏。
 * 用独立标志，避免污染主区的 NFI[8]。 */
#define F_LENOOB 0x4000000u
/* ★ 单独写 OOB：描述符A → OBUF，描述符B 禁用。配 cmdset 81 用。 */
#define F_OBONLY 0x8000000u

static u32 g_stat_before = 0u;
/* 写页图案序号：每次写用不同图案。
 * 若两次写同一页用同一图案，芯片无需 0→1 翻转就能"编程成功"，
 * 就分不出「真的写了」还是「什么都没发生」。 */
static u32 g_wr_seq = 0u;
static u32 g_poll_status = 0xEEu;   /* SPI NAND 状态轮询最后读到的值 */
/* ★ 写页策略（主机用 0xFB 设）：
 * 0 = 4×512 逐扇区编程，编程后等 ~2ms （旧默认）
 * 1 = 4×512 逐扇区编程，编程后等 ~20ms
 * 2 = 整页一次 LOAD(2048) + 一次 EXECUTE，cfg 扇区数 = 0
 * 3 = 整页一次 LOAD(2048) + 一次 EXECUTE，cfg 扇区数 = 3
 * 同一页被编程 4 次会让片内 ECC 反复重算，2/3 是正解候选。
 *
 * ★★★ 2026-11-03 21:20 实测：mode 2 / 3 **不可用**，默认值已退回 0。
 * bsl_rw.py --sweep 在块 250 写满 64 页：mode 2 在第 4096 字节（第 3 页）
 * 就 MIDST 失败，与「整页 2048 不能一次搬完」的旧结论一致 —— 整页单事务
 * 传输这条路彻底作废。
 * 同一轮扫描还证明：mode 0（4×EXECUTE）写满整块零错，且 B0=9 与 B0=19
 * 都零错 ⇒ **主区数据从来没错过**，变砖只可能出在 OOB / 片内 ECC。
 *
 * 4 = 4 次 PROGRAM LOAD 攒进 cache + 最后 1 次 EXECUTE（见 cmdset 84） */
/* ★ p83（A2）：默认走快路径。厂家工具不发 0xFB，用的就是这里。
 * 默认 0 = 每扇区 2ms 死等 ⇒ 每 16KB 帧 83ms、整盘约 2 分钟；
 * 默认 20 = 页 ≥2 零延时 ⇒ 约 20ms/帧。 */
static u32 g_wr_mode = 20u;
/* ★ 协议日志开关（0xE0 控制）：把收到的命令用 LOG 包回传，
 * 厂家工具会打印成 "BSL_REP_LOG: ..."，出问题能直接看到停在哪条命令。 */
static u32 g_proto_log = 0u;   /* p72：默认关，避免 LOG 与应答交错导致主机错位 */
/* ★★★★★ 2026-11-03 晚：传输长度的高字节 = 备用区（OOB）字节数
 *
 * 控制器写页把 NFI[0x008] 设成 `0x3000 | (页大小-1) | (dev[0x1B] << 24)`，
 * 而 dev[0x1B] = 64 = OOB 大小 ⇒ 实际值 **0x400037FF**。
 * 我们一直传 0x37FF（高字节 0），等于告诉控制器「本次没有备用区」，
 * 于是 ECC 引擎算出的校验值**没有地方落**，擦完的 OOB 就一直是全 FF。
 * 设备启动按 ECC 校验读 UserImg ⇒ 69 块全失败 ⇒ 红灯亮一下即灭。
 *
 * 高字节到底填「整页 OOB 64」还是「每扇区 OOB 16」，靠实测决定：
 * 0xF3 SET_LENHI 可运行时改，扫 0 / 16 / 64 三档，写完读 OOB 对比。 */
static u32 g_wr_lenhi = 0u;
/* ★ 片内配置寄存器 0xB0 的值。
 * ★★★ 2026-11-03 晚：默认值改回 0x09 —— 与控制器FDL2 完全一致。
 * 控制器写页序列第二步就是 SET_FEATURE(0xB0, 0x09)，
 * 读页也是 0x09。也就是说这颗片子在本机固件里
 * **片内 ECC 一直是关的**，纠错由主控/软件层自己做。
 *
 * 之前改 0x19 的理由是「0x09 下 256KB 有 53 字节错，0x19 只剩 22」，
 * 但那批数据是用 **4×EXECUTE** 写的 —— 同一页被编程 4 次造成的
 * program disturb 才是错位来源。改成单次 EXECUTE 后 0x09 应当零错。
 * （0x08 / 0x00 是无效配置，读回全 0xCC，别用。） */
static u32 g_b0_val  = 0x09u;
/* ★ p83（A1）：本次事务实际写的 0xB0。
 * 主机给了 OOB（g_oob_w!=0，我们自己的工具）⇒ 9（ECC 关，OOB 由软件算）；
 * 主机没给 OOB（厂家工具，只发主区）⇒ 0x19（ECC 开，芯片自己生成 ECC），
 * 否则备用区写进去就是一片 FF。 */
static u32 g_b0_apply = 0x09u;
static u32 g_oob_local = 0u;   /* p86：本页 OOB 由固件自己生成 */
/* ★ NFI[0x00C]（ECC 配置）的值，主机用 0xFB 第 4 个参数设。
 * 控制器每个写页事务都会写这个寄存器；我们以前在 F_DMA3 分支里
 * 直接把它清成 0 —— 等于告诉控制器「这次不做 ECC」。 */
static u32 g_ecc_val = 0x00000004u;
/* ★ 写 OOB 开关（0xFB 第 5 个参数）。1 = cmdset 84 同时写备用区。 */
static u32 g_oob_w = 0u;
/* ★ 2026-11-05：OOB 写入字节数。真实 OOB 是 78 字节（2×39），不是 64。
 * NFI[0x008] 编码：bits0-15 = 0x3000|(字节数-1)，bits16-23 = (OOB字节数-1)
 * 64 → 0x003F303F（历史验证值），78 → 0x004D304D。用 0xEE 改。 */
static u32 g_oob_wlen = 64u;
/* ★ 2026-11-06：写页扇区大小。512 = 旧行为（4 次 EXECUTE）；
 * 1024 = 按文档 §十八 的正确扇区（2 次 EXECUTE）。由 cmdset 84 的 mode 5 切换。 */
static u32 g_wr_sector = 512u;
/* ★ 片内配置寄存器 0xA0 的值（0xFB 第 6 参数）。
 * 控制器写页 第一条就是 SET FEATURE(0xA0, 0x38)；
 * 我们 cmdset 84 里一直用 0x00。这个差异可能就是「芯片不自动写 OOB 校验值」的原因。 */
static u32 g_a0_val = 0u;
static u32 g_stat_after  = 0u;

static int nf_one(u32 cfg, u32 psiz, u32 lenhi, u32 row, u32 col,
                  u32 cmdset, u32 startmode, u32 mode)
{

    u32 i;

    if (psiz == 0u)
        psiz = 2048u;

 /* ================= 寄存器配置：始终按 mode 位图 ================= */
    if ((mode & F_OBFF) != 0u) {
 /* ★ 备用区缓冲区填 0xFF：控制器在调用方不传 spare 时
 * 就是 memset(0x80176000, 0xFF, spare*pages)。 */
        for (i = 0u; i < 512u; i++)
 *(volatile unsigned char *)(OBUF + i) = 0xFFu;
    }

    if ((mode & F_OBONLY) != 0u) {
 /* ★★★★★ 单独写 OOB（2026-11-04 修正）
 *
 * 之前的组 D 结论「描述符 B 完全无效」是**错的**，真正的原因是：
 * 组 D 里 F_DESC 把**描述符 A 也设成了 DBUF**，
 * 而 0x34 在「A 和 B 都有效」时会优先用 A ⇒ OOB 装的是主区数据。
 *
 * 证据：`cmdset 75`（2026-11-03 实测成功，OOB 64 字节逐字节一致）
 * 的配置里**根本没设 NFI[0x200]/[0x204]**，只设了 B：
 * NFI[0x208] = 0 ; NFI[0x20C] = 0x80176000
 *
 * ⇒ 正确做法：**描述符 A 必须显式禁用（-1）**，B 指向 OBUF。
 */
 *(volatile u32 *)(NFI_BASE + 0x200u) = 0xFFFFFFFFu; /* A 标志 = -1 */
 *(volatile u32 *)(NFI_BASE + 0x204u) = 0xFFFFFFFFu; /* A 地址 = -1 */
 *(volatile u32 *)(NFI_BASE + 0x208u) = 0u; /* B 标志 = 0 */
 *(volatile u32 *)(NFI_BASE + 0x20Cu) = OBUF; /* B → OBUF */
    } else if ((mode & F_DESC) != 0u) {
        u32 bdst = ((mode & F_DSTOB) != 0u) ? OBUF : DBUF;

 *(volatile u32 *)(NFI_BASE + 0x200u) = 0u;
 *(volatile u32 *)(NFI_BASE + 0x204u) =
            ((mode & F_DSTO) != 0u) ? (DBUF + col) : DBUF;
 *(volatile u32 *)(NFI_BASE + 0x208u) =
            ((mode & F_DSTB) != 0u)
                ? (((mode & F_BLEN) != 0u) ? psiz : 0u)
                : 0xFFFFFFFFu;
 *(volatile u32 *)(NFI_BASE + 0x20Cu) =
            ((mode & F_DSTB) != 0u) ? bdst : 0xFFFFFFFFu;
    }

    FIFO_IDX = 0u;

    if ((mode & F_COL) != 0u) {
        nfc(0x32u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);
        nfc(0xE000u);
    }

    NFI_CFG = cfg;

    if ((mode & F_NOECC) != 0u)
 *(volatile u32 *)(NFI_BASE + 0x0ECu) = 0u; /* 关 ECC */

    if ((mode & (F_DMA3 | F_DST)) != 0u) {
 *(volatile u32 *)(NFI_BASE + 0x00Cu) = 0u;
        if ((mode & F_OOBW) != 0u) {
 /* ★ 写 OOB 时用控制器ECC 分支的配置：
 * NFI[0x210] = -1、NFI[0x214] = 0x80173400（OOB 缓冲区） */
 *(volatile u32 *)(NFI_BASE + 0x210u) = 0xFFFFFFFFu;
 *(volatile u32 *)(NFI_BASE + 0x214u) = 0x80173400u;
        } else {
 /* 普通写页：0 / 1（22:00 已验证主区零错的配置）*/
 *(volatile u32 *)(NFI_BASE + 0x210u) = 0u;
 *(volatile u32 *)(NFI_BASE + 0x214u) =
                ((mode & F_DST) != 0u) ? DBUF : 1u;
        }
    }

 /* ★ 忠实模式：NFI[0x00C] = (planes-1)<<24 | dev[0x1A]<<16 | dev[0x19]
 * planes=2, dev[0x1A]=0, dev[0x19]=4 ⇒ 0x01000004 */
    if ((mode & F_FAITH) != 0u) {
 *(volatile u32 *)(NFI_BASE + 0x00Cu) = 0x01000004u;
 *(volatile u32 *)(NFI_BASE + 0x210u) = 0u;
 *(volatile u32 *)(NFI_BASE + 0x214u) = 1u;
    }

 /* ★★★ F_ECC：只设 ECC 配置寄存器 NFI[0x00C]，**不碰 NFI[0x000]**。
 *
 * 为什么要和 F_FAITH 拆开：F_FAITH 会顺手把启动值改成 0x2801，而那是
 * 本控制器的挂死值。实现同样行为写页时既要 NFI[0x00C] 又不能要 0x2801，
 * 所以单独给一个标志。
 *
 * 控制器：`str r0, [r1, #12]` ⇒ NFI[0x00C] = sl，而
 * sl = dev[0x19] | (dev[0x1A] << 16)，再 | 上 (0xFF000000 + (页数<<24))
 * （页数=1 时该式溢出为 0）⇒ 实际就是 dev[0x19] | dev[0x1A]<<16。
 * 值由 0xFB 的第 4 个参数指定，默认 4。 */
    if ((mode & F_ECC) != 0u) {
 *(volatile u32 *)(NFI_BASE + 0x00Cu) = g_ecc_val;
 *(volatile u32 *)(NFI_BASE + 0x210u) = 0u;
 /* ★ 2026-11-04：A8 参数应指向 **0x80178000**（ECC 状态缓冲区），不是 1。
 * 依据：读页路径会对该地址做 16 字节的 cache 维护，并按扇区解码纠错等级
 * ⇒ 该区域存放 ECC 状态，而非普通数据。 */
 *(volatile u32 *)(NFI_BASE + 0x214u) = 0x80178000u;
    }

 /* ================= 命令序列 ================= */
    switch (cmdset) {
 case 0u: /* 读页（控制器buf!=0 路径）*/
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u);
        nfc(0xE201u); nfc(0xF000u);
        break;

 case 1u: /* 控制器/ 的形态（读 1 字节）*/
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE000u); nfc(0xF000u);
        break;

    case 2u:   /* 芯片复位 */
        nfc(0xFFu); nfc(0xE000u); nfc(0x0Fu); nfc(0xC0u);
        nfc(0x1000u); nfc(0xE001u); nfc(0xF000u);
        break;

    case 3u:   /* ReadID —— ★ 序列必须是 0x9F, 0x00, 0x1000×4
 *
 * 2026-11-04 实测：写成 (0,0x1000)×4 交替会拿到 0xB0B0B0B0，
 * 看起来像芯片坏了。文档 `FDL2-下一步计划-20261031.md` 4.4
 * 早就记过这条：「写成交替就拿到 0xB0B0B0B0」。
 * 正确形态：0x9F → 1 个哑字节 0x00 → 连续 4 次 0x1000 收 4 字节。 */
        nfc(0x9Fu);
        nfc(0x0000u);                                  /* 1 个哑字节 */
        nfc(0x1000u); nfc(0x1000u); nfc(0x1000u); nfc(0x1000u);
        nfc(0xE000u); nfc(0xF000u);
        break;

    case 6u:   /* ★ 用 0x0F（真读操作码）+ 3 字节行地址 */
        nfc(0x0Fu);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0xF000u);
        break;

    case 7u:   /* ★ ONFI READ 0x00 + 5 字节地址 */
        nfc(0x00u);
        nfc(col & 0xFFu);
        nfc((col >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc((row >> 16) & 0xFFu);
        nfc(0xE000u); nfc(0xF000u);
        break;

    case 8u:   /* ★ 0x0F + 带 0x1000 标记的 3 字节地址 */
        nfc(0x0Fu);
        nfc((row >> 16) & 0xFFu); nfc(0x1000u);
        nfc((row >> 8) & 0xFFu);  nfc(0x1000u);
        nfc(row & 0xFFu);         nfc(0x1000u);
        nfc(0xE000u); nfc(0xF000u);
        break;

 case 10u: /* ★ SET FEATURE 0x1F, 0xA0, 0x00（控制器读页前置第 1 条）*/
        nfc(0x1Fu); nfc(0xA0u); nfc(0x00u); nfc(0xE000u); nfc(0xF000u);
        break;

 case 11u: /* ★ SET FEATURE 0x1F, 0xB0, 0x09（控制器读页前置第 2 条）*/
        nfc(0x1Fu); nfc(0xB0u); nfc(0x09u); nfc(0xE000u); nfc(0xF000u);
        break;

 /* ★★★ 2026-11-03 新增：控制器写页/读页的**第一条**前置命令
 * movs r0,#56 (0x38) ; movs r1,#160 (0xA0)
 * ⇒ set_feature(0xA0, 0x38)。我们从来没发过这一条。
 *
 * ★ 编号从 61 改到 67：switch 后面的老诊断分支 `if (cmdset == 61u)`
 * （WREN + PROGRAM LOAD，会 g_wr_seq++ 并用 0x5A5A0000^... 图案
 * **覆盖 DBUF 前 512 字节**）和它撞了同一个编号。结果是每次「设
 * A0=0x38」都会把我们刚写进 DBUF 的页数据冲掉，写进 flash 的是
 * 图案而不是数据 —— 2026-11-03 两轮实验全部失败都是这个原因。
 */
    case 67u:
        nfc(0x1Fu); nfc(0xA0u); nfc(0x38u); nfc(0xE000u); nfc(0xF000u);
        break;

 /* ★ case 76 = SET FEATURE(0xA0, g_a0_val)，值可由主机设 */
    case 76u:
        nfc(0x1Fu); nfc(0xA0u); nfc(g_a0_val & 0xFFu);
        nfc(0xE000u); nfc(0xF000u);
        break;

    case 12u:  /* 读 feature 0xB0：0x0F,0xB0,0x1000 */
        nfc(0x0Fu); nfc(0xB0u); nfc(0x1000u); nfc(0xE000u); nfc(0xF000u);
        break;

    case 18u:  /* ★ SET FEATURE 0xB0 = g_b0_val（写/读都用它，可扫）
 * 9 = 控制器值；8 = 清掉 bit0；0 = 全关；0x19/0x10 = 试探 bit4 */
        nfc(0x1Fu); nfc(0xB0u); nfc(g_b0_apply & 0xFFu);
        nfc(0xE000u); nfc(0xF000u);
        break;

    case 13u:  /* ★ 0x10 + 带 0x1000 标记的地址 + 状态读 + DMA */
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu); nfc(0x1000u);
        nfc((row >> 8) & 0xFFu);  nfc(0x1000u);
        nfc(row & 0xFFu);         nfc(0x1000u);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u);
        nfc(0xE201u); nfc(0xF000u);
        break;

    case 14u:  /* ★ 0x10 + 标记地址，去掉状态读 */
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu); nfc(0x1000u);
        nfc((row >> 8) & 0xFFu);  nfc(0x1000u);
        nfc(row & 0xFFu);         nfc(0x1000u);
        nfc(0xE000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 15u:  /* ★ 最简：0x10 + 3 地址 + 0xE201 */
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 16u:  /* ★ 2 字节行地址（小页假设）*/
        nfc(0x10u);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u);
        nfc(0xE201u); nfc(0xF000u);
        break;

 /* ★★★★★ case 40/41/42 = 擦除专用（按控制器的工作方式实现，逐条核对）
 * 控制器：SET_FEATURE(0xA0,0) → SET_FEATURE(0xB0,0) ← 注意是 0，不是 9
 * 0x06, 0xE000, 0xD8, addr>>16, addr>>8, addr, 0xE000, 0xF000
 * NFI[4]=0x8102 NFI[8]=0x3000 NFI[0xF8]=0x3A NFI[0]=1
 * 然后另起一个事务读状态：0x0F,0xC0,0x1000,0xE001,0xF000
 */
    case 40u:  /* SET FEATURE 0x1F, 0xB0, 0x00（擦除前置第 2 条）*/
        nfc(0x1Fu); nfc(0xB0u); nfc(0x00u); nfc(0xE000u); nfc(0xF000u);
        break;

    case 41u:  /* 擦除主体：WREN 独立事务 + BLOCK ERASE + 3 字节块地址 */
        nfc(0x06u); nfc(0xE000u);
        nfc(0xD8u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0xF000u);
        break;

 case 42u: /* ★ 读状态 0xC0（按控制器的工作方式实现：0xE000，不是 0xE001）*/
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE000u); nfc(0xF000u);
        break;

    case 82u:  /* ★★★★★ 写 OOB 78 字节（每扇区 39，整页 2 段）
 *
 * 和 cmdset 75（64 字节）的区别只有两处：
 * NFI[0x008] = 0x004D304D （bits16-23=77, 低12位=0x04D=77）
 * FIFO = 0x424E （78 字节）
 *
 * 为什么要 78 而不是 64：
 * 实测 OOB 结构是「每 39 字节一段 × 2 段 = 78 字节」。
 * 只写 64 会缺第 2 段的最后 14 字节 ⇒ 第 2 个扇区 ECC 不完整。
 *
 * 寄存器由 mode 配：
 * F_DESC|F_DSTB|F_DSTOB 描述符B → OBUF
 * F_DMA3|F_OOBW NFI[0x00C]=0, 0x210=-1, 0x214=0x80173400
 * F_LENRAW + lenhi=0x004D304D
 * F_F8, F_NOMARK
 */
        nfc(0x06u); nfc(0xE000u);                     /* WREN */
        nfc(0x34u);                                    /* QUAD PROGRAM LOAD（备用区）*/
        nfc(0x08u); nfc(0x00u);                        /* 列地址 2048 */
 /* ★ 2026-11-04 21:05 修正：FIFO 保持 0x4240（和 cmdset 75 一样）。
 * 文档 FDL2-擦除与写入-20261102.md 12.3 明确写过：
 * 「0x4200 / 0x4240 改长度无效（长度不由 FIFO 的 N 决定）」
 * 上一版改成 0x424E 后 OOB 完全没写进去（读回全 FF）。
 * 长度只由 NFI[0x008] 决定，这里改回 0x4240。 */
        nfc(0x4240u);                                  /* 描述符B（长度由 NFI[8] 决定）*/
        nfc(0xE000u);
        nfc(0x10u);                                    /* PROGRAM EXECUTE */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 81u:  /* ★★★★★ 单独写 OOB（前人的方案，2026-11-03 设计，2026-11-04 首次实装）
 *
 * 为什么不用「同一事务两次 LOAD」：
 * 组 D 实测（OBUF 填 0xAA 但 OOB 读回主区数据）证明
 * **描述符 B 完全无效** —— 0x34 只会用描述符 A 的数据。
 * 所以改回前人 2026-11-03 在 FDL2-下一步-20261103.md 里
 * 设计的方案：**用 0x32 + 描述符 A 写 OOB**。
 *
 * 依据（前人原文）：
 * 「编程 0xFF 不改变已有位 ⇒ 主区写完再单独编程 OOB
 * 不会破坏主区。」
 *
 * 参数：
 * row = 页地址
 * col = 2048（OOB 起点，函数内再 |0x8000 区标志）
 * psiz = OOB 字节数（78）
 * NFI[0x204] = OBUF（描述符 A 指向 OOB 缓冲）
 * NFI[0x208] = NFI[0x20C] = -1（描述符 B 禁用）
 * NFI[0x008] = 0x3000 | (psiz-1)（必须精确等于描述符字节数）
 *
 * ⚠️ 前人标的三个坑：
 * ① NFI[8] 必须精确等于描述符字节数，否则永久挂 START=0x40000000
 * ② NFI[0] 必须写 1，不能写 burst 0x2800
 * ③ 列地址必须带区标志（备用区 |0x8000）
 */
        cache_clean(OBUF, psiz ? psiz : 128u);
        nfc(0x06u); nfc(0xE000u);                     /* WREN */
        nfc(0x32u);                                   /* PROGRAM LOAD（不是 0x34）*/
        nfc(((col | 0x8000u) >> 8) & 0xFFu);          /* ★ 区标志 0x8000 */
        nfc((col | 0x8000u) & 0xFFu);
        nfc(0x2000u | (psiz ? psiz : 64u));           /* ★ 描述符A 发 psiz 字节 */
        nfc(0xE000u);
        nfc(0x10u);                                   /* PROGRAM EXECUTE */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

 case 80u: /* ★★★★★ 2026-11-04 控制器完整写页（主区 1024×2 + OOB 39×2，同一事务）
 *
 * 完整写页流程：
 * · 一次事务，两次 PROGRAM LOAD（0x32 主区 / 0x34 备用区）
 * · 描述符 A → 主区缓冲，描述符 B → OOB 缓冲
 * · 描述符 C → 0x80178000（ECC 状态）
 * · OOB 缓冲全 0xFF，ECC 校验值由控制器硬件在 DMA 时插入
 *
 * 与 cmdset 46 的区别：
 * · 开头先 cache clean（46 依赖调用方已 clean）
 * · OOB LOAD 无条件发（46 要 F_OOBW 才发）
 * · OOB 列地址固定 2048（备用区起点）
 *
 * 寄存器由 mode 配：
 * cfg = 0x01008178 (2 扇区)
 * lenhi= 0x002633FF (1024B 数据 + 39B OOB/扇区) 用 F_LENRAW
 * F_ECC + g_ecc_val = 0x01000004
 * F_DESC|F_DSTB|F_DSTOB|F_OBFF|F_NOMARK|F_STARTB
 */
        cache_clean(DBUF, 2048u);
        cache_clean(OBUF, 128u);
        nfc(0x06u); nfc(0xE000u);                     /* WRITE ENABLE */
        nfc(0x32u);                                   /* 主区 PROGRAM LOAD */
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);                                 /* 描述符A 发 512 字节 */
        nfc(0xE000u);
        nfc(0x34u);                                   /* ★ 备用区 PROGRAM LOAD */
        nfc(0x08u); nfc(0x00u);                       /* 列地址 2048 */
        nfc(0x4200u);                                 /* 描述符B 发 512 字节 */
        nfc(0xE000u);
        nfc(0x10u);                                   /* PROGRAM EXECUTE */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

 case 46u: /* ★★★★★ 写页主体（控制器的单扇区简化版）
 * WREN → PROGRAM LOAD(0x32 + col + 0x2200，描述符A 发 512B)
 * → PROGRAM EXECUTE(0x10 + row×3)
 * → 读状态(0x0F,0xC0,0x1000,0xE201)
 * 方向是「主机→flash」，控制器要读主机内存 ⇒ 只需 cache clean */
        nfc(0x06u); nfc(0xE000u);                     /* WRITE ENABLE */
        nfc(0x32u);                                   /* QUAD PROGRAM LOAD */
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2000u | g_wr_sector);                   /* 描述符A 发 g_wr_sector 字节 */
        nfc(0xE000u);
 /* ★★★ F_OOBW：第二次 PROGRAM LOAD —— 备用区（OOB）
 * 控制器：0x34 + 列地址 + 0x4200，描述符 B 发 512 字节。
 * 列地址 = 页大小 2048（备用区在页内的起始列）。
 * 必须在同一个 NFI 事务里，EXECUTE 之前。 */
        if ((mode & F_OOBW) != 0u) {
            nfc(0x34u);                               /* QUAD PROGRAM LOAD（备用区）*/
            nfc(0x08u);                               /* 2048 >> 8 */
            nfc(0x00u);                               /* 2048 & 0xFF */
 /* ★ 长度用 0x40（64 字节 = OOB 大小），不是 0x200。
 * 实测 0x4200 只写进去 1 个字节 —— 备用区只有 64 字节空间，
 * 按 512 字节搬会被截断。 */
 /* ★ 长度参数化：78 字节 → 0x424E（原来写死 0x4240=64B）*/
            nfc(0x4200u | (g_oob_wlen & 0xFFu));
            nfc(0xE000u);
        }
        nfc(0x10u);                                   /* PROGRAM EXECUTE */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 71u:  /* ★ 整页读 cache：0x3800 = 描述符A 收 0x800 = 2048 字节 */
        nfc(0x6Bu);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x6000u);
        nfc(0x3800u);
        nfc(0xE000u);
        nfc(0xF000u);
        break;

    case 72u:  /* ★ 整页写：0x2800 = 描述符A 发 0x800 = 2048 字节 */
        nfc(0x06u); nfc(0xE000u);
        nfc(0x32u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2800u);
        nfc(0xE000u);
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 70u:  /* ★★★★★ 干净的读页第二步（不用 F_FAITH 的 burst，不读 OOB）
 * 0x6B + 列地址 + 0x6000(哑) + 0x3200(描述符A 收 512B) */
        nfc(0x6Bu);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x6000u);
        nfc(0x3200u);
        nfc(0xE000u);
        nfc(0xF000u);
        break;

    case 73u:  /* ★★★★★ 读 cache 的**备用区（OOB）**64 字节
 * 与 case 70 同形，三处不同：
 * ① 列地址必须带区标志 |0x8000（主区是 |0x4000）
 * ② FIFO 用 0x5040 = **描述符B 收 0x40 = 64 字节**
 * （不是描述符A 的 0x3000+N，备用区走 B 通道）
 * ③ NFI[0x008] 必须 = 0x303F（= 0x3000|(64-1)），由 F_LEN 算
 * 前提：调用方已设 F_DSTB（描述符B 指向 DBUF），
 * 且 NFI[0x200]=0 / NFI[0x204]=-1 已由 F_DESC 处理。
 */
        nfc(0x6Bu);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x6000u);
        nfc(0x5040u);
        nfc(0xE000u);
        nfc(0xF000u);
        break;

    case 47u:  /* ★ 只做 WREN + PROGRAM LOAD A（不发 PROGRAM EXECUTE）
 * 用来隔离「描述符 A 发数据」这一步会不会卡 */
        nfc(0x06u); nfc(0xE000u);
        nfc(0x32u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);
        nfc(0xE000u);
        nfc(0xF000u);
        break;

    case 48u:  /* ★ 只做 WREN + PROGRAM EXECUTE（不做 PROGRAM LOAD）
 * 用来隔离「0x10 执行」这一步会不会卡 */
        nfc(0x06u); nfc(0xE000u);
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0xF000u);
        break;

 case 99u: /* ★★★★★ 忠实实现同样行为 的单页事务
 *
 * 控制器顺序（分析确认）：
 * 0x13 + row + E000 → 0F C0 1000 → E001 （PAGE READ + 读状态）
 * 06 + E000 （WREN）
 * 32 + col + 2200 + E000 （主区 LOAD，512B）
 * [34 + spare_col + 4200 + E000] （备用区 LOAD）
 * 10 + row + E000 → 0F C0 1000 E201 F000 （EXECUTE + 读状态）
 *
 * 寄存器由 nf_one 按 mode 配：F_FAITH（NFI[0]=0x2801、NFI[0x0C]=0x01000004）、
 * F_LENRAW（NFI[8] 手工）、F_F8、F_NOMARK。
 */
        nfc(0x13u);                                   /* PAGE READ */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE001u);
        nfc(0x06u); nfc(0xE000u);                     /* WREN，在 PAGE READ 之后 */
        nfc(0x32u);                                   /* 主区 LOAD */
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);                                 /* 恒 512 字节 */
        nfc(0xE000u);
        nfc(0x34u);                                   /* 备用区 LOAD（列固定 2048）*/
        nfc(0x08u); nfc(0x00u);
        nfc(0x4200u | (g_oob_wlen & 0xFFu));
        nfc(0xE000u);
        nfc(0x10u);                                   /* PROGRAM EXECUTE */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 96u:  /* ★ 判决实验 A：只写主区，不碰备用区
 *
 * 目的：把「备用区 LOAD」从整页单事务里摘出去，
 * 单独验证「cfg 高字节=3 + 描述符 512 ⇒ 控制器连拍 4 次」
 * 这个假设到底成不成立。
 *
 * 参数（主机给）：cfg = 0x03810078、NFI[0x008] = 0x0037FF、
 * mode = F_DESC|F_DMA3|F_LENRAW|F_F8|F_NOMARK（描述符B 禁用）
 */
 /* ★★★ 必须有：页面数据还在 CPU 的写缓冲/cache 里，
 * nf_one 末尾的 cache_inv2 是**作废**，会把脏行直接丢掉 ⇒
 * 实测症状：读回全 FF。cmdset 84 就是在开头 clean 才正常的。*/
        cache_clean(DBUF, 2048u);
        cache_clean(OBUF, 128u);
        nfc(0x06u); nfc(0xE000u);                     /* WRITE ENABLE */
        nfc(0x32u);                                   /* QUAD PROGRAM LOAD */
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);                                 /* 512B × cfg 拍数 */
        nfc(0xE000u);
        nfc(0x10u);                                   /* PROGRAM EXECUTE（一次）*/
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 98u:  /* ★ 判决实验 C：把「拍数-1」放进描述符 A 的 bit10-12
 *
 * 0x2200 = bit13(发送) | bit9 | 长度 511
 * 0x2E00 = 0x2200 | (3<<10) ⇒ 试探「4 拍」
 */
        nfc(0x06u); nfc(0xE000u);
        nfc(0x32u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2E00u);                                 /* ★ 试探 4 拍 */
        nfc(0xE000u);
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 97u:  /* ★ 判决实验 B：和 96 相同，但**前面带 PAGE READ**
 *
 * 控制器在同一个事务里先发 0x13（PAGE READ）+ 读状态，
 * 然后才 WREN / LOAD / EXECUTE。
 * 如果 96 超时、97 正常 ⇒ 说明「先灌 cache」是必需的。
 */
 /* ★★★ 必须有：页面数据还在 CPU 的写缓冲/cache 里，
 * nf_one 末尾的 cache_inv2 是**作废**，会把脏行直接丢掉 ⇒
 * 实测症状：读回全 FF。cmdset 84 就是在开头 clean 才正常的。*/
        cache_clean(DBUF, 2048u);
        cache_clean(OBUF, 128u);
        nfc(0x13u);                                   /* PAGE READ */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE001u);
        nfc(0x06u); nfc(0xE000u);                     /* WRITE ENABLE */
        nfc(0x32u);                                   /* QUAD PROGRAM LOAD */
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);
        nfc(0xE000u);
        nfc(0x10u);                                   /* PROGRAM EXECUTE（一次）*/
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 95u:  /* ★★★★★ 整页单事务：WREN → 0x32(512) → 0x34(备用区) → 0x10 一次
 *
 * 依据控制器的完整时序（2026-11-07 逐条复核）：
 *
 * · cfg 的**高字节** = 扇区数-1
 * （控制器：cfg = 0xFF000000 + dev[0x10]<<24 | flags，
 * 4 扇区时算出 0x03xxxxxx ⇒ 0x03810078）
 * · 描述符 A 的长度**始终是 512**（0x2200）
 * 整页 2048 是**控制器自己按 cfg 连拍 4 次**，
 * 不是把 2048 塞进描述符 —— 那会溢出 9 位长度字段
 * （mode 2 用 0x2800、mode 5 用 0x2400 都是这样废掉的）
 * · NFI[0x008] 低 12 位 = **整页字节数-1**，不是单拍字节数
 * （控制器：0x3000 | (页大小-1) | (备用区-1)<<16）
 * · 命令流只有**一次** 0x10
 *
 * ⇒ 一页只编程 1 次，和控制器完全一致。
 */
 /* ★★★ 必须有：页面数据还在 CPU 的写缓冲/cache 里，
 * nf_one 末尾的 cache_inv2 是**作废**，会把脏行直接丢掉 ⇒
 * 实测症状：读回全 FF。cmdset 84 就是在开头 clean 才正常的。*/
        cache_clean(DBUF, 2048u);
        cache_clean(OBUF, 128u);
        nfc(0x06u); nfc(0xE000u);                     /* WRITE ENABLE */
        nfc(0x32u);                                   /* QUAD PROGRAM LOAD */
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);                                 /* 描述符A：512B × cfg 拍数 */
        nfc(0xE000u);
        nfc(0x34u);                                   /* 备用区 PROGRAM LOAD */
        nfc(0x08u); nfc(0x00u);                       /* 列 2048 */
 /* ★ 与实测可用的 cmdset 75 保持一致：0x4200 | 字节数（78 → 0x424E）。
 * 控制器写 0x4200，我们自己试出来 0x4200 只写进 1 字节。 */
        nfc(0x4200u | (g_oob_wlen & 0xFFu));          /* 描述符B：OOB 字节数 */
        nfc(0xE000u);
        nfc(0x10u);                                   /* PROGRAM EXECUTE（仅一次）*/
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

 case 90u: /* ★★★★★ 单事务 4×LOAD(512) + 1×EXECUTE —— 按控制器的工作方式实现
 *
 * 为什么必须单事务：`mode 4` 用 4 个独立事务攒扇区，
 * 实测**只有最后一个扇区写进去**（前 3 个全 FF）⇒
 * 芯片 cache 在 NFI 事务之间会被清（case 49 的旧注释是错的）。
 *
 * 控制器在同一个事务里，每次 LOAD 前**直接改 NFI[0x204]**
 * （描述符A 地址 = 缓冲 + 扇区偏移），见 / ，
 * 所以 4 个扇区能攒进 cache，最后只 EXECUTE 一次。
 *
 * ⇒ 一页只编程 1 次，和控制器一致。
 * ⚠️ 前提：NFI 的 FIFO 是**流式执行**（写完即执行），
 * 否则 CPU 改 NFI[0x204] 时 FIFO 还没跑，4 次 LOAD 会用同一个地址。
 * 这一点必须靠实测确认（读回 4 个扇区都要零错）。
 */
    {
        u32 s90;
        nfc(0x06u); nfc(0xE000u);                  /* WRITE ENABLE，只需一次 */
        for (s90 = 0u; s90 < 4u; s90++) {
            u32 c90 = col + s90 * 512u;
 *(volatile u32 *)(NFI_BASE + 0x200u) = 0u;
 *(volatile u32 *)(NFI_BASE + 0x204u) = DBUF + s90 * 512u;
            nfc(0x32u);                            /* QUAD PROGRAM LOAD */
            nfc((c90 >> 8) & 0xFFu);
            nfc(c90 & 0xFFu);
            nfc(0x2200u);                          /* 描述符A 发 512 字节 */
            nfc(0xE000u);
        }
        nfc(0x10u);                                /* PROGRAM EXECUTE，只一次 */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;
    }

    case 49u:  /* ★★★★★ 只做 PROGRAM LOAD（不发 WREN、不发 EXECUTE）
 * 整页写的扇区循环用：WREN 只发一次，0x32 发 4 次，
 * 最后一次 0x10 才真正编程。芯片的 cache 寄存器在
 * 多个 NFI 事务之间不会丢，所以可以这样拆。
 * col = 本扇区列地址（0/512/1024/1536），描述符 A 发 512B */
        nfc(0x32u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);
        nfc(0xE000u); nfc(0xF000u);
        break;

 /* ============ ★★★ 备用区（OOB）PROGRAM LOAD —— 2026-11-03 新增 ============
 *
 * 控制器写页 / 的第二次 LOAD（-）：
 * cmd 0x34, (spare_col>>8), (spare_col&0xFF), 0x4200, 0xE000
 * 其中 0x4200 = 描述符 B 发 0x200 字节，spare_col = 页大小 + 备用区*页内序号
 * （dev[0x17]!=0 分支：`ldrh r0,[r4,#12]` + `ldrh r1,[r4,#38]`×r6）。
 * 本芯片页 2048 / 备用区 64，页内序号 0 ⇒ spare_col = 2048 = 0x0800。
 *
 * 配合 NFI[0x008] 的 bits 16-23 = 备用区字节数-1（= 0x3F），
 * 控制器才知道这次编程要连备用区一起写。
 */
    case 57u:  /* 只做备用区 PROGRAM LOAD（不 WREN、不 EXECUTE）*/
        nfc(0x34u);
        nfc(0x08u);                    /* 2048 >> 8 */
        nfc(0x00u);                    /* 2048 & 0xFF */
        nfc(0x4200u);                  /* 描述符B 发 0x200 字节 */
        nfc(0xE000u); nfc(0xF000u);
        break;

 /* ★★★★★ case 75 = **只写备用区（OOB）64 字节**，独立事务
 *
 * 为什么要独立事务：主区 PROGRAM LOAD 要 512 字节，而备用区只有 64 字节，
 * 两者共用同一个 NFI[0x008]（本次传输字节数）。放同一个事务里必然冲突 ——
 * 实测「主区 512 + 备用区 64」时备用区只写进去 1 个字节。
 *
 * 依据 `FDL2-擦除与写入-20261102.md` 9.7：
 * 「编程 0xFF 不改变已有位，所以主区写完再单独编程 OOB 不会破坏主区。」
 *
 * NFI[0x008] 由 F_LEN + psiz=64 算成 0x303F（精确等于描述符字节数）。
 * 描述符 B 指向 OBUF（F_DSTB|F_DSTOB）。
 */
    case 75u:
        nfc(0x06u); nfc(0xE000u);                     /* WREN */
        nfc(0x34u);                                    /* QUAD PROGRAM LOAD（备用区）*/
        nfc(0x08u); nfc(0x00u);                        /* 列地址 2048 */
 /* ★ 2026-11-05：长度改成参数（默认 64，可用 0xEE 设为 78） */
        nfc(0x4200u | (g_oob_wlen & 0xFFu));
        nfc(0xE000u);
        nfc(0x10u);                                    /* PROGRAM EXECUTE */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

 /* ★★★★★ case 68 = 写页用**标准单线** PROGRAM LOAD（0x02）
 *
 * 逻辑依据（2026-11-03）：
 * · 擦除用 0xD8 能成功改 flash ⇒ WREN、写保护都没问题
 * · 写页事务 ret=0（NFI 传输完成），但 flash 一个字节都没变
 * · ⇒ 问题只可能在 0x32（QUAD PROGRAM LOAD）或 0x10（EXECUTE）
 *
 * 0x32 是**四线**加载，要求芯片配置寄存器真正打开 QE 位。
 * 读页用的 0x6B 在 QE 没开时也可能勉强出数据，所以「读页正常」
 * 不能证明 QE 已开。标准 SPI NAND 的单线写 cache 命令是 0x02。
 */
    case 68u:
        nfc(0x06u); nfc(0xE000u);                     /* WREN */
        nfc(0x02u);                                    /* PROGRAM LOAD（单线）*/
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);                                  /* 描述符A 发 512 字节 */
        nfc(0xE000u);
        nfc(0x10u);                                    /* PROGRAM EXECUTE */
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

 /* ★ case 69 = 只做 WREN + 0x02 LOAD，不发 EXECUTE（隔离用）*/
    case 69u:
        nfc(0x06u); nfc(0xE000u);
        nfc(0x02u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);
        nfc(0xE000u); nfc(0xF000u);
        break;

 case 58u: /* ★ 整页写（主区 2048 + 备用区）一次事务，最接近控制器形态 */
        nfc(0x06u); nfc(0xE000u);                     /* WRITE ENABLE */
        nfc(0x32u); nfc(0x00u); nfc(0x00u); nfc(0x2800u); nfc(0xE000u);
        nfc(0x34u); nfc(0x08u); nfc(0x00u); nfc(0x4200u); nfc(0xE000u);
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

 /* ★★★★★ case 64 = **逐条实现同样行为写页**，一次事务做完全部步骤
 *
 * 实证的命令顺序（含每条指令地址）：
 * 0x13 row_hi row_mid row_lo ← PAGE READ（我们从来没做过）
 * 0xE000 0x0F 0xC0 0x1000 0xE001 ← 读状态，等 OIP 落
 * 0x06 ← WREN
 * 0xE000
 * 0x32 col_hi col_lo 0x2200 ← PROGRAM LOAD 主区（描述符A）
 * 0xE000
 * 0x34 spr_hi spr_lo 0x4200 ← PROGRAM LOAD 备用区（描述符B）
 * 0xE000
 * 0x10 row_hi row_mid row_lo ← PROGRAM EXECUTE
 * 0xE000 0x0F 0xC0 0x1000 0xE201 0xF000
 *
 * 配合的寄存器（控制器-）：
 * NFI[0x000] = 0x1001 (F_STARTB)
 * NFI[0x008] = 0x003F37FF (F_LENRAW，bits16-23 = OOB-1 = 63)
 * NFI[0x00C] = 0x01000004 (F_FAITH)
 * NFI[0x0F8] = 0x3A (F_F8)
 * NFI[0x004] = 0x8178 (cfg 参数)
 *
 * ★ 关键差别：**所有命令在同一个 NFI 事务里**。之前我们把 WREN / 4 次 LOAD /
 * 备用区 LOAD / EXECUTE 拆成 7 个独立事务，而芯片的 cache 在事务之间会被
 * 清掉，备用区数据根本留不住 —— 这就是 V7/V8 明明「补了备用区 LOAD」
 * 却仍然写出全 FF 的原因。
 */
    case 64u:
        nfc(0x13u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE001u);
        nfc(0x06u);
        nfc(0xE000u);
        nfc(0x32u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);
        nfc(0xE000u);
        nfc(0x34u);
        nfc(0x08u);                    /* 备用区列地址 2048 = 页大小 */
        nfc(0x00u);
        nfc(0x4200u);
        nfc(0xE000u);
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

 /* ★ case 65 = 同 64，但把备用区 LOAD 去掉（对照组）。
 * 用来证明「OOB 有内容」到底来自 ECC 引擎自动附加，还是来自那次 0x34。
 */
    case 65u:
        nfc(0x13u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE001u);
        nfc(0x06u);
        nfc(0xE000u);
        nfc(0x32u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x2200u);
        nfc(0xE000u);
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

 /* ★ case 66 = 同 64，但主区 LOAD 用 0x2800（整页 2048 一次）而不是 0x2200 */
    case 66u:
        nfc(0x13u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE001u);
        nfc(0x06u);
        nfc(0xE000u);
        nfc(0x32u); nfc(0x00u); nfc(0x00u); nfc(0x2800u);
        nfc(0xE000u);
        nfc(0x34u); nfc(0x08u); nfc(0x00u); nfc(0x4200u);
        nfc(0xE000u);
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 59u:  /* 只做主区 2048 LOAD（拆开做时用）*/
        nfc(0x32u); nfc(0x00u); nfc(0x00u); nfc(0x2800u); nfc(0xE000u);
        nfc(0xF000u);
        break;

    case 60u:  /* 备用区 LOAD + PROGRAM EXECUTE（配合 case 49/59 分步做）*/
        nfc(0x06u); nfc(0xE000u);
        nfc(0x34u); nfc(0x08u); nfc(0x00u); nfc(0x4200u); nfc(0xE000u);
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE201u); nfc(0xF000u);
        break;

    case 45u:  /* ★ WREN 单独一发：0x06, 0xE000, 0xF000
 * 用来证明「状态寄存器读得到」：WREN 之后 WEL(bit1) 必须为 1 */
        nfc(0x06u); nfc(0xE000u); nfc(0xF000u);
        break;

    case 17u:  /* ★ 擦除：0x06, 0xE000, 0xD8, addr_hi, addr_mid */
        nfc(0x06u); nfc(0xE000u); nfc(0xD8u);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0xF000u);
        break;

 case 19u: /* ★★ 控制器第一步：0x1F0, 0xFF */
        nfc(0x1F0u); nfc(0xFFu); nfc(0xE000u); nfc(0xF000u);
        break;

 case 20u: /* ★★★ 忠实 ReadID（控制器）
 * 0x9F, 0x00, 0x1000 ×4, 0xE000, 0xF000
 * ★ 注意：不是 (0,0x1000)×4，而是 1 个 0x00 + 连续 4 个 0x1000 */
        nfc(0x9Fu);
        nfc(0x00u);
        nfc(0x1000u);
        nfc(0x1000u);
        nfc(0x1000u);
        nfc(0x1000u);
        nfc(0xE000u); nfc(0xF000u);
        break;

 /* ============ ★★★ SPI NAND 标准命令集 ============
 * 本项目控制器（GD5F1GQ5 / W25N01GW）都是 SPI NAND，
 * 页结构 2048+OOB / 64 页每块 / 128KB 块 —— 与 spec[64] 完全一致。
 *
 * 0x9F READ ID 0xFF RESET
 * 0x0F GET FEATURE 0x1F SET FEATURE
 * 0x13 PAGE READ (3 字节行地址)
 * 0x0B READ FROM CACHE (2 字节列地址 + 1 dummy)
 * 0x10 PROGRAM EXECUTE ← 我们之前误当"读页"用的就是它
 * 0x06 WRITE ENABLE 0xD8 BLOCK ERASE
 */

    case 21u:  /* SPI NAND 第一步：PAGE READ 0x13 + 3 字节行地址 */
        nfc(0x13u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u); nfc(0xF000u);
        break;

 case 22u: /* PAGE READ + 读状态轮询（控制器写路径的前缀就是这个）*/
        nfc(0x13u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u);
        nfc(0xE000u); nfc(0xF000u);
        break;

    case 23u:  /* SPI NAND 第二步：READ FROM CACHE 0x0B + 列地址 + dummy */
        nfc(0x0Bu);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x1000u);                    /* 1 个 dummy 字节 */
        nfc(0xE201u); nfc(0xF000u);      /* 带 DMA 取数据 */
        break;

    case 24u:  /* READ FROM CACHE，不带 DMA（数据留在 FIFO，从 0x19C 读）*/
        nfc(0x0Bu);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x1000u);
        nfc(0xE000u); nfc(0xF000u);
        break;

    case 26u:  /* 0x03 = READ（非 fast），2 字节列 + dummy */
        nfc(0x03u);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x1000u);
        nfc(0xE201u); nfc(0xF000u);
        break;

 /* ★★★★★ case 32 = 忠实读页（与控制器逐条对应）
 * ① PAGE READ 0x13 + 3 字节行地址 → 0xE000
 * ② GET FEATURE 0x0F,0xC0,0x1000 → 0xE001（轮询状态）
 * ③ 描述符 A = DBUF，描述符 B 禁用
 * ④ 0x6B（Quad Output Fast Read）+ 列地址|0x4000 + 0x6000(哑字节) + 0x3200
 * → 0xE000 → 0xF000
 * ★ 全镜像从没用过 0x0B，读 cache 只有 0x6B。
 * ★ 0x3200 = 描述符 A 收 0x200(512) 字节。
 */
    case 32u:
        nfc(0x13u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE001u);
 *(volatile u32 *)(NFI_BASE + 0x200u) = 0u;
 *(volatile u32 *)(NFI_BASE + 0x204u) = DBUF;
 *(volatile u32 *)(NFI_BASE + 0x208u) = 0xFFFFFFFFu;
 *(volatile u32 *)(NFI_BASE + 0x20Cu) = 0xFFFFFFFFu;
        nfc(0x6Bu);
        nfc(((col | 0x4000u) >> 8) & 0xFFu);
        nfc((col | 0x4000u) & 0xFFu);
        nfc(0x6000u);
        nfc(0x3200u);
        nfc(0xE000u);
        nfc(0xF000u);
        break;

    default:   /* 读页 + 尾部换成 0x0F 状态读 */
        nfc(0x10u);
        nfc((row >> 16) & 0xFFu);
        nfc((row >> 8) & 0xFFu);
        nfc(row & 0xFFu);
        nfc(0xE000u);
        nfc(0x0Fu); nfc(0xC0u); nfc(0x1000u); nfc(0xE000u);
        nfc(0xE201u); nfc(0xF000u);
        break;
    }

 /* ================= 收尾 ================= */
    NFI_STAT |= 0xF00u;
    g_stat_before = NFI_STAT;

    if ((mode & F_START0) != 0u)
        NFI_START = 0u;

    NFI_LEN = 0x3000u;
    if ((mode & F_LEN) != 0u)
        NFI_LEN |= (psiz - 1u) | (lenhi << 24);
    if ((mode & F_LENRAW) != 0u)
        NFI_LEN = lenhi;                             /* ★ 完全手工指定 */
    if ((mode & F_LENOOB) != 0u) {
        u32 _n = (g_oob_wlen - 1u) & 0xFFu;
        NFI_LEN = 0x3000u | _n | (_n << 16);         /* 64→0x003F303F，78→0x004D304D */
    }
    if ((mode & F_LEN2) != 0u)
        NFI_LEN = (psiz - 1u) | (lenhi << 24);   /* 不带 0x3000 基 */

    if ((mode & F_F8) != 0u)
 *(volatile u32 *)(NFI_BASE + 0x0F8u) = 0x3Au;
    else if ((mode & F_F8C) != 0u)
 *(volatile u32 *)(NFI_BASE + 0x0F8u) = 0x403Au; /* ★ bit14 */
    else if ((mode & F_F82) != 0u)
 *(volatile u32 *)(NFI_BASE + 0x0F8u) |= 2u; /* ★ 读-改-写 */
    else if ((mode & F_F8B) != 0u)
 *(volatile u32 *)(NFI_BASE + 0x0F8u) = 0x02u;
    else if ((mode & F_F87A) != 0u)
 *(volatile u32 *)(NFI_BASE + 0x0F8u) = 0x7Au; /* ★ 读方向 0x08|0x30|0x40|2 */

    if ((mode & F_START0) != 0u)
        NFI_START = 0u;

 /* ★ 忠实模式：NFI[0x000] = burst = 0x2800（= nf_map(dev[0x18]=0x10)） */
    if ((mode & F_FAITH) != 0u)
        NFI_START = 0x2800u;

 /* ★★★ 2026-11-03 关键修复（第 2 版，这才是真凶）
 *
 * 主机用 0xFF WRITE_MEM 把页数据写进 DBUF 时，数据还在 CPU 的 data
 * cache 里（脏行）。要写 flash，必须先把它 clean 回物理内存。
 *
 * 原来的代码在这里无条件 `cache_inv2(DBUF, psiz)` —— 而擦除（cmdset
 * 40/41）、SET FEATURE（18/67）这些**不碰 DBUF 的调用也不带 F_DMA3**，
 * 于是走 else 分支做了 invalidate：cache 行被作废，**脏数据直接丢弃**，
 * 物理内存里还是旧内容（全 0）。等真正写页时 DMA 读到 0，
 * 写进 flash 的自然就是 0 —— 读回来「00 00 00…」，1473/2048 不同。
 *
 * 修法：
 * · 写方向（F_DMA3）：clean，把页数据刷回内存
 * · 读方向（F_DST，DMA 往 DBUF 写）：invalidate，避免读到旧副本
 * · 其它（擦除 / SET FEATURE 等）：**什么都不要做**，别碰 DBUF
 */
 /* ★★★ 2026-11-03 定案：这里**什么 cache 操作都不要做**。
 *
 * 实测 SCTLR = 0x00C52878 —— MMU(M)、D-cache(C)、I-cache(I) 全是 0，
 * 也就是说 CPU 访问本来就是直通物理内存的，DMA 读到的就是 CPU 写的。
 *
 * 而 cache_clean(DCCMVAC) / cache_inv2(DCIMVAC) 都是**按 MVA 的**
 * cache 维护指令，在 MMU 关闭时行为是未定义的 —— 之前在这里调用它们，
 * 反而把刚写进 DBUF 的页数据破坏了（症状：写页读回全 FF，
 * 而 CPU 回读 DBUF 却是对的）。
 *
 * 代价：性能。但 FDL2 的瓶颈在 USB 和 NAND，不在内存访问。
 *
 * ★ 但 `dsb sy` 必须有！CPU 的 str 写出去之后可能还停在写缓冲里，
 * 没有内存屏障的话 NFI 的 DMA 读不到（症状：DBUF 里明明是 page0，
 * 写进 flash 的却是擦除态 FF）。原来这行屏障藏在 cache_inv2 末尾，
 * 删 cache 操作时差点把它一起删掉。 */
    cache_inv2(DBUF, psiz);          /* ★ 恢复：实测去掉它写页就全 FF */
    __asm__ volatile("dsb sy" ::: "memory");

 /* ★★★ F_STARTB 必须**优先于** F_FAITH。
 * 2026-11-03 实测：两者同时给时走了 F_FAITH 的 0x2801，四组写实验
 * 全部 nf_wait 超时（ret=1）。0x2800/0x2801 这个 burst 值在本控制器上
 * 是挂死值，只有 0x1001（控制器查表算出来的）能用。 */
    if ((mode & F_STARTB) != 0u)
 NFI_START = 0x1001u; /* ★ 控制器的启动值 */
    else if ((mode & F_FAITH) != 0u)
        NFI_START = 0x2801u;                            /* burst | 1 */
    else if (startmode == 0u)
        NFI_START = ((mode & F_START8) != 0u) ? 8u : 1u;
    else
        NFI_START = (NFI_START & ~1u) | 1u;

    i = (u32)nf_wait();
    g_stat_after = NFI_STAT;
    return (int)i;
}

/* ================================================================
 * ★ 多步执行：控制器读页前会先发两条 SET FEATURE（各自 start+wait）
 * 清状态 ; NFI[0]=1 ; wait
 * 然后才发读页命令。
 * cmdset 9 = SET FEATURE A → SET FEATURE B → 读页
 * ================================================================ */

/* ==================================================================
 * ★★★★★ 2026-11-08（p86）：软件 ECC 生成（给厂家工具用）
 *
 * 背景：厂家工具只发主区数据、不发 OOB。硬件生成 OOB 的两条路都实测失败
 * （控制器寄存器 11-05、芯片片上 ECC 11-08），所以只能软件算。
 *
 * 实现：8193 行映射不是死数据 —— 它由**224 阶线性递推**生成
 * （Berlekamp-Massey 求得，8192/8193 行完全还原；第 8193 行是常数行，本就全 0）。
 * 所以镜像里只带 224 行种子 + 224 位系数（约 6.3 KB），
 * 启动后第一次用到时展开成 229 KB 的查表（一次性）。
 *
 * 验证：与 ecc_map.npz 逐行一致；与 build_oobs() 抽样 438 页 100% 一致。
 * ================================================================== */
#include "ecc_tab.h"

#define ECC_WORDS 7u          /* 224 位 = 7 × u32 = 28 字节 */
#define ECC_ROWS  8193u

static u32 ecc_tab[ECC_ROWS][ECC_WORDS];
static u32 ecc_ready = 0u;

void ecc_build_pub(void);

static void ecc_build(void)
{
    u32 i, k, w;

    if (ecc_ready != 0u)
        return;

    for (i = 0; i < ECC_RECUR_ORDER; i++)
        for (w = 0; w < ECC_WORDS; w++)
            ecc_tab[i][w] = ecc_seed[i][w];

    for (i = ECC_RECUR_ORDER; i < ECC_ROWS; i++) {
        u32 acc[ECC_WORDS] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u };
        for (k = 0; k < ECC_RECUR_ORDER; k++) {
            if (((ecc_recur[k >> 5] >> (k & 31u)) & 1u) != 0u) {
                const u32 *p = ecc_tab[i - 1u - k];
                for (w = 0; w < ECC_WORDS; w++)
                    acc[w] ^= p[w];
            }
        }
        for (w = 0; w < ECC_WORDS; w++)
            ecc_tab[i][w] = acc[w];
    }
    ecc_ready = 1u;
}

void ecc_build_pub(void);

/* 由 2048 字节的一页主区数据，生成 78 字节 OOB（结构见 ECC 文档） */
static void ecc_fill_oob(const unsigned char *pg, unsigned char *oob)
{
    u32 a0[ECC_WORDS] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u };   /* 段0：0~1023 */
    u32 a1[ECC_WORDS] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u };   /* 段1：1024~2047 */
    u32 i, w;

    ecc_build();

 /* ★ p90：改成「字节作外层、位作内层」。
 *
 * 原来是 8192 次外层，每次都要 i>>3 / i&7 —— 同一个字节被读了 8 遍，
 * 每遍都要移位取位。现在外层只有 1024 次，一个字节读一次、内层走 8 位，
 * 内存读少 8 倍。位序断言：新索引 i*8+k 恒等于原来的位序号，结果等价。
 */
    for (i = 0; i < 1024u; i++) {
        u32 b0 = pg[i];
        u32 b1 = pg[1024u + i];
        u32 k;

        if ((b0 | b1) == 0u)
            continue;
        for (k = 0; k < 8u; k++) {
            int v0 = (int)((b0 >> k) & 1u);
            int v1 = (int)((b1 >> k) & 1u);
            if (v0 | v1) {
                const u32 *p = ecc_tab[i * 8u + k];
                for (w = 0; w < ECC_WORDS; w++) {
                    if (v0) a0[w] ^= p[w];
                    if (v1) a1[w] ^= p[w];
                }
            }
        }
    }

    oob[0] = 0xFFu; oob[1] = 0xD4u; oob[2] = 0x40u; oob[3] = 0xFFu;
    for (w = 0; w < ECC_WORDS; w++) {
        oob[4u + w * 4u + 0u] = (unsigned char)(a0[w]      );
        oob[4u + w * 4u + 1u] = (unsigned char)(a0[w] >>  8);
        oob[4u + w * 4u + 2u] = (unsigned char)(a0[w] >> 16);
        oob[4u + w * 4u + 3u] = (unsigned char)(a0[w] >> 24);
    }
    for (i = 32u; i < 39u; i++) oob[i] = 0xFFu;
    oob[39] = 0xFFu; oob[40] = 0xD4u; oob[41] = 0x40u; oob[42] = 0xFFu;
    for (w = 0; w < ECC_WORDS; w++) {
        oob[43u + w * 4u + 0u] = (unsigned char)(a1[w]      );
        oob[43u + w * 4u + 1u] = (unsigned char)(a1[w] >>  8);
        oob[43u + w * 4u + 2u] = (unsigned char)(a1[w] >> 16);
        oob[43u + w * 4u + 3u] = (unsigned char)(a1[w] >> 24);
    }
    for (i = 71u; i < 78u; i++) oob[i] = 0xFFu;
}

void ecc_build_pub(void)
{
    ecc_build();
}

static int nf_exec(u32 cfg, u32 psiz, u32 lenhi, u32 row, u32 col,
                   u32 cmdset, u32 startmode, u32 mode)
{
    int rr;

    if (cmdset == 9u) {
        rr = nf_one(cfg, 0x3000u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0)
            return rr;
        rr = nf_one(cfg, 0x3000u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0)
            return rr;
        return nf_one(cfg, psiz, lenhi, row, col, 0u, startmode, mode);
    }

 /* ★★★★★ cmdset 31 = 忠实读页全流程（控制器）
 * SET FEATURE(0xA0=0) → SET FEATURE(0xB0=9) → 0x6B 读页
 * 三个步骤各自 start + wait，与 的行为一致
 */
    if (cmdset == 31u) {
        rr = nf_one(cfg, 0x3000u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0)
            return rr;
        rr = nf_one(cfg, 0x3000u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0)
            return rr;
        return nf_one(cfg, psiz, lenhi, row, col, 32u, startmode,
                      mode | F_F8 | F_LEN | F_FAITH);
    }

 /* ★★★★★ cmdset 50 = 擦除一个块（按控制器的工作方式实现）
 * 选它的理由：擦除**不需要传输任何数据**，完全绕开
 * 「描述符 DMA 不往内存写」这个还没解决的故障。
 * 验证判据：状态寄存器 0xC0 的 bit2 = E_FAIL，清零即擦除成功。
 *
 * row = 块首页号 = block * 64（块大小 128KB / 页 2048 = 64 页）
 */
    if (cmdset == 50u) {
        u32 tries, st;

 /* 1. SET FEATURE(0xA0, 0) —— 解除块保护 */
        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0)
            return rr;

 /* 2. SET FEATURE(0xB0, 0) —— 擦除用 0（读/写才用 9）*/
        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 40u, 0u, 0u);
        if (rr != 0)
            return rr;

 /* 3. 擦除主体：cfg=0x8102（写方向），NFI[8]=0x3000，NFI[0xF8]=0x3A */
        rr = nf_one(0x8102u, 2048u, 0u, row, 0u, 41u, 0u, F_F8);
        if (rr != 0)
            return rr;

 /* 4. 轮询芯片状态 0xC0 —— ★ 必须用 cfg=0x8002（控制器），
 * 不是擦除主体的 0x8102；也不改写 NFI[0x0F8]。
 * ★ 先等 tBERS 再读：立刻读会读到 OIP=0，误判擦完。 */
        nf_delay(60000u);              /* ≈12ms，块擦典型 2~5ms */
        st = 0xFFu;
        for (tries = 0u; tries < 300u; tries++) {
            rr = nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u)
                break;
            nf_delay(20000u);          /* ~4ms，擦除典型 2~5ms */
        }
        g_poll_status = st | ((tries & 0xFFu) << 16);   /* 高字节 = 轮询次数 */
        return rr;
    }

 /* ★★★★★ cmdset 51 = WREN 自检：先 WREN，再读状态，WEL(bit1) 必须置 1
 * 这是「状态寄存器读得到」的判据 —— 没有它，擦除报成功也不可信。
 */
    if (cmdset == 51u) {
        rr = nf_one(0x8002u, 2048u, 0u, 0u, 0u, 45u, 0u, 0u);
        if (rr != 0)
            return rr;
        rr = nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
        g_poll_status = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
        return rr;
    }

 /* ★★★★★ cmdset 53 = 写一页（512 字节，单扇区）
 * 先把 DBUF 填成 0x5A 图案（绝不能是 0xFF，否则编程等于没写），
 * 再 cache clean（写方向：主机内存 → 控制器），然后走写页序列。
 * 判据：状态 0xC0 的 P_FAIL(bit3) 必须为 0。
 */
    if (cmdset == 53u) {
        u32 k;

        g_wr_seq++;
        for (k = 0u; k < 512u; k += 4u)
 *(volatile u32 *)(DBUF + k) =
                0x5A5A0000u ^ (row + k) ^ (g_wr_seq * 0x01010101u);
        cache_clean(DBUF, 512u);

        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 10u, 0u, 0u);   /* SET FEATURE A0=0 */
        if (rr != 0)
            return rr;
        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 11u, 0u, 0u);   /* SET FEATURE B0=9 */
        if (rr != 0)
            return rr;

        rr = nf_one(0x8178u, psiz, lenhi, row, col, 46u, 0u,
                    F_DESC | F_DMA3 | F_LEN | F_F8);
        if (rr != 0)
            return rr;

        rr = nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
        g_poll_status = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
        return rr;
    }

 /* ★★★★★ cmdset 71 = 整页读 2048 字节（一次搬完，不循环扇区）
 * NFI[0x008] = 0x3000|(2048-1) = 0x37FF，与 0x3800(收 2048) 自洽
 */
    if (cmdset == 71u) {
        u32 tries, st;

        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0) return rr;

        rr = nf_one(0x02u, 0x3000u, 0u, row, 0u, 21u, 0u, 0u);
        if (rr != 0) return rr;

        st = 0xFFu;
        for (tries = 0u; tries < 300u; tries++) {
            nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u) break;
            nf_delay(20000u);
        }

        rr = nf_one(0x78u, 2048u, 0u, row, col, 71u, 0u,
                    F_DESC | F_LEN | F_F8);
        g_poll_status = st | ((tries & 0xFFu) << 16);
        return rr;
    }

 /* ★★★★★ cmdset 72 = 整页写 2048 字节（一次发完）
 * 图案 = 0xA5A50000 ^ (row+k) ^ (seq*0x01010101)，seq 每次递增保证不同
 */
    if (cmdset == 72u) {
        u32 k;

        g_wr_seq++;
        for (k = 0u; k < 2048u; k += 4u)
 *(volatile u32 *)(DBUF + k) =
                0xA5A50000u ^ (row + k) ^ (g_wr_seq * 0x01010101u);
        cache_clean(DBUF, 2048u);

        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 76u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0) return rr;

        rr = nf_one(0x8178u, 2048u, 0u, row, col, 72u, 0u,
                    F_DESC | F_DMA3 | F_LEN | F_F8);
        if (rr != 0) return rr;

        rr = nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
        g_poll_status = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
        return rr;
    }

 /* ★★★★★ cmdset 80 = 整页读 2048 字节（4×512 扇区循环）
 *
 * 为什么不用一次 2048：实测单次 0x3800 + NFI[8]=0x37FF 会永久卡死
 * （START=0x40000000，bit4 永不置位）。控制器处读 cache
 * 的 FIFO 值硬编码就是 0x3200（512 字节），它是按扇区循环的。
 * ⇒ 每轮：描述符 A = DBUF + s*512，NFI[8] = 0x3000|(512-1) = 0x31FF
 */
    if (cmdset == 80u) {
        u32 tries, st, s;

        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 18u, 0u, 0u);
        if (rr != 0) return rr;

        rr = nf_one(0x02u, 0x3000u, 0u, row, 0u, 21u, 0u, 0u);
        if (rr != 0) return rr;

        st = 0xFFu;
        for (tries = 0u; tries < 300u; tries++) {
            nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u) break;
            nf_delay(20000u);
        }

        for (s = 0u; s < 4u; s++) {
            u32 c = s * 512u;
            rr = nf_one(0x78u, 512u, 0u, row, c, 70u, 0u,
                        F_DESC | F_LEN | F_F8 | F_DSTO);
            if (rr != 0) {
                g_poll_status = st | ((tries & 0xFFu) << 16);
                return rr;
            }
        }

 /* 4 段 DMA 都写完了，整页丢掉 cache 行，让 CPU 读到真数据 */
        cache_inv(DBUF, 2048u);
        g_poll_status = st | ((tries & 0xFFu) << 16);
        return 0;
    }

 /* ★★★★★ cmdset 81 = 整页写 2048 字节（4×512 扇区循环）
 * WREN 只发一次 → 0x32 发 4 次（列 0/512/1024/1536）→ 最后一次
 * 同事务里带 0x10 PROGRAM EXECUTE（就是已验证过的 case 46 形态）
 */
    if (cmdset == 81u) {
        u32 k, s, tries, st;

        g_wr_seq++;
        for (k = 0u; k < 2048u; k += 4u)
 *(volatile u32 *)(DBUF + k) =
                0xA5A50000u ^ (row + k) ^ (g_wr_seq * 0x01010101u);
        cache_clean(DBUF, 2048u);

        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 76u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0) return rr;

        for (s = 0u; s < 4u; s++) {
            u32 c = s * 512u;

 /* ★★★★★ 每一轮都是「WREN + LOAD + EXECUTE」完整一套，不能攒着。
 *
 * 实测：4 次 LOAD 攒着、最后 EXECUTE 一次 ⇒ 只有最后一个扇区
 * 的内容写进去了（前 1536 字节读回全 FF）。原因是这颗
 * UM19A8LISW 的 cache 寄存器在 NFI 事务之间会被清成 0xFF，
 * 上一轮的 512 字节留不住。
 *
 * 控制器就是这么干的：推 0x32 之后，
 * 立刻推 0x10 EXECUTE，再跳回循环头。
 * 编程 0xFF 不会改变已有位，所以逐扇区编程不会破坏前面的扇区。
 */
            rr = nf_one(0x8178u, 512u, 0u, row, c, 46u, 0u,
                        F_DESC | F_DMA3 | F_LEN | F_F8 | F_DSTO);
            if (rr != 0) return rr;

 /* 等本扇区编程完成再动下一扇区。
 * ★ 先等 tPROG 再读状态：EXECUTE 完立刻读会读到 OIP=0（还没置起来），
 * 误判"已完成"就发下一扇区，会把上一次编程打断，产生零星位错误。 */
            nf_delay(10000u);              /* ≈2ms */
            st = 0xFFu;
            for (tries = 0u; tries < 300u; tries++) {
                nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                if ((st & 1u) == 0u) break;
                nf_delay(20000u);
            }
            if ((st & 0x0Cu) != 0u) {          /* E_FAIL 或 P_FAIL */
                g_poll_status = st | (s << 8);
                return 5;
            }
        }

        g_poll_status = st;
        return 0;
    }

 /* ★★★ cmdset 82 = 整页读 2048 单次（实验）：NFI[4] 高字节 = 扇区数-1 = 3
 * 控制器处 NFI[0x004] = ((count-1)<<24) | cfg
 * 之前传 2048 字节却用高字节 0（=1 扇区），控制器等不满就卡死 */
    if (cmdset == 82u) {
        u32 tries, st;

        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 0x3000u, 0u, row, 0u, 21u, 0u, 0u);
        if (rr != 0) return rr;

        st = 0xFFu;
        for (tries = 0u; tries < 300u; tries++) {
            nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u) break;
            nf_delay(20000u);
        }

        rr = nf_one(0x03000078u, 2048u, 0u, row, 0u, 71u, 0u,
                    F_DESC | F_LEN | F_F8);
        g_poll_status = st | ((tries & 0xFFu) << 16);
        return rr;
    }

 /* ★★★ cmdset 83 = 整页写 2048 单次（实验）：同上，cfg 高字节 = 3 */
    if (cmdset == 83u) {
        u32 k;

        g_wr_seq++;
        for (k = 0u; k < 2048u; k += 4u)
 *(volatile u32 *)(DBUF + k) =
                0xA5A50000u ^ (row + k) ^ (g_wr_seq * 0x01010101u);
        cache_clean(DBUF, 2048u);

        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 76u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0) return rr;

        rr = nf_one(0x03810078u, 2048u, 0u, row, 0u, 72u, 0u,
                    F_DESC | F_DMA3 | F_LEN | F_F8);
        if (rr != 0) return rr;

        rr = nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
        g_poll_status = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
        return rr;
    }

 /* ★★★★★ cmdset 84 = 整页写，但**不填测试图案**
 * 直接用 DBUF 里现有的 2048 字节（BSL 写 flash 用）。
 * 和 cmdset 81 完全相同的时序，只是去掉填充。
 */
    if (cmdset == 84u) {
        u32 s, tries, st;

        cache_clean(DBUF, 2048u);

 /* ★★★★★ p86：主机没给 OOB（厂家工具就是这样）⇒ 固件自己算 78 字节，
 * 填进 OBUF，然后走与「主机给 OOB」**完全相同**的路径
 * （那条路径已验证 4376/4376 页全对），风险最小。 */
        g_oob_local = 0u;
        if (g_oob_w == 0u) {
            ecc_fill_oob((const unsigned char *)DBUF, (unsigned char *)OBUF);
            cache_clean(OBUF, 128u);
            g_oob_wlen = 78u;
            g_oob_local = 1u;
        }

        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 76u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 18u, 0u, 0u);
        if (rr != 0) return rr;

 /* ★★★★★ 按控制器的工作方式实现：先把这一页读进 cache
 *
 * 为什么必须有这一步：PROGRAM LOAD(0x32) **只装主区**。
 * 芯片 cache 里还有一块「备用区（OOB）」64 字节，PROGRAM EXECUTE(0x10)
 * 会把主区 + 备用区**一起**编程下去。所以备用区写什么，
 * 取决于 cache 里现在是什么：
 *
 * - 控制器：先 PAGE READ ⇒ 备用区 = 这一页阵列里的原内容 ⇒ 保住 OOB
 * - 我们（改之前）：什么都不做 ⇒ 备用区 = 上一次操作的残留 ⇒ 写垃圾
 *
 * 而刷机是**先擦后写**，擦完这一页读出来主区备用区都是 0xFF
 * ⇒ PAGE READ 之后备用区 = 全 FF ⇒ 写下去也是 FF，
 * 与控制器刷机的结果完全一致。
 *
 * ★ 顺带的好处：修补模式（g_wr_noerase=1，不擦块直接重编程）下，
 * 这一步天然就是「读-改-写」——原 OOB 被完整保住。
 */
        rr = nf_one(0x02u, 0x3000u, 0u, row, 0u, 21u, 0u, 0u);   /* PAGE READ 0x13 */
        if (rr != 0) return rr;

        st = 0xFFu;
        for (tries = 0u; tries < 400u; tries++) {
            nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u) break;
            nf_delay(2000u);       /* ≈0.4ms；tR 只有几十微秒 */
        }
 /* PAGE READ 只动芯片 cache，不碰 DBUF，所以上面的 cache_clean 仍然有效 */

 /* ---- 策略 6：整页单事务（512×4 拍 + 备用区，一次 EXECUTE）----
 *
 * 这是控制器的真实形态。三个关键点（2026-11-07 复核）：
 * · cfg 高字节 = 扇区数-1 ⇒ 4 扇区 = 0x03 ⇒ cfg = 0x03810078
 * · 描述符 A 长度恒为 512（0x2200），多拍由控制器自己走
 * · NFI[0x008] 低 12 位 = 整页字节数-1（0x7FF），不是 511
 *
 * 预期：一页只编程 1 次 ⇒ 单比特翻转 / OOB 扰动 / 速度慢 一起消失。
 */
        if (g_wr_mode == 17u || g_wr_mode == 18u) {
 /* ★★★ 分析 得到的关键：
 * cfg = ((count-1)<<24) | ((v15-1)<<16) | 0x8178
 * ⇒ 4 个扇区时 = 0x03038178（bits16-23 也是「个数-1」，一直是 0）
 * NFI[8] = 0x3000 | (页大小-1) | ((OOB-1)<<16) = 0x003F37FF
 * NFI[0] = 0x2801（burst|1；burst 基准 0x2800 由查表得出）
 * 17 = 启动值 0x2801 ｜ 18 = 启动值 1（对照） */
            u32 m17 = F_DESC | F_LENRAW | F_F8 | F_NOMARK |
                      F_DSTB | F_DSTOB;
            if (g_wr_mode == 17u) m17 |= F_FAITH;
            rr = nf_one(0x03038178u, 2048u, 0x003F37FFu, row, 0u, 99u, 0u, m17);
            if (rr != 0) return rr;
            nf_delay(6000u);
            st = 0xFFu;
            for (tries = 0u; tries < 400u; tries++) {
                nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                if ((st & 1u) == 0u) break;
                nf_delay(2000u);
            }
            g_poll_status = st;
            return ((st & 0x0Cu) != 0u) ? 5 : 0;
        }

        if (g_wr_mode == 16u) {
 /* 完整实现：cfg 0x8178、NFI[8]=0x003F37FF、启动值 0x2801 */
            u32 m16 = F_DESC | F_FAITH | F_LENRAW | F_F8 | F_NOMARK |
                      F_DSTB | F_DSTOB;
            rr = nf_one(0x00008178u, 2048u, 0x003F37FFu, row, 0u, 99u, 0u, m16);
            if (rr != 0) return rr;
            nf_delay(6000u);
            st = 0xFFu;
            for (tries = 0u; tries < 400u; tries++) {
                nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                if ((st & 1u) == 0u) break;
                nf_delay(2000u);
            }
            g_poll_status = st;
            return ((st & 0x0Cu) != 0u) ? 5 : 0;
        }

        if ((g_wr_mode >= 6u && g_wr_mode <= 10u) ||
            g_wr_mode == 14u || g_wr_mode == 15u) {
 /* mode 7 = 对照：单扇区（cfg 0x8178，1 拍），验证这条新通道本身通不通
 * mode 6 = 整页 4 拍，**不带**备用区（cfg 0x03810078，NFI[8]=0x0037FF）
 * mode 8 = 整页 4 拍 + 备用区（NFI[8]=0x004D37FF，cmdset 95）
 */
 /* ★ 2026-11-08 00:10 修正：cfg 高字节 = **页数-1**，不是扇区数-1。
 * 写一页 ⇒ 高字节 = 0 ⇒ cfg = 0x8178（不是 0x03008178）。
 * 依据：的 a6 是页数（cmp r5,#8），
 * 里 (r7-1)<<24 的 r7 也是页数。 */
            u32 cfg6 = (g_wr_mode == 7u || g_wr_mode == 10u ||
                        g_wr_mode >= 14u)
                       ? 0x00008178u : 0x03008178u;
            u32 cs6  = (g_wr_mode == 8u || g_wr_mode == 15u) ? 95u :
                       ((g_wr_mode == 9u) ? 98u : 96u);
            u32 m6   = F_DESC | F_LENRAW | F_F8 | F_NOMARK;
            u32 len6;

            if (g_wr_mode == 7u || g_wr_mode == 9u || g_wr_mode == 10u) {
                len6 = 0x3000u | 0x1FFu;                     /* 512 字节 */
            } else if (g_wr_mode == 14u) {
                len6 = 0x3000u | 0x7FFu;                     /* 2048，一拍 4 次 */
            } else if (g_wr_mode == 15u) {
                len6 = 0x3000u | 0x7FFu |
                       (((g_oob_wlen - 1u) & 0xFFu) << 16);  /* 2048 + OOB */
                m6 |= F_DSTB | F_DSTOB | F_ECC;
            } else if (g_wr_mode == 6u) {
                len6 = 0x3000u | 0x7FFu;                     /* 2048，无备用区 */
            } else {
                len6 = 0x3000u | 0x7FFu |
                       (((g_oob_wlen - 1u) & 0xFFu) << 16);  /* 2048 + 78 */
                m6 |= F_DSTB | F_DSTOB | F_ECC;
            }
            rr = nf_one(cfg6, 2048u, len6, row, 0u, cs6, 0u, m6);
            if (rr != 0) return rr;
            nf_delay(6000u);
            st = 0xFFu;
            for (tries = 0u; tries < 400u; tries++) {
                nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                if ((st & 1u) == 0u) break;
                nf_delay(2000u);
            }
            g_poll_status = st;
            return ((st & 0x0Cu) != 0u) ? 5 : 0;
        }

 /* ---- 策略 4：4 次 PROGRAM LOAD 攒进芯片 cache，最后 1 次 EXECUTE ----
 *
 * 目的：把「每页编程 4 次」降到「每页编程 1 次」。
 * 0x10 EXECUTE 会把主区 + 备用区一起编程，片内 ECC 也会跟着重算；
 * 一页编程 4 次 ⇒ OOB 里留下的是最后一次（cache 只剩 512 字节有效
 * 数据）的校验值 ⇒ 设备启动读 UserImg 时校验对不上。
 *
 * 为什么不用 mode 2 的「一次 2048 字节 DMA」：实测第 4096 字节
 * 就 MIDST 失败 —— 控制器单笔搬运不能超过 512 字节（扇区粒度）。
 * 所以这里保持每笔 512 字节，靠「只装 cache 不编程」把 4 笔串起来：
 * s=0,1,2 → case 49（只有 0x32 PROGRAM LOAD）
 * s=3 → case 46（WREN + LOAD + EXECUTE + 读状态）
 * 每笔事务的 DMA 长度都是 512，不触碰那个挂死边界。
 *
 * ★ 前提：芯片 cache 在多个 NFI 事务之间不被清掉。
 * 这一条尚未验证，所以 mode 4 不是默认值，先用 --sweep 试。 */
        if (g_wr_mode == 4u) {
            for (s = 0u; s < 4u; s++) {
                u32 c  = s * 512u;
                u32 cs = (s == 3u) ? 46u : 49u;
                rr = nf_one(0x8178u, 512u, 0u, row, c, cs, 0u,
                            F_DESC | F_DMA3 | F_LEN | F_F8 | F_DSTO);
                if (rr != 0) return rr;
            }
            nf_delay(6000u);
            st = 0xFFu;
            for (tries = 0u; tries < 400u; tries++) {
                nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                if ((st & 1u) == 0u) break;
                nf_delay(2000u);
            }
            g_poll_status = st;
            return ((st & 0x0Cu) != 0u) ? 5 : 0;
        }

 /* ---- 策略 5：1024 字节扇区 × 2（文档 §十八 的正确扇区）----
 *
 * 把每页编程次数从 4 次降到 2 次（再加 OOB 那次共 3 次），
 * 目标是消掉「单比特翻转」和「页 0/1 的 OOB 被扰乱」。
 * cfg = 0x01008178（2 扇区），NFI[0x008] = 0x3000|1023 = 0x33FF。
 */
        if (g_wr_mode == 5u) {
            g_wr_sector = 1024u;
            for (s = 0u; s < 2u; s++) {
                u32 c = s * 1024u;
                u32 m = F_DESC | F_DMA3 | F_F8 | F_DSTO | F_NOMARK | F_LEN;
                rr = nf_one(0x01008178u, 1024u, 0u, row, c, 46u, 0u, m);
                if (rr != 0) { g_wr_sector = 512u; return rr; }

                nf_delay(10000u);
                st = 0xFFu;
                for (tries = 0u; tries < 300u; tries++) {
                    nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                    st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                    if ((st & 1u) == 0u) break;
                    nf_delay(20000u);
                }
                if ((st & 0x0Cu) != 0u) {
                    g_poll_status = st | (s << 8);
                    g_wr_sector = 512u;
                    return 5;
                }
            }
            g_wr_sector = 512u;
            g_poll_status = st;
            return 0;
        }

 /* ---- 策略 2 / 3：整页一次 LOAD(2048) + 一次 EXECUTE ---- */
        if (g_wr_mode == 2u || g_wr_mode == 3u) {
            u32 cfg = (g_wr_mode == 3u) ? 0x03810078u : 0x810078u;
            rr = nf_one(cfg, 2048u, g_wr_lenhi, row, 0u, 72u, 0u,
                        F_DESC | F_DMA3 | F_LEN | F_F8);
            if (rr != 0) return rr;
 /* ★ 固定延时从 20ms 降到 1.2ms：4376 页 × 20ms = 87s，
 * 光这一条就把整分区写入拖成 85 秒。tPROG 量级 0.4~0.8ms，
 * 1.2ms 起步后再用 OIP 轮询兜底。 */
            nf_delay(6000u);
            st = 0xFFu;
            for (tries = 0u; tries < 400u; tries++) {
                nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                if ((st & 1u) == 0u) break;
                nf_delay(2000u);
            }
            g_poll_status = st;
            return ((st & 0x0Cu) != 0u) ? 5 : 0;
        }

 /* ---- 策略 0 / 1：4×512 逐扇区编程 ---- */
        for (s = 0u; s < 4u; s++) {
            u32 c = s * 512u;
 /* ★ g_wr_lenhi = 0 时与原行为完全一致；非 0 时把备用区字节数
 * 写进 NFI[0x008] 高字节，让控制器把 ECC 校验值落到 OOB。 */
 /* ★★★ 必须带 F_NOMARK！
 * nf_one 在事务开始前会往 DBUF 前 64 字节打 0xAA 哨兵（诊断用），
 * 而 DBUF 里装的正是要写进 flash 的页数据 —— 不打 F_NOMARK 就会
 * 把首页前 64 字节写成 AA。实测症状：扇区 1/2/3 零错，
 * 扇区 0 的前 64 字节全是 AA（64/2048 不同）。 */
            {
                u32 m = F_DESC | F_DMA3 | F_F8 | F_DSTO | F_NOMARK;
                u32 lh = g_wr_lenhi;
 /* ★ 2026-11-08：g_oob_w 时把备用区并入**同一个事务**
 * （控制器形态：0x32 主区 → 0x34 备用区 → 一次 0x10）
 * NFI[0x008] = 0x3000 | (扇区字节-1) | ((OOB字节-1) << 16)
 * 78 字节 → 0x004D31FF
 * 这样一页只编程 4 次，不再有第 5 次 OOB 编程造成的 disturb。 */
                if (g_oob_w != 0u || g_oob_local != 0u) {
                    m |= F_OOBW | F_DSTB | F_DSTOB | F_LENRAW;
                    lh = 0x3000u | (g_wr_sector - 1u) |
                         (((g_oob_wlen - 1u) & 0xFFu) << 16);
                } else if (g_wr_lenhi != 0u) {
                    m |= F_LENRAW;
                } else {
                    m |= F_LEN;
                }
                rr = nf_one(0x8178u, 512u, lh, row, c, 46u, 0u, m);
            }
            if (rr != 0) return rr;

 /* ★★★ 必须先等 tPROG 再读状态。
 * 以前是 EXECUTE 完立刻 GET FEATURE，OIP 位这时还没置起来，
 * 第一次轮询就读到"已完成"，于是上一次编程没结束就发下一次
 * LOAD/EXECUTE，造成零星位错误（实测 256KB 里 55 字节错，
 * 且是 1→0 方向，正是编程被打断的特征）。
 * nf_delay 是纯 nop，1M≈0.2s ⇒ 10000≈2ms，足够 512B 编程。 */
            if (g_wr_mode == 20u) {
 /* ★★★ P1：整页自动快慢路径（2026-11-08）
 *
 * 实测（同一会话、整块 64 页、页序 [2..63,0,1]）：
 * mode 0（本分支的慢路径） nfi84 = 11.09 ms/页、主区 0/64 错
 * mode 19（零延时零轮询） nfi84 = 2.94 ms/页、主区 0/64 错
 * ⇒ 快路径本身是安全的；之前 8 页测试里页 0/1 报错来自**页序干扰**。
 *
 * 仍然给块内页 0/1 保留慢路径：代价 2 页 × 2.5ms × 69 块 ≈ 0.35s，
 * 可以忽略，但能兜住调用方页序不同（例如顺序写）的情况。
 */
                if ((row & 63u) < 2u) {
                    nf_delay(10000u);
                    st = 0xFFu;
                    for (tries = 0u; tries < 300u; tries++) {
                        nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                        st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                        if ((st & 1u) == 0u) break;
                        nf_delay(20000u);
                    }
                } else {
                    st = 0u;
                }
            } else if (g_wr_mode == 19u) {
 /* ★ 完全按控制器节奏：不加延时、不额外读状态，
 * 只等 nf_one 内部那次 NFI 事务完成（约 0.2ms）。
 * 控制器每扇区就是这么连发的。 */
                st = 0u;
            } else if (g_wr_mode == 12u || g_wr_mode == 13u) {
 /* ★ 只靠固定延时，**不轮询**：每次状态读都是一次 USB 往返，
 * 比 2ms 死等还贵。整页写完后再统一查一次状态。
 * 12 = 1.0ms/扇区，13 = 0.6ms/扇区 */
                nf_delay((g_wr_mode == 12u) ? 5000u : 3000u);
                st = 0u;
            } else if (g_wr_mode == 11u) {
 /* ★ 省掉固定 2ms 死等：先确认看到 OIP=1，再等它落下。
 * 只等一次「忙→闲」就退出；8 次轮询内没见到忙就认为已经写完
 * （tPROG 很短，可能在我们读之前就结束了）。 */
                u32 busy11 = 0u;
                nf_delay(1000u);                       /* ≈0.2ms */
                st = 0xFFu;
                for (tries = 0u; tries < 40u; tries++) {
                    nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                    st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                    if ((st & 1u) != 0u) {
                        busy11 = 1u;
                        nf_delay(2000u);
                        continue;
                    }
                    if (busy11 != 0u || tries >= 8u) break;
                    nf_delay(1000u);
                }
            } else {
                nf_delay((g_wr_mode == 1u) ? 100000u : 10000u);

                st = 0xFFu;
                for (tries = 0u; tries < 300u; tries++) {
                    nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
                    st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
                    if ((st & 1u) == 0u) break;
                    nf_delay(20000u);
                }
            }
            if ((st & 0x0Cu) != 0u) {
                g_poll_status = st | (s << 8);
                return 5;
            }
        }

        g_poll_status = st;
        return 0;
    }

 /* ★★★★★ cmdset 85 = 只读一页的 OOB（备用区 64 字节）—— 判决实验用，只读不写
 *
 * 目的：判断「控制器FDL2 刷过的分区，OOB 里到底有没有东西」。
 * 若全 FF ⇒ OOB 与刷机无关，我们只写主区的实现已经等价，不必做读-改-写。
 *
 * 流程与读页相同，只有最后一步换成「读备用区」：
 * SET_FEATURE(A0,0) → SET_FEATURE(B0,0x19) → PAGE READ → 等 OIP
 * → cfg=0x78, NFI[0x008]=0x303F(64字节), 列 = 0x8000|col, 0x5040 收 64B
 *
 * 数据落在 DBUF（F_DSTB 让描述符B 指向 DBUF）。
 */
    if (cmdset == 85u) {
        u32 tries, st;

        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0) return rr;
        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 18u, 0u, 0u);   /* B0 = g_b0_val = 0x19 */
        if (rr != 0) return rr;

 /* PAGE READ：整页（主区+备用区）一起搬进芯片 cache */
        rr = nf_one(0x02u, 0x3000u, 0u, row, 0u, 21u, 0u, 0u);
        if (rr != 0) return rr;

        st = 0xFFu;
        for (tries = 0u; tries < 300u; tries++) {
            nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u) break;
            nf_delay(20000u);
        }

 /* 读 cache 备用区：列带 |0x8000，描述符B 收 64 字节 */
        rr = nf_one(0x78u, 64u, 0u, row, (col | 0x8000u), 73u, 0u,
                    F_DESC | F_DSTB | F_LEN | F_F8);
        if (rr != 0) {
            g_poll_status = st | ((tries & 0xFFu) << 16);
            return rr;
        }

        cache_inv(DBUF, 64u);
        g_poll_status = st | ((tries & 0xFFu) << 16);
        return 0;
    }

 /* ★★★★★ cmdset 70 = 干净读页（长度对齐版，不带 F_FAITH burst）
 * SET FEATURE → PAGE READ → 等 OIP → 0x6B 读 cache（描述符 A 收 512B）
 * 关键：psiz 必须 = 512，与 0x3200 一致，否则控制器等不满就卡死
 */
    if (cmdset == 70u) {
        u32 tries, st;

        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0)
            return rr;
        rr = nf_one(0x02u, 0x3000u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0)
            return rr;

 /* PAGE READ：把阵列里的页搬进芯片 cache */
        rr = nf_one(0x02u, 0x3000u, 0u, row, 0u, 21u, 0u, 0u);
        if (rr != 0)
            return rr;

 /* 等 OIP 清零 */
        st = 0xFFu;
        for (tries = 0u; tries < 300u; tries++) {
            nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u)
                break;
            nf_delay(20000u);
        }

 /* 读 cache → 描述符 A 收 512 字节到 DBUF */
        rr = nf_one(0x78u, psiz, lenhi, row, col, 70u, 0u,
                    F_DESC | F_LEN | F_F8);
        g_poll_status = st | ((tries & 0xFFu) << 16);
        return rr;
    }

 /* ★ cmdset 61 = 只做 WREN + PROGRAM LOAD（看描述符 A 发数据卡不卡） */
    if (cmdset == 61u) {
        u32 k;

        g_wr_seq++;
        for (k = 0u; k < 512u; k += 4u)
 *(volatile u32 *)(DBUF + k) =
                0x5A5A0000u ^ (row + k) ^ (g_wr_seq * 0x01010101u);
        cache_clean(DBUF, 512u);

        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0)
            return rr;
        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0)
            return rr;
        rr = nf_one(0x8178u, psiz, lenhi, row, col, 47u, 0u,
                    F_DESC | F_DMA3 | F_LEN | F_F8);
        g_poll_status = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
        return rr;
    }

 /* ★ cmdset 62 = 只做 WREN + PROGRAM EXECUTE（看 0x10 执行卡不卡） */
    if (cmdset == 62u) {
        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 10u, 0u, 0u);
        if (rr != 0)
            return rr;
        rr = nf_one(0x02u, 2048u, 0u, 0u, 0u, 11u, 0u, 0u);
        if (rr != 0)
            return rr;
        rr = nf_one(0x8178u, psiz, lenhi, row, col, 48u, 0u,
                    F_DMA3 | F_LEN | F_F8);
        g_poll_status = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
        return rr;
    }

 /* cmdset 52 = 只读状态（对照组，同样用 cfg=0x8002） */
    if (cmdset == 52u) {
        rr = nf_one(0x8002u, 2048u, 0u, 0u, 0u, 42u, 0u, 0u);
        g_poll_status = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
        return rr;
    }

 /* ★★★ cmdset 18 = 控制器的两步流程
 * 第一步：NFI[4]=2 / NFI[0x0F8]=0x3A / NFI[8]=0 / NFI[0x0C]=0
 * 发 0x1F0, 0xFF，启动，等待完成
 * 第二步：配 cfg / 描述符 / NFI[8]=psiz-1
 * NFI[0x0F8] = 0x403A（bit14）
 * NFI[0x000] = 8（bit3）启动
 */
    if (cmdset == 18u) {
        rr = nf_one(cfg, 1u, 0u, 0u, 0u, 19u, 0u, F_F8 | F_LEN2);
        if (rr != 0)
            return rr;
        return nf_one(cfg, psiz, lenhi, row, col, 0u, startmode,
                      (mode & ~F_F8) | F_F8C | F_START8);
    }

 /* ★★★ cmdset 25 = SPI NAND 完整读页（两步，各自 start+wait）
 * 第一步：0x13 + 行地址 → PAGE READ（页搬到 cache），再读状态
 * 第二步：0x0B + 列地址 + dummy → READ FROM CACHE，DMA 取数据
 */
    if (cmdset == 25u) {
        rr = nf_one(cfg, psiz, lenhi, row, col, 22u, 0u, F_F82);
        if (rr != 0)
            return rr;
        return nf_one(cfg, psiz, lenhi, row, col, 23u, startmode, mode);
    }

 /* ★★★★ cmdset 27 = SPI NAND 完整读页 + 【状态轮询】
 * 状态寄存器 bit0 = OIP（操作进行中）。
 * PAGE READ 之后芯片是忙的，必须等 OIP 清零才能读 cache。
 * 之前没轮询 ⇒ 忙的时候就去读 cache ⇒ 拿到 0xFF / 超时。
 */
    if (cmdset == 27u) {
        u32 tries, st;

 /* 1. PAGE READ */
        rr = nf_one(cfg, psiz, lenhi, row, col, 21u, 0u, F_F82);
        if (rr != 0)
            return rr;

 /* 2. 轮询状态直到 OIP(bit0) 清零 */
        st = 0xFFu;
        for (tries = 0u; tries < 200u; tries++) {
            rr = nf_one(cfg, psiz, lenhi, row, col, 1u, 0u, F_F82);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u)
                break;
            nf_delay(2000u);
        }
        g_poll_status = st;

 /* 3. READ FROM CACHE + DMA */
        return nf_one(cfg, psiz, lenhi, row, col, 23u, startmode, mode);
    }

 /* ★★★★ cmdset 28 = SPI NAND 读页 + 状态轮询 + 【软件读 FIFO】
 * ReadID 能工作，靠的就是直接读 0x19C（一次 32 位 = 4 字节）。
 * 所以 cache 读也不该依赖 DMA，直接读 0x19C 512 次 = 2048 字节。
 */
    if (cmdset == 28u) {
        u32 tries, st, k;
        volatile u32 *dst = (volatile u32 *)DBUF;

 /* 1. PAGE READ */
        rr = nf_one(cfg, psiz, lenhi, row, col, 21u, 0u, F_F82);
        if (rr != 0)
            return rr;

 /* 2. 轮询状态直到 OIP(bit0) 清零 */
        st = 0xFFu;
        for (tries = 0u; tries < 200u; tries++) {
            nf_one(cfg, psiz, lenhi, row, col, 1u, 0u, F_F82);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u)
                break;
            nf_delay(2000u);
        }
        g_poll_status = st;

 /* 3. READ FROM CACHE（0x0B），不用 DMA */
        nf_one(cfg, psiz, lenhi, row, col, 24u, 0u, 0u);

 /* 4. 软件读 FIFO：512 个 32 位 = 2048 字节 */
        for (k = 0u; k < 512u; k++)
            dst[k] = *(volatile u32 *)(NFI_BASE + 0x19Cu);

        return 0;
    }

 /* ★★★★★ cmdset 29 = SPI NAND 读页 + 轮询 + 【按字节读 FIFO】
 * 实测：32 位读 0x19C 只出 1 个有效字节（低字节 0xFF，高 3 字节为 0）
 * ⇒ FIFO 是字节宽度的，要按字节读 2048 次。
 */
    if (cmdset == 29u) {
        u32 tries, st, k;
        volatile u8 *dst = (volatile u8 *)DBUF;

        rr = nf_one(cfg, psiz, lenhi, row, col, 21u, 0u, F_F82);
        if (rr != 0)
            return rr;

        st = 0xFFu;
        for (tries = 0u; tries < 200u; tries++) {
            nf_one(cfg, psiz, lenhi, row, col, 1u, 0u, F_F82);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u)
                break;
            nf_delay(2000u);
        }
        g_poll_status = st;

        nf_one(cfg, psiz, lenhi, row, col, 24u, 0u, 0u);

        for (k = 0u; k < 2048u; k++)
            dst[k] = *(volatile u8 *)(NFI_BASE + 0x19Cu);

        return 0;
    }

 /* ★★★★★ cmdset 30 = 先 SET FEATURE(0xB0=0x09)，再按字节读
 * 配置寄存器 0xB0 里有 BUF（buffer 模式）位，会改变读 cache 的语义。
 * 控制器读页路径开头正是 0x1F,0xB0,0x09。
 */
    if (cmdset == 30u) {
        u32 tries, st, k;
        volatile u8 *dst = (volatile u8 *)DBUF;

 /* SET FEATURE 0xB0 = 0x09 */
        nf_one(cfg, psiz, lenhi, row, col, 11u, 0u, F_F82);

 /* PAGE READ */
        rr = nf_one(cfg, psiz, lenhi, row, col, 21u, 0u, F_F82);
        if (rr != 0)
            return rr;

        st = 0xFFu;
        for (tries = 0u; tries < 200u; tries++) {
            nf_one(cfg, psiz, lenhi, row, col, 1u, 0u, F_F82);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u)
                break;
            nf_delay(2000u);
        }
        g_poll_status = st;

        nf_one(cfg, psiz, lenhi, row, col, 24u, 0u, 0u);

        for (k = 0u; k < 2048u; k++)
            dst[k] = *(volatile u8 *)(NFI_BASE + 0x19Cu);

        return 0;
    }

 /* ★★★★★ cmdset 31 = PAGE READ，地址字节【带 0x1000 标记】
 * 依据：能工作的命令里地址都带标记
 * 0x0F, 0xC0, 0x1000 ← 读状态（1 个带标记地址）
 * 0x9F, 0x00, 0x1000 ×4 ← 读 ID
 * 而我们之前发 0x13, hi, mid, lo 是裸字节 ⇒ 控制器可能当数据字节处理
 */
    if (cmdset == 31u) {
        u32 tries, st, k;
        volatile u8 *dst = (volatile u8 *)DBUF;

 /* PAGE READ + 带标记的 3 字节行地址 */
        nfc(0x13u);
        nfc(((row >> 16) & 0xFFu) | 0x1000u);
        nfc(((row >> 8) & 0xFFu)  | 0x1000u);
        nfc((row & 0xFFu)         | 0x1000u);
        nfc(0xE000u); nfc(0xF000u);
        NFI_STAT |= 0xF00u;
        NFI_START = 1u;
        (void)nf_wait();

 /* 轮询状态 */
        st = 0xFFu;
        for (tries = 0u; tries < 200u; tries++) {
            nf_one(cfg, psiz, lenhi, row, col, 1u, 0u, F_F82);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u)
                break;
            nf_delay(2000u);
        }
        g_poll_status = st;

 /* READ FROM CACHE：0x0B + 带标记的 2 字节列地址 + 1 个 dummy */
        nfc(0x0Bu);
        nfc(((col >> 8) & 0xFFu) | 0x1000u);
        nfc((col & 0xFFu)        | 0x1000u);
        nfc(0x1000u);
        nfc(0xE000u); nfc(0xF000u);
        NFI_STAT |= 0xF00u;
        NFI_START = 1u;
        (void)nf_wait();

        for (k = 0u; k < 2048u; k++)
            dst[k] = *(volatile u8 *)(NFI_BASE + 0x19Cu);

        return 0;
    }

 /* ★★★★★ cmdset 32 = 假设验证：0x1000 = 「从 SPI 读 1 字节进 FIFO」
 * 依据：ReadID 的序列是 0x9F, 0x00, 0x1000×4，然后读 0x19C 一次拿到 4 字节。
 * ⇒ 0x1000 的个数 = 读到的字节数。
 * 本命令：PAGE READ → 轮询 → 0x0B + 列地址 + dummy → 0x1000×32 → 读 8 个 32 位
 */
    if (cmdset == 32u) {
        u32 tries, st, k;
        volatile u32 *dst = (volatile u32 *)DBUF;

 /* PAGE READ */
        rr = nf_one(cfg, psiz, lenhi, row, col, 21u, 0u, F_F82);
        if (rr != 0)
            return rr;

 /* 轮询状态直到 OIP 清零 */
        st = 0xFFu;
        for (tries = 0u; tries < 200u; tries++) {
            nf_one(cfg, psiz, lenhi, row, col, 1u, 0u, F_F82);
            st = *(volatile u32 *)(NFI_BASE + 0x19Cu) & 0xFFu;
            if ((st & 1u) == 0u)
                break;
            nf_delay(2000u);
        }
        g_poll_status = st;

 /* READ FROM CACHE：0x0B + 2 字节列地址 + 1 dummy + 0x1000 ×32 */
        FIFO_IDX = 0u;
        nfc(0x0Bu);
        nfc((col >> 8) & 0xFFu);
        nfc(col & 0xFFu);
        nfc(0x1000u);                    /* dummy 字节 */
        for (k = 0u; k < 32u; k++)
            nfc(0x1000u);                /* ★ 读 32 个字节进 FIFO */
        nfc(0xE000u); nfc(0xF000u);
        NFI_STAT |= 0xF00u;
        NFI_START = 1u;
        (void)nf_wait();

 /* 读 FIFO：8 个 32 位 = 32 字节 */
        for (k = 0u; k < 8u; k++)
            dst[k] = *(volatile u32 *)(NFI_BASE + 0x19Cu);

        return 0;
    }

    return nf_one(cfg, psiz, lenhi, row, col, cmdset, startmode, mode);
}

/* ★ 2026-11-08（p80）：默认走自实现 USB 层；改回借用 FDL1 的旧路径见下面 #if 0 */

/* ---------- 协议辅助 ---------- */

static u32 be32(const unsigned char *p)
{
    return ((u32)p[0] << 24) | ((u32)p[1] << 16)
         | ((u32)p[2] << 8)  | (u32)p[3];
}

static void put_be32(unsigned char *p, u32 v)
{
    p[0] = (unsigned char)(v >> 24);
    p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);
    p[3] = (unsigned char)v;
}

/* add 校验：16 位小端字累加 → 进位回卷两次 → 取反 → 字节交换 */
static u32 add_cs(const unsigned char *p, u32 n)
{
    u32 s = 0u, i = n;
    while (i > 1u) {
        s += (u32)p[n - i] + ((u32)p[n - i + 1u] << 8);
        i -= 2u;
    }
    if (i == 1u)
        s += (u32)p[n - 1u];
    s = (s >> 16) + (s & 0xFFFFu);
    s += (s >> 16);
    s = (~s) & 0xFFFFu;
    return ((s >> 8) | (s << 8)) & 0xFFFFu;
}

/* ---------- 全局缓冲 ---------- */
/* ★ 尺寸说明：
 * p_rx 要能接住 BSL 的 MIDST_DATA 大块（厂商工具用 0x1000 = 4096）。
 * p_tx 要能装下 READ_FLASH 的响应；最坏情况每个字节都被 HDLC 转义翻倍，
 * 所以 1024 字节数据 → 帧 ~2060 字节。给到 4608 有余量。
 */
/* ★ 2026-11-08：扩到 16 KB（厂家工具 blk_size 上限 0x4000）
 *
 * 起因：spd_dump_uis8310 帮助里写着
 * "- Max Block Read Cap: 16KB (0x4000, 16x speedup)"
 * "[UIS8310] Warning: Capping blk_size from %d to 16KB (0x4000)"
 * ⇒ 它会发 16 KB 的 MIDST_DATA / 读请求。原来 4608 会把超长帧
 * 直接丢弃（if (n > BUF_RX_MAX - 8) continue;），表现为刷机中途
 * 挂死且只报"超时"，极难定位。
 *
 * 尺寸计算：
 * 16 KB 载荷 + 4（cmd/len）+ 2（校验）= 16390
 * p_tx 存转义后的帧，最坏情况每个字节都转义 ⇒ 2× 再加首尾 0x7E
 */
#define BUF_RX_MAX   16896u          /* 0x4200：16KB 帧 + 余量 */
#define BUF_BODY_MAX 16896u          /* 组包缓冲：4 + 16384 + 2 */
#define BUF_TX_MAX   33024u          /* 0x8100：转义最坏 2× + 余量 */
static unsigned char p_rx[BUF_RX_MAX];
static unsigned char p_tx[BUF_TX_MAX];
static unsigned char p_body[BUF_BODY_MAX];

/* 组帧 + 发送（cmd 已含响应码） */
/* ==================================================================
 * ★★★★★ 2026-11-08（p80）：自实现 USB 设备层
 *
 * 为什么要自己实现：
 * 原来 FDL2 借用 FDL1 留下的两个函数（0x8B50 / 0x8BDA），那是**绝对地址**，
 * 换任何一个别的 fdl1.bin 就会跳进别的代码。要做一个"通用 FDL2"，
 * 必须自己初始化 USB 控制器、自己收发 —— 该平台的下载器 就是这么做的
 * （它的二进制里同样有 0x20200000）。
 *
 * 寄存器序列来自开源的 spreadtrum_flash / t117_fdl/usbio.c（公有领域），
 * 与本机核对过的四点：
 * 控制基址 0x20200000 ✔ 端点区 0x20201be0+21*32 ✔ 端点 0x85/0x06 ✔ PID 1782:4D00 ✔
 * ================================================================== */
#define USE_OWN_USB 1


#define U8(a)  (*(volatile unsigned char  *)(a))
#define U16(a) (*(volatile unsigned short *)(a))
#define U32(a) (*(volatile unsigned int   *)(a))

#define USB_CTL   0x20200000u      /* 控制器基址 */
#define USB_EPB   0x20201BE0u      /* 端点描述符区 */
/* ★ p82：接收缓冲从 12800 提到 32 KB
 *
 * 开源/原厂用 12800，是因为它们只跑小帧；厂家工具按 16 KB 发 MIDST，
 * 16 384 + 6 字节头 = 16 390 > 12 800 ⇒ 这一笔传输被判溢出、整包丢弃，
 * 表现为"一个字节都不回、超时"。
 * 缓冲区是我们自己的，直接开到 32 KB，一次装下整个 16 KB 帧还有余量。 */
#define USB_RXBUF 32768u

typedef struct {
    void          *ptr;
    unsigned short len1, len2;
    unsigned int   flags;
} usb_tr_t;

static unsigned char usb_rxbuf[USB_RXBUF] __attribute__((aligned(64)));
static usb_tr_t usb_tx_tr __attribute__((aligned(16)));
static usb_tr_t usb_rx_tr __attribute__((aligned(16)));
static unsigned usb_rx_idx = 0u, usb_rx_len = 0u;
static unsigned char usb_64 = 0u;

/* 设备描述符：VID 0x1782 / PID 0x4D00（与原厂一致）*/
static const unsigned char usb_devdesc[18] __attribute__((aligned(4))) = {
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0x82, 0x17, 0x00, 0x4d, 0xff, 0xff, 0x00, 0x00,
    0x00, 0x01
};

/* 配置描述符：1 个接口，EP 0x85(IN) / 0x06(OUT)，各 512 字节包 */
static unsigned char usb_cfgdesc[32] __attribute__((aligned(4))) = {
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xc0, 0x32,
    0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0x00, 0x00, 0x00,
    0x07, 0x05, 0x85, 0x02, 0x00, 0x02, 0x00,
    0x07, 0x05, 0x06, 0x02, 0x00, 0x02, 0x00
};

static void usb_ep_config(void)
{
    u32 b = USB_CTL;
    u32 n = usb_64 ? 64u : 0x200u;

    U8(b + 0x0e) = 5u;  U16(b + 0x06) = 0x21u; U16(b + 0x08) = 0x40u;
    U8(b + 0x62) = 0x17u; U16(b + 0x64) = 0x108u;
    U16(b + 0x150) = (unsigned short)n; U8(b + 0x152) = 0x48u;
    U8(b + 0x153) = 0x20u; U8(b + 0x153) |= 0x94u;

    U8(b + 0x0e) = 6u;  U8(b + 0x63) = 0x17u; U16(b + 0x66) = 8u;
    U16(b + 0x164) = (unsigned short)n;
    U8(b + 0x166) = 0x90u; U8(b + 0x167) |= 0xa8u;
}

/* 武装 EP6（OUT）接收描述符 */
static void usb_start_recv(void)
{
    u32 b = USB_EPB + (15u + 6u) * 32u;
    U32(b + 8u) |= 0x14u;
    U32(b + 0x14u) = (u32)&usb_rx_tr;
    U32(b + 4u) |= 1u;
}

static void usb_send_desc(const unsigned char *buf)
{
    unsigned i, n, type;

    U8(0x20200012u) |= 0x40u;
    type = buf[3];
    n = buf[6];
    if (type == 1u) {
        if (n > sizeof(usb_devdesc)) n = sizeof(usb_devdesc);
        for (i = 0u; i < n; i++)
            U8(0x20200020u) = usb_devdesc[i];
    } else if (type == 2u) {
        if (usb_64) {
            usb_cfgdesc[9 + 9 + 4] = 64u;
            usb_cfgdesc[9 + 9 + 5] = 0u;
            usb_cfgdesc[9 + 9 + 7 + 4] = 64u;
            usb_cfgdesc[9 + 9 + 7 + 5] = 0u;
        }
        if (n > sizeof(usb_cfgdesc)) n = sizeof(usb_cfgdesc);
        for (i = 0u; i < n; i++)
            U8(0x20200020u) = usb_cfgdesc[i];
    } else {
        return;
    }
    U8(0x20200012u) |= 0x0au;
}

/* 处理标准请求（枚举必需）*/
static void usb_sys_events(void)
{
    unsigned i, n;
    unsigned char buf[8];

    if ((U8(0x20200012u) & 1u) == 0u)
        return;

    for (i = 0u; i < 8u; i += 2u) {
        buf[i]     = U8(0x20200020u);
        buf[i + 1] = U8(0x20200020u);
    }

    if ((buf[0] & 0x60u) != 0u) {
        U8(0x20200012u) |= 0x48u;
        while ((U16(0x20200002u) & 1u) == 0u)
            ;
        if ((buf[0] & 0x7fu) == 0x21u && buf[2] != 0u) {
            usb_ep_config();
            usb_start_recv();
        }
    } else if (buf[1] != 0u) {
        switch (buf[1]) {
        case 5u:
            U8(0x20200012u) |= 0x48u;
            while ((U16(0x20200002u) & 1u) == 0u)
                ;
            U8(0x20200000u) = buf[2];
            break;
        case 6u:
            usb_send_desc(buf);
            break;
        case 9u:
            U8(0x20200012u) |= 0x48u;
            while ((U16(0x20200002u) & 1u) == 0u)
                ;
            break;
        default:
            break;
        }
    } else {
        U8(0x20200012u) |= 0x40u;
        n = (unsigned)buf[6] | ((unsigned)buf[7] << 8);
        for (i = 0u; i < n; i++)
            U8(0x20200020u) = 0u;
        U8(0x20200012u) |= 0x0au;
    }
}

/* 轮询一次：处理枚举事件 + 收包完成 */
static unsigned usb_handler(void)
{
    u32 tmp;

    if ((U8(0x2020000au) & 4u) != 0u)
        usb_64 = ((U8(0x20200001u) & 0x10u) == 0u) ? 1u : 0u;

    tmp = U32(0x20201e88u);
    if ((U16(0x20200002u) & 1u) != 0u) {
        U8(0x2020000eu) = 0u;
        usb_sys_events();
    }
    if ((tmp & 0x40000u) == 0u)
        return 0u;
    {
        u32 b = USB_EPB + (15u + 6u) * 32u;
        U32(b + 8u) |= 0x14000000u;
        tmp = USB_RXBUF - (U32(0x20201e90u) >> 16);
        usb_rx_len = tmp;
        usb_rx_idx = 0u;
        cache_inv((u32)usb_rxbuf, tmp);
        U32(b + 8u) |= 0x14u;
        U32(b + 0x14u) = (u32)&usb_rx_tr;
        U32(b + 4u) |= 1u;
    }
    return 1u;
}

static unsigned char usb_getbyte(void)
{
    if (usb_rx_idx >= usb_rx_len)
        while (usb_handler() == 0u)
            ;
    return usb_rxbuf[usb_rx_idx++];
}

/* 发送一整帧（已含 0x7E 与转义）*/
static void usb_send_raw(const void *src, unsigned int len)
{
    u32 b = USB_EPB + 5u * 32u, a;
    usb_tr_t *tr = &usb_tx_tr;

    tr->ptr   = (void *)src;
    tr->len1  = 0x20;
    tr->len2  = (unsigned short)len;
    tr->flags = 5u;

    cache_clean((u32)src, len);
    cache_clean((u32)tr, 12u);

    U32(b + 8u) |= 4u;
    U32(b + 0x14u) = (u32)tr;
    U32(b + 4u) |= 1u;
    {
 /* ★ p83（A3）：原来是无界等待。TX 一旦不完成就永久停住 ——
 * 主机等 ACK、我们等 TX，双方互等，正是"写到一半突然停住"。 */
        u32 guard = 0u;
        while (((a = U32(b + 8u)) & (1u << 18)) == 0u) {
            if (++guard > 4000000u)
                break;
        }
    }
    U32(b + 8u) = a | (1u << 26);
}

static void usb_init(void)
{
    usb_rx_tr.ptr   = usb_rxbuf;
    usb_rx_tr.len1  = 0x20;
    usb_rx_tr.len2  = (unsigned short)USB_RXBUF;
    usb_rx_tr.flags = 5u;
    cache_clean((u32)&usb_rx_tr, 12u);

    usb_64 = 0u;
    usb_rx_idx = usb_rx_len = 0u;
    usb_start_recv();
}

/* 收一整帧：跳过 0x7E、反转义、校验反码和
 * 返回 = 去转义后的长度（含 2 字节校验和），与原来 FN_RECV 的口径一致 */
static unsigned usb_recv_frame(unsigned char *dst, unsigned max)
{
    unsigned n = 0u, esc = 0u, started = 0u;
    unsigned char c;

    for (;;) {
        c = usb_getbyte();
        if (esc != 0u) {
            esc = 0u;
            if (started != 0u) {
                if (n < max) dst[n++] = (unsigned char)(c ^ 0x20u);
                else { started = 0u; n = 0u; }
            }
            continue;
        }
        if (c == 0x7Eu) {
            if (started != 0u && n >= 6u) {
                u32 cs = add_cs(dst, n - 2u);
                if (((cs >> 8) & 0xFFu) == dst[n - 2u] &&
                    (cs & 0xFFu) == dst[n - 1u])
                    return n;
            }
            started = 1u; n = 0u;
            continue;
        }
        if (c == 0x7Du) { esc = 1u; continue; }
        if (started != 0u) {
            if (n < max) dst[n++] = c;
            else { started = 0u; n = 0u; }
        }
    }
}

static unsigned usb_recv_auto(void *dst)
{
    return usb_recv_frame((unsigned char *)dst, BUF_RX_MAX);
}


#define FN_SEND usb_send_raw
#define FN_RECV usb_recv_auto

static void send_resp(u32 cmd, const unsigned char *d, u32 len)
{
    u32 n = 0u, i, cs, t = 0u;

    p_body[n++] = (unsigned char)(cmd >> 8);
    p_body[n++] = (unsigned char)cmd;
    p_body[n++] = (unsigned char)(len >> 8);
    p_body[n++] = (unsigned char)len;
    for (i = 0u; i < len && n < (BUF_BODY_MAX - 8u); i++)
        p_body[n++] = d[i];

    cs = add_cs(p_body, n);
    p_body[n++] = (unsigned char)(cs >> 8);
    p_body[n++] = (unsigned char)cs;

    p_tx[t++] = 0x7Eu;
    for (i = 0u; i < n && t < (BUF_TX_MAX - 4u); i++) {
        if (p_body[i] == 0x7Eu) {
            p_tx[t++] = 0x7Du; p_tx[t++] = 0x5Eu;
        } else if (p_body[i] == 0x7Du) {
            p_tx[t++] = 0x7Du; p_tx[t++] = 0x5Du;
        } else {
            p_tx[t++] = p_body[i];
        }
    }
    p_tx[t++] = 0x7Eu;

    FN_SEND(p_tx, t);   /* 必须是 p_tx，否则上面的写入会被当死代码删掉 */
}



/* ----------------------------------------------------------------
 * ★ NAND 完整初始化（上电 → 时钟/复位 → PHY → NFI）
 *
 * P90 第一次测时漏了上电序列，直接调 phy_init/nfi_init，
 * 结果访问 NFI 寄存器挂总线，设备死机、0xF0 无响应。
 * 这里按控制器+ P85 的顺序补齐。
 * ---------------------------------------------------------------- */
static u32 g_nand_power = 0u;   /* 上电+时钟已完成 */
static u32 g_nand_phy   = 0u;   /* PHY+NFI 已完成 */
static u32 g_nfi_init_r = 0xEEu;/* nfi_init 的返回值（0=成功） */

/* 第 1 段：上电 + ADI + 时钟复位（0xF1 用，一次性做完） */
static void nand_power_on(void)
{
    if (g_nand_power)
        return;

 *(volatile u32 *)0x402E1000u = 0x10000u; /* ★ NAND 电源 */
 *(volatile u32 *)0x40600008u = 0u;
 *(volatile u32 *)0x4060000Cu = 0u;
 *(volatile u32 *)0x40600024u |= 0x80000000u;
 *(volatile u32 *)0x40600020u &= ~0x40000000u;
    nf_delay(40000u);

 *(volatile u32 *)0x21500060u = 3u;
 *(volatile u32 *)0x21500024u |= 2u;
 *(volatile u32 *)0x20E00000u |= 0x200u;
 *(volatile u32 *)0x20E00004u |= 0x80u;
    nf_delay(40000u);
 *(volatile u32 *)0x20E00004u &= ~0x80u;
    nf_delay(40000u);

    g_nand_power = 1u;
}

/* 第 2 段：PHY + NFI（这一步才会碰 0x21A00000） */
static void nand_phy_on(void)
{
    if (g_nand_phy)
        return;
    nand_power_on();
    phy_init();
    g_nfi_init_r = (u32)nfi_init();
    g_nand_phy = 1u;
}

/* ================================================================
 * ★★★ 分区层：BSL 写/读/擦 flash
 *
 * 分区 ID 用厂商工具那套（高位 0x8 = 数据区）：
 * 0x80000003 KernelImg 0x80000004 UserImg 0x80000008 PreloadUdisk
 *
 * ★★★ 块号是 2026-11-02 逐块扫描 dump 前缀实测出来的，不是厂商工具的槽位序号。
 * 旧表（块 2 / 60 / 131）抄的是工具里的"分区序号"，全错 —— 照它写会把
 * KernelImg 写到 BOOT1 上。真实布局（块 = 128KB = 64 页 × 2048）：
 *
 * 块 0~ 1 BOOT0 块 2 BOOT1
 * 块 21~ 31 GSMDSPCODE (11 块) 块 33~ 38 LTEDSPCODE (6 块)
 * 块 41 CM4CODE_AB 块 42~ 46 WCNCODE (5 块)
 * 块 48 DeltaNV 块 49~ 78 (空隙 30 块)
 * 块 79~134 KernelImg (56 块) ← 7340032 B / 131072 = 56.0
 * 块 135~203 UserImg (69 块) ← 8962048 B 向上取整 = 68.375 → 69
 * 块 204~209 MMIRes (6 块)
 * 块 210~239 PreloadUdisk(30 块) ← PRELOAD_UDISK_FULL 3870720 B → 30 块
 * 块 240~245 NV (6) 246 PhaseCheck 247 FotaUpdate 248~251 Erase FS
 * 块 252~255 _MAP / VBM_BOOT
 *
 * 校验：各分区字节数 / 131072 向上取整后逐段首尾相接，0~251 全被占满，
 * 无空洞无重叠 —— 自洽，故可信。PreloadUdisk 一段仍需上机复核首块。
 *
 * 三者都块对齐，所以可以直接按块擦、按页写。
 *
 * 写流程（START_DATA / MIDST_DATA / END_DATA）：
 * 数据先攒进 DBUF，满 2048 就写一页；写某块的第一页之前先擦该块。
 * 顺序写 + 逐块擦，不需要实现同样行为的 FTL/VBM。
 * ================================================================ */
#define PAGE_SZ        2048u
#define PAGES_PER_BLK  64u
#define BLK_SZ         (PAGE_SZ * PAGES_PER_BLK)   /* 128 KB */

#define PART_KERNEL    0x80000003u
#define PART_USER      0x80000004u
#define PART_PRELOAD   0x80000008u
/* ★ 任意单块伪分区：0x8000F000 | 块号（0~255）。
 * 走和真分区完全相同的擦/写/读路径，但块号由主机指定，用于安全自测。
 * 例：0x8000F032 = 块 50。
 * ★ 上一版把靶子写死成块 200，而块 200 其实落在 UserImg(135~203) 里面，
 * 结果把真实固件写坏了一块。现在改成运行时指定 + 主机侧写前强制校验全 FF。
 * 与真实分区 ID（0x8000000x）不在同一段，不会误认。 */
#define PART_BLK_BASE  0x8000F000u

typedef struct {
    u32 id;
    u32 blk0;
    u32 nblk;
} part_t;

/* ★★★ 分区表来源：NAND 块 254/255 的 VBM_BOOT（2026-11-03 解出并实测验证）
 * 头部 0x14 起是 10 字节定长记录：[u16 起块][u16 止块][u16 x][u16 y][u16 z]，
 * 14 条记录把块 10~247 无缝铺满，无空洞无重叠。三条交叉验证：
 * ① KernelImg 79..134（56 块）—— 逐块扫描 56 个分片全部命中
 * ② UserImg 135..203（69 块）—— 块 200/201 内容与镜像 chunk 65/66 全等
 * ③ MMIRes 204..209（ 6 块）—— 设备块 209 与 MMIRes.bin[0xA0000] 全等，
 * 块 208 首 64 字节落在 MMIRes.bin 偏移 0x80000 ⇒ 起始块 = 204
 * ⇒ 物理块 = 逻辑块（identity 映射），VBM 的 _MAP 块 252/253 除 12 字节头外
 * 全 0xFF，即没有任何坏块重映射记录。所以裸块直写是成立的。 */
static const part_t g_parts[] = {
    { PART_KERNEL,   79u, 56u },   /* 块 79 ~ 134 7,340,032 字节 */
    { PART_USER,    135u, 69u },   /* 块 135 ~ 203 8,962,048 字节 */
 /* ★ Preload 是 38 块不是 30 块：FAT12 卷按 BPB 是 4284 扇区 × 1024 =
 * 4,386,816 字节 = 33.47 块，写 30 块会把卷尾截掉。镜像文件
 * PRELOAD_UDISK_FULL.bin 只有 3,870,720 字节（29.53 块）是被截断的，
 * 不能拿它当分区尺寸。 */
    { PART_PRELOAD, 210u, 38u },   /* 块 210 ~ 247 */
};

/* ★★★★★ 2026-11-08（p75）：分区"名字式"载荷识别
 *
 * 厂家工具（spreadtrum_flash 血统）在 NAND 上不是发地址，而是发**分区名**：
 * struct pkt { uint16_t name[36]; uint32_t size, size_hi; }
 * encode_msg_nocpy(io, BSL_CMD_START_DATA / BSL_CMD_ERASE_FLASH, 76);
 * name 是 UTF-16LE 的 ASCII（"KernelImg" → 'K'0 'e'0 'r'0 ...）。
 *
 * 我们原来只认 8 字节的 addr+size，于是把 "K\0e\0" 当成地址 0x4B006500，
 * 判定"小于 0x80000000" ⇒ 走 RAM 加载 ⇒ 整个固件被写进非法内存并卡死，
 * flash 一个字节都没动。这就是"能开机但其实是老固件"的原因。
 */
static u32 nm_is(const unsigned char *nm, const char *s)
{
    u32 i;
    for (i = 0u; s[i] != 0; i++) {
        if (nm[i * 2u] != (unsigned char)s[i])
            return 0u;
        if (nm[i * 2u + 1u] != 0u)
            return 0u;
    }
    return (nm[i * 2u] == 0u) ? 1u : 0u;
}

/* 名字 → 分区表项；不认识返回 0 */
static const part_t *part_by_name(const unsigned char *nm)
{
    if (nm_is(nm, "KernelImg") != 0u)     return &g_parts[0];
    if (nm_is(nm, "UserImg") != 0u)       return &g_parts[1];
    if (nm_is(nm, "PreloadUdisk") != 0u)  return &g_parts[2];
    return 0;
}

/* PART_BLK_BASE 动态项的落点 */
static part_t g_dyn_part;

static const part_t *part_find(u32 id)
{
    u32 i;

    if ((id & 0xFFFFFF00u) == PART_BLK_BASE) {
        g_dyn_part.id   = id;
        g_dyn_part.blk0 = id & 0xFFu;
        g_dyn_part.nblk = 1u;
        return &g_dyn_part;
    }
    for (i = 0u; i < (u32)(sizeof(g_parts) / sizeof(g_parts[0])); i++)
        if (g_parts[i].id == id)
            return &g_parts[i];
    return 0;
}

/* 写 flash 的流式状态 */
static u32 g_wr_active  = 0u;          /* 1 = 正在写分区 */
static u32 g_wr_base    = 0u;          /* 本次流的起始行（块首） */
static u32 g_wr_n       = 0u;          /* 本次流已写的页数 */
static u32 g_wr_fill    = 0u;          /* DBUF 里已攒的字节数 */
static u32 g_wr_blk     = 0xFFFFFFFFu; /* 已经擦过的块号 */
static u32 g_wr_noerase = 0u;          /* ★ 1 = 本次流不擦块（修补重编程用） */

/* READ_FLASH 的应答数据缓冲（最大 1024，受 p_tx 帧长限制） */
static unsigned char g_rd[16384];   /* ★ p69：READ_FLASH 一次最多回 16KB */

static void nand_ensure(void)
{
    nand_power_on();
    nand_phy_on();
}

static int flash_erase_block(u32 blk)
{
    return nf_exec(0x8102u, 2048u, 0u, blk * PAGES_PER_BLK, 0u, 50u, 0u, 0u);
}

static int flash_write_page(u32 row)
{
    return nf_exec(0x8178u, 512u, 0u, row, 0u, 84u, 0u, 0u);
}

static int flash_read_page(u32 row)
{
    return nf_exec(0x78u, 512u, 0u, row, 0u, 80u, 0u, 0u);
}

/* ★ 本次流第 g_wr_n 页该写到哪一行。
 * 顺序写：行号 = 流起始行 + 已写页数。program disturb 的规避改由
 * wr_page_now()/flush_hold() 在固件内部处理（见下），这里不再做页序旋转。 */

/* ★★★ program disturb 规避：写满一块时，块内第 0/1 页必须最后写。
 *
 * 实测（2026-11-03）：按 0→63 顺序写满一块，错误 100% 集中在第 0、1 页，
 * 页 2~63 全对，重试 4 次都收敛不到零错，且错列固定（确定性错误）。
 * 只写 2 页时零错。⇒ 后续页编程把先写好的头两页干扰了。
 *
 * 做法：收到块内第 0/1 页时先扣在 RAM 里不写，等第 63 页写完再补写。
 * 这样厂商刷机工具走标准 BSL 流程（不会发 0xFC）也能自动规避。
 * 用 0xFC 设了页偏移、或 0xFD 设了不擦块（修补模式）时，暂存自动关闭。 */
static unsigned char g_hold[2][PAGE_SZ];
static u32 g_hold_has  = 0u;   /* bit0 = 页0 已暂存，bit1 = 页1 已暂存 */
static u32 g_hold_off  = 0u;   /* 1 = 关闭暂存（纯顺序写，诊断用） */

static int flash_prep_row(u32 row);      /* 定义在下面 */

/* 把暂存的头两页补写到 blk_base 块的页 0/1 */
static int flush_hold(u32 blk_base)
{
 /* ★ p87：**必须先写页 1、再写页 0**。
 * 实测（11-06 记录，今天 11-08 又撞到）：页 1 的编程会把页 0 扰乱，
 * 所以页 0 必须最后写。原来这里是 0→1，正好反了，导致页 0 主区出错。 */
    static const u32 order[2] = { 1u, 0u };
    u32 n, k, i;
    int rc = 0;
    for (n = 0u; n < 2u; n++) {
        k = order[n];
        if ((g_hold_has & (1u << k)) == 0u)
            continue;
        for (i = 0u; i < PAGE_SZ; i++)
 *(volatile unsigned char *)(DBUF + i) = g_hold[k][i];
        if (flash_prep_row(blk_base + k) != 0 ||
            flash_write_page(blk_base + k) != 0)
            rc = -1;
        g_hold_has &= ~(1u << k);
    }
    return rc;
}

/* 写当前这一页（DBUF 里已经是整页数据）。返回 0 成功。
 * 内部决定：扣下 / 立即写 / 写完第 63 页后补写头两页。 */
static int wr_page_now(void)
{

    u32 lin = g_wr_base + g_wr_n;
    u32 pg  = g_wr_n % PAGES_PER_BLK;
    int rc;

    if (g_hold_off == 0u && g_wr_noerase == 0u && pg < 2u) {
        u32 i;
        for (i = 0u; i < PAGE_SZ; i++)
            g_hold[pg][i] = *(volatile unsigned char *)(DBUF + i);
        g_hold_has |= (1u << pg);
        return 0;
    }

    rc = flash_prep_row(lin);
    if (rc != 0)
        return rc;
    rc = flash_write_page(lin);
    if (rc != 0)
        return rc;

 /* 本块最后一页刚写完 ⇒ 补写扣下的头两页 */
    if (pg == PAGES_PER_BLK - 1u && g_hold_has != 0u)
        return flush_hold(lin - (PAGES_PER_BLK - 1u));
    return 0;
}

/* 写某页之前，确保它所在的块已经擦过（顺序写，所以每块只擦一次） */
static int flash_prep_row(u32 row)
{
    u32 blk = row / PAGES_PER_BLK;
 /* ★ 修补模式：不擦块，直接在已写好的页上重编程。
 * NAND 允许同一页二次编程，只要只做 1→0。写同样的数据时，
 * 凡是第一次没写牢的位（0 读成 1）会被补上，1→0 是合法的；
 * 本来就已经是 0 的位不受影响。这是修 program disturb 的常规手段。 */
    if (g_wr_noerase)
        return 0;
    if (blk != g_wr_blk) {
        int r = flash_erase_block(blk);
        if (r != 0)
            return r;
        g_wr_blk = blk;
    }
    return 0;
}

/* ==================================================================
 * ★★★★★ 2026-11-08（p94）：建 MMU + 开 D/I-cache
 *
 * 原因：SCTLR = 0x00C52878，M/C/I 全 0 ⇒ 所有内存访问直通 DDR。
 * 实测（p93 A/B）：ecc_tab 229 KB 每页要读约 5.7 万次 u32，
 * 每次 ~460 ns（裸 DDR 延迟），ECC 占写页耗时 26.4 ms/页 = 75%。
 * 该平台的下载器 自己建 MMU，这就是它 3.6 s、我们 40 s 的主因。
 *
 * 映射：1 MB section 直通（虚拟 = 物理）
 * 0x80000000-0x803FFFFF Normal, TEX=001 C=1 B=1 (WBWA, cacheable)
 * 其余 Strongly-ordered（MMIO / SRAM，绝不缓存）
 *
 * ★ DMA 一致性（开 cache 后下面的维护才真正生效）：
 * DBUF/OBUF CPU写→NFI DMA读 ⇒ cache_clean（cmdset84 / ecc_fill_oob）
 * DBUF NFI DMA写→CPU读 ⇒ cache_inv2（nf_one）
 * p_tx/g_rd CPU写→USB DMA读 ⇒ cache_clean（usb_send_raw）
 * usb_rxbuf USB DMA写→CPU读 ⇒ cache_inv（usb_handler）
 * ================================================================== */
#define USE_MMU 1

static u32 mmu_tab[4096] __attribute__((aligned(16384)));

static void mmu_init(void)
{
    u32 i, v, z = 0u;

    for (i = 0u; i < 4096u; i++) {
        u32 pa = i << 20;
        mmu_tab[i] = (pa >= 0x80000000u && pa < 0x80400000u)
                     ? (pa | 0x00001C0Eu)    /* Normal, WBWA, cacheable */
                     : (pa | 0x00000C02u);   /* Strongly-ordered, **AP=11 全权限**
 * ★ p95 修正：原来写 0x2，bit[11:10]=00
 * 表示"任何权限都不可访问"，MMU 一开
 * 第一次访问 0x20200000 就权限异常挂死。 */
    }

    __asm__ volatile("mcr p15, 0, %0, c7, c5, 0" :: "r"(z) : "memory");  /* ICIALLU */
    __asm__ volatile("dsb" ::: "memory");

    __asm__ volatile("mcr p15, 0, %0, c2, c0, 0" :: "r"((u32)mmu_tab) : "memory");
    __asm__ volatile("mcr p15, 0, %0, c2, c0, 2" :: "r"(z) : "memory");
    __asm__ volatile("mcr p15, 0, %0, c3, c0, 0" :: "r"(0x55555555u) : "memory");
    __asm__ volatile("dsb" ::: "memory");
    __asm__ volatile("isb" ::: "memory");

    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(v));
    v &= ~(1u << 1);                              /* 关对齐检查，避免非对齐 abort */
    v |= (1u << 0) | (1u << 2) | (1u << 12);      /* M + D-cache + I-cache */
    __asm__ volatile("mcr p15, 0, %0, c1, c0, 0" :: "r"(v) : "memory");
    __asm__ volatile("isb" ::: "memory");
}

int main(void)
{
 /* ================================================================
 * P90：BSL 协议 + 两个自定义调试命令（USB 变成调试通道）
 *
 * 标准命令：
 * 0x00 CONNECT / 0x01 START_DATA / 0x02 MIDST_DATA
 * 0x03 END_DATA / 0x04 EXEC_DATA / 0x05 NORMAL_RESET
 *
 * ★ 自定义命令（本项目专用，主机侧自己发）：
 * 0xF0 读 NAND 关键寄存器，回传 8 个 u32（大端）：
 * [0]=NFI_ID [1]=CFG(+0x04) [2]=STAT(+0x10) [3]=出口(+0x19C)
 * [4]=+0x014 [5]=+0x018 [6]=+0x0F8 [7]=nand_ready
 * 0xF1 读一页：参数 row(4B BE) + cfg(4B BE)
 * 回传 40 字节：ret(4) + stat(4) + 数据前 32 字节
 * ★ 有了它，一次插拔可以试任意 cfg，不用再靠关机时刻猜
 * ================================================================ */

    u32 n, cmd, len, i;
    u32 load_addr = 0u, load_size = 0u, load_base = 0u;
    (void)load_size;

 /* ★★★★★ 2026-11-08：FDL2 一启动必须立刻回一个 ACK（生产工具兼容）
 *
 * 主机加载 FDL2 的流程是：
 * START_DATA → MIDST_DATA… → END_DATA → EXEC_DATA
 * 收到 EXEC_DATA 后 FDL1 **直接跳转、不回应答**，这个 ACK 必须由
 * FDL2 自己补发。控制器的第一句 MEMORY[0x80121444]()
 * 干的就是这件事（调用前连 "fdl2 started!" 都还没打印）。
 *
 * 不回会怎样：spreadtrum_flash 的 send_and_check() 里
 * ret = recv_msg(io); if (!ret) ERR_EXIT("timeout reached");
 * 只认 BSL_REP_ACK(0x80)，否则报 "FDL2 Exec timeout"。
 * ⇒ 厂家 spd_dump_uis8310.exe 刷不了机，就是这么卡住的。
 */
 /* ★★★★★ 2026-11-08（p71）：启动自预热
 *
 * 实测现象：加载后**第一个**数据操作可能丢一拍——踩在擦除上就"擦不掉"，
 * 踩在写上就整块 AND 成乱码，踩在 16KB 读上就是厂家工具
 * read_part KernelImg 得到 0 字节（而第二个、第三个分区都正常）。
 *
 * 对策：把这"第一次"消化在这里。做一次真实的页读（只读，安全），
 * 之后再回 LOG/ACK，主机拿到的第一个命令就不是第一个 NAND 操作了。
 */
 /* ★ 先把自己的 USB 设备层拉起来（不再依赖 FDL1 留下的任何东西）*/
 /* ★ p94：必须在任何 RAM/MMIO 访问之前把 MMU 立起来 */
    mmu_init();
    usb_init();

    nand_ensure();
    (void)flash_read_page(0u);

 /* ★★★★★ 2026-11-08（p88）：ECC 查表**在启动时展开**，不留在数据路径里。
 *
 * 为什么要挪：这套表 8193 行 × 106 个非零系数 × 7 字 ≈ 590 万次 32 位异或，
 * 而我们原来把它放在 `cmdset 84` 的第一句（ecc_fill_oob → ecc_build），
 * 也就是**主机等 ACK 的那段时间里**。
 *
 * 厂家工具的时间预算（源码 common.c:315）：每块 MIDST_DATA 只等 **1000 ms**，
 * 只有 `erase all` 才放宽到 100 s。而一帧 16 KB = 8 页，8 页编程 + 8 页 ECC
 * 本来就已占掉 ~170 ms，再叠一次表展开，第一条数据命令就超时。
 *
 * 挪到这里的代价：启动多花一次展开的时间；换来的是一条**干净的热路径**。
 * `ecc_fill_oob` 里仍然保留 ecc_build() 调用（它看到 ready 立刻返回），
 * 所以无论走哪条入口都不会漏。
 */
    {
        extern void ecc_build_pub(void);
        ecc_build_pub();
    }

    {
 /* ★★★★★ 2026-11-08（p76）：启动**只发一帧**。
 *
 * 原来发 LOG(0xFF) + 0x96 两帧，实测厂家工具在 exec 处只消费一帧，
 * 剩下的一帧就变成"下一个命令的应答"，导致整体错位一格：
 * START_DATA 收到 0x96、READ_FLASH 收到 0x80、下一个 START 收到 0x93…
 * LOG 是可选的（控制器会发，但工具不依赖），去掉它换来时序绝对干净。
 *
 * 仍回 0x96 + DA_INFO（控制器行为）：
 *
 * 为什么不是只回 ACK：spd_dump 的 exec 分支是
 * if (ret == BSL_REP_INCOMPATIBLE_PARTITION) get_Da_Info(io);
 * else if (ret != BSL_REP_ACK) ERR_EXIT(...)
 * 回 0x96 它会去解析 DA_INFO，从而知道 dwStorageType=0x101(NAND)，
 * 走 "Storage is nand" 分支，不会误判成 eMMC/GPT。
 *
 * 载荷格式（common.c: get_Da_Info 实测）：
 * "newt" + 若干 TLV，TLV = id(u16 LE) + len(u16 LE) + value
 * id 0 = bDisableHDLC (4B)
 * id 2 = bSupportRawData(1B)
 * id 3 = dwFlushSize (4B)
 * id 6 = dwStorageType (4B) 0x101 = NAND
 */
        {
 /* ★ 必须是静态常量数组：运行时逐字节赋值会被编译器合并/重排，
 * 协议字段一旦错位，主机解析 DA_INFO 就会得到垃圾。
 * 顺序 = "newt" + id0(4B) + id2(1B) + id3(4B) + id6(4B)，
 * id/len 用 u16 小端（主机按本机字节序读）。 */
            static const unsigned char da_info[] = {
                'n', 'e', 'w', 't',
                0x00, 0x00, 0x04, 0x00,  0x00, 0x00, 0x00, 0x00, /* id0 HDLC=0 */
                0x02, 0x00, 0x01, 0x00,  0x00,                   /* id2 rawdata=0 */
                0x03, 0x00, 0x04, 0x00,  0x00, 0x00, 0x00, 0x00, /* id3 flush=0 */
                0x06, 0x00, 0x04, 0x00,  0x01, 0x01, 0x00, 0x00  /* id6 =0x101 NAND */
            };
            send_resp(0x96u, da_info, (u32)sizeof(da_info));
        }
    }

    for (;;) {
        n = FN_RECV(p_rx);

 /* ★ 上限放宽到 p_rx 容量：厂商工具的 MIDST_DATA 用 0x1000(4096) 一块，
 * 旧上限 500 会把这种大块直接丢掉，导致写入中途卡死。 */
        if (n < 6u || n > (BUF_RX_MAX - 8u))
            continue;

        cmd = ((u32)p_rx[0] << 8) | (u32)p_rx[1];
        len = ((u32)p_rx[2] << 8) | (u32)p_rx[3];
        if (len > n - 6u) {
 /* ★ 2026-11-08 实测教训：这里回 VERIFY_ERROR(0x8B) 会让厂家工具
 * 打印 "unexpected response (0x008b)" 并打乱写入节奏。
 * 标准实现只在**校验失败**时回 VERIFY_ERROR；我们的传输层
 * （FDL1 的 FN_RECV）已经做过校验，这里属于误报。
 * 保持静默丢弃，配合协议日志定位即可。 */
            continue;
        }

 /* ★ 协议日志：开关打开时把本条命令回传成 LOG 包 */
        if (g_proto_log != 0u) {
            static const char hx[] = "0123456789ABCDEF";
            unsigned char m[24];
            u32 k = 0u;
            m[k++] = 'c'; m[k++] = 'm'; m[k++] = 'd'; m[k++] = '=';
            m[k++] = (unsigned char)hx[(cmd >> 12) & 0xFu];
            m[k++] = (unsigned char)hx[(cmd >> 8) & 0xFu];
            m[k++] = (unsigned char)hx[(cmd >> 4) & 0xFu];
            m[k++] = (unsigned char)hx[cmd & 0xFu];
            m[k++] = ' '; m[k++] = 'l'; m[k++] = 'e'; m[k++] = 'n'; m[k++] = '=';
            m[k++] = (unsigned char)hx[(len >> 12) & 0xFu];
            m[k++] = (unsigned char)hx[(len >> 8) & 0xFu];
            m[k++] = (unsigned char)hx[(len >> 4) & 0xFu];
            m[k++] = (unsigned char)hx[len & 0xFu];
            m[k++] = '\n';
            send_resp(0xFFu, m, k);
        }

 /* ★ 0xF0 必须写在 switch 外面：
 * 编译器生成的跳转表基址是 0xF1（sub r3,r3,#0xF1），
 * 0xF0 会算出 r3=-1 被无符号比较排除，直接走 default。
 * （实测：表内只有 0xF1~0xF5 五项） */
        if (cmd == 0xF0u) {
            nand_power_on();
            send_resp(0x80u, 0, 0u);
            continue;
        }

 /* ★ 0xFB SET_WR_MODE：设写页策略，参数 1 个 u32（大端）。
 * 和 0xF0 一样写在 switch 外面，避开跳转表基址的坑。
 * 0/1 = 4×512（延时 2ms / 20ms）
 * 2/3 = 整页一次 LOAD(2048)+EXECUTE（cfg 扇区数 0 / 3） */
        if (cmd == 0xE0u) {          /* PROTO_LOG 开关：1=开 0=关 */
            if (len >= 4u)
                g_proto_log = be32(p_rx + 4) ? 1u : 0u;
            send_resp(0x80u, 0, 0u);
            continue;
        }

        if (cmd == 0xFBu) {
            if (len >= 4u)
                g_wr_mode = be32(p_rx + 4);
            if (len >= 8u)
                g_b0_val = be32(p_rx + 8) & 0xFFu;
 /* ★ 第三个参数（可选）：NFI[0x008] 高字节 = 备用区字节数。
 * 0 = 老行为（不写 OOB）；16 / 64 = 让 ECC 校验值落进 OOB。
 * 只发 8 字节的老调用方不受影响。 */
            if (len >= 12u)
                g_wr_lenhi = be32(p_rx + 12) & 0xFFu;
 /* ★ 第四个参数（可选）：NFI[0x00C]（ECC 配置）的完整值。 */
            if (len >= 16u)
                g_ecc_val = be32(p_rx + 16);
 /* ★ 第五个参数（可选）：写 OOB 开关 */
            if (len >= 20u)
                g_oob_w = be32(p_rx + 20);
 /* ★ 第六个参数（可选）：SET FEATURE(0xA0, v) 的值 */
            if (len >= 24u)
                g_a0_val = be32(p_rx + 24) & 0xFFu;
            send_resp(0x80u, 0, 0u);
            continue;
        }

 /* ★ 0xFC SET_HOLD_OFF：0 = 正常（块内第 0/1 页扣下来最后写，默认），
 * 1 = 关闭暂存，纯顺序写（诊断用，会复现 program disturb）。
 * 暂存逻辑见 wr_page_now()。 */
        if (cmd == 0xFCu) {
            if (len >= 4u)
                g_hold_off = be32(p_rx + 4) ? 1u : 0u;
            send_resp(0x80u, 0, 0u);
            continue;
        }

 /* ★ 0xFD SET_NOERASE：1 = 后续写流不擦块（修补重编程）。
 * 默认 0；每次修补完记得发一次 0 复位，否则下一次真正写会漏擦。 */
        if (cmd == 0xFDu) {
            if (len >= 4u)
                g_wr_noerase = be32(p_rx + 4) ? 1u : 0u;
            send_resp(0x80u, 0, 0u);
            continue;
        }

        switch (cmd) {
        case 0xFEu: {                   /* ★★★ 读一页的 OOB（备用区 64 字节）
 * 请求：row (u32, BE)
 * 响应 72 字节：ret(4) + poll(4) + OOB(64)
 * 只读，不改任何 flash 内容。 */
            unsigned char out[72];
            u32 row, r, k2, ca;

            g_nand_phy = 0u;
            nand_phy_on();

            row = (len >= 4u) ? be32(p_rx + 4) : 0u;

 /* 先把 DBUF 前 64 字节标 0xAA：若回读仍是 AA ⇒ DMA 根本没写 */
            for (k2 = 0u; k2 < 64u; k2++)
 *(volatile unsigned char *)(DBUF + k2) = 0xAAu;
            ca = DBUF & ~0x3Fu;
            while (ca < (DBUF + 64u)) {
                __asm__ volatile("mcr p15, 0, %0, c7, c10, 1" :: "r"(ca) : "memory");
                ca += 64u;
            }
            __asm__ volatile("dsb sy" ::: "memory");

            r = (u32)nf_exec(0x78u, 64u, 0u, row, 0u, 85u, 0u, 0u);
            cache_inv(DBUF, 64u);

            put_be32(out + 0u, r);
            put_be32(out + 4u, g_poll_status);
            for (k2 = 0u; k2 < 64u; k2++)
                out[8u + k2] = *(volatile unsigned char *)(DBUF + k2);

            send_resp(0x80u, out, 72u);
            break;
        }

        case 0x00u:                     /* CONNECT */
            send_resp(0x80u, 0, 0u);
            break;

        case 0x01u:                     /* START_DATA：RAM 地址 或 分区 ID */
            if (len >= 72u) {
 /* ★ 名字式（76 字节）：name(72, UTF-16LE) + size(4, LE) */
                const part_t *pp = part_by_name(p_rx + 4);
                if (pp == 0) {
                    send_resp(0x96u, 0, 0u);   /* 只回一帧 */
                    break;
                }
                nand_ensure();
                g_wr_base  = pp->blk0 * PAGES_PER_BLK;
                g_wr_n     = 0u;
                g_wr_fill  = 0u;
                g_hold_has = 0u;
                g_wr_blk   = 0xFFFFFFFFu;
                g_wr_active = 1u;
                send_resp(0x80u, 0, 0u);
                break;
            }
            if (len >= 8u) {
                u32 a  = be32(p_rx + 4);
                u32 sz = be32(p_rx + 8);

                if (a >= 0x80000000u) {
 /* ★ 分区写入：只认我们支持的 3 个分区 */
                    const part_t *pp = part_find(a);
                    if (pp == 0) {
                        g_wr_active = 0u;
 /* ★ p74 诊断：改回 0x84（OPERATION_FAILED）。
 * 0x96 会和启动时的 DA_INFO 撞车，分不清是谁发的；
 * 0x84 只可能来自这里 ⇒ 看到 0x0084 就证明
 * START_DATA 真的到了命令循环、且地址没认出来。 */
                        send_resp(0x84u, 0, 0u);
                        break;
                    }
                    nand_ensure();
                    g_wr_base  = pp->blk0 * PAGES_PER_BLK;
                    g_wr_n     = 0u;
                    g_wr_fill  = 0u;
                    g_hold_has = 0u;               /* 新任务，暂存清空 */
                    g_wr_blk   = 0xFFFFFFFFu;      /* 新任务，块擦除标记复位 */
                    g_wr_active = 1u;
                    (void)sz;                      /* 长度靠 END_DATA 收尾 */
                } else {
                    load_base   = a;
                    load_addr   = a;
                    load_size   = sz;
                    g_wr_active = 0u;
                }
            }
            send_resp(0x80u, 0, 0u);
            break;

        case 0x02u:                     /* MIDST_DATA */
            if (g_wr_active) {
                u32 bad = 0u;
                for (i = 0u; i < len; i++) {
 *(volatile unsigned char *)(DBUF + g_wr_fill) = p_rx[4u + i];
                    g_wr_fill++;
                    if (g_wr_fill == PAGE_SZ) {
                        if (wr_page_now() != 0) {
                            bad = 1u;
                            break;
                        }
                        g_wr_n++;
                        g_wr_fill = 0u;
                    }
                }
                if (bad) {
                    g_wr_active = 0u;
                    send_resp(0x84u, 0, 0u);       /* OPERATION_FAILED */
                } else {
                    send_resp(0x80u, 0, 0u);
                }
            } else {
                for (i = 0u; i < len; i++)
 *(volatile unsigned char *)(load_addr + i) = p_rx[4u + i];
                load_addr += len;
                send_resp(0x80u, 0, 0u);
            }
            break;

        case 0x03u:                     /* END_DATA：把不足一页的尾巴补 0xFF 写掉 */
            if (g_wr_active) {
                u32 bad = 0u;
                if (g_wr_fill > 0u) {
                    while (g_wr_fill < PAGE_SZ) {
 *(volatile unsigned char *)(DBUF + g_wr_fill) = 0xFFu;
                        g_wr_fill++;
                    }
                    if (wr_page_now() != 0)
                        bad = 1u;
                    else
                        g_wr_n++;
                    g_wr_fill = 0u;
                }
 /* ★ 收尾：流结束时若还有扣下的头两页（例如只写了 2 页，
 * 或写到块中间就结束），必须补写，否则这两页是空的。 */
                if (!bad && g_hold_has != 0u) {
                    u32 lin = g_wr_base + g_wr_n;
                    u32 bb  = (lin / PAGES_PER_BLK) * PAGES_PER_BLK;
                    if (flush_hold(bb) != 0)
                        bad = 1u;
                }
                g_wr_active = 0u;
                send_resp(bad ? 0x84u : 0x80u, 0, 0u);
            } else {
                send_resp(0x80u, 0, 0u);
            }
            break;

        case 0x06u: {                   /* READ_FLASH：参数 addr, size, offset */
            u32 a   = be32(p_rx + 4);
            u32 sz  = (len >= 8u)  ? be32(p_rx + 8)  : 0u;
            u32 off = (len >= 12u) ? be32(p_rx + 12) : 0u;
            u32 base, pa, page, col, k, done;
            const part_t *pp;

            if (a >= 0x80000000u) {
                pp = part_find(a);
                if (pp == 0) { send_resp(0x96u, 0, 0u); break; }
                base = pp->blk0 * BLK_SZ;
            } else {
                base = a;
            }
 /* ★★★★★ 2026-11-08（p69）：原来这里死限 1024 字节，
 * 导致厂家工具 read_part 只拿到 1KB（备份文件就 1024 字节）。
 * p66 起帧缓冲已到 16KB，这里放开到 g_rd 容量，并支持跨页拼接：
 * 16KB = 连续 8 页，必须逐页读后拼起来，不能夹到页边界。 */
            if (sz > (u32)sizeof(g_rd)) sz = (u32)sizeof(g_rd);
            if (sz == 0u) { send_resp(0x84u, 0, 0u); break; }

            nand_ensure();
            done = 0u;
            while (done < sz) {
                u32 chunk;
                pa    = base + off + done;
                page  = pa / PAGE_SZ;
                col   = pa % PAGE_SZ;
                chunk = PAGE_SZ - col;
                if (chunk > (sz - done)) chunk = sz - done;
                if (flash_read_page(page) != 0) break;
                for (k = 0u; k < chunk; k++)
                    g_rd[done + k] = *(volatile unsigned char *)(DBUF + col + k);
                done += chunk;
            }
            if (done == 0u) { send_resp(0x84u, 0, 0u); break; }
            send_resp(0x93u, g_rd, done);      /* READ_FLASH */
            break;
        }

        case 0x0Au: {                   /* ERASE_FLASH：参数 addr, size, offset */
            if (len >= 72u) {
 /* ★ 名字式擦除：name(72, UTF-16LE)，size 忽略 */
                const part_t *pp = part_by_name(p_rx + 4);
                u32 b;
                int bad = 0;
                if (pp == 0) { send_resp(0x96u, 0, 0u); break; }
                nand_ensure();
                for (b = pp->blk0; b < pp->blk0 + pp->nblk; b++)
                    if (flash_erase_block(b) != 0) { bad = 1; break; }
                g_wr_blk = 0xFFFFFFFFu;
                send_resp(bad ? 0x84u : 0x80u, 0, 0u);
                break;
            }
            u32 a   = be32(p_rx + 4);
            u32 sz  = (len >= 8u)  ? be32(p_rx + 8)  : 0u;
            u32 off = (len >= 12u) ? be32(p_rx + 12) : 0u;
            u32 base, first, last, b, bad = 0u;
            const part_t *pp;

            if (a >= 0x80000000u) {
                pp = part_find(a);
                if (pp == 0) { send_resp(0x96u, 0, 0u); break; }
                base = pp->blk0 * BLK_SZ;
            } else {
                base = a;
            }
            if (sz == 0u) { send_resp(0x80u, 0, 0u); break; }
            first = (base + off) / BLK_SZ;
            last  = (base + off + sz - 1u) / BLK_SZ;
            nand_ensure();
            for (b = first; b <= last; b++)
                if (flash_erase_block(b) != 0) { bad = 1u; break; }
            g_wr_blk = 0xFFFFFFFFu;
            send_resp(bad ? 0x84u : 0x80u, 0, 0u);
            break;
        }

        case 0x04u:                     /* EXEC_DATA：跳 addr + 0x200（DHTB 头）*/
            send_resp(0x80u, 0, 0u);
            nf_delay(0x00100000u);
            if (load_base != 0u) {
 /* DHTB 头占 0x200，入口在 +0x200 */
                ((void (*)(void))(load_base + 0x200u))();
            }
 *(volatile u32 *)0x40608E14u &= ~0x8u;
            p0_poweroff();
            break;

        case 0x05u: {                   /* NORMAL_RESET = **关机**（本平台定案）
 *
 * ★ 2026-11-08（p102）：必须照 p0_关机 的写法，
 * 不能像 p101 那样"强制清 bit3 + 裸写 bit0"。
 *
 * p0_关机（已验证能让设备断电）的三步：
 * ① 轮询 0x40608E14 的 bit3，**等它自己清零**
 * （bit3 是 USB 挂载标志，强制清会把电源状态机搞乱）
 * ② 等 0x40600030 的 bit11（APB busy）清零
 * ③ 读 0x40608C20 置 bit0 写回
 *
 * p101 只做了"强制清 + 延时 0.04 s + 裸写"，实测不关机。
 */
            u32 n, k;

            send_resp(0x80u, 0, 0u);
            nf_delay(200000u);

            for (n = 0u; n < 2000000u; n++) {      /* ① 等 USB 标志自然清零 */
                if ((*(volatile u32 *)0x40608E14u & 0x8u) == 0u)
                    break;
            }

            k = 1000u;                             /* ② 等 APB 不忙 */
            while ((*(volatile u32 *)0x40600030u & (1u << 11)) && --k)
                ;

 *(volatile u32 *)0x40608C20u |= 1u; /* ③ PMIC bit0 = 关机 */
            for (;;)
                ;
        }

                case 0xE1u:
            send_resp(0x80u, 0, 0u);
            nf_delay(200000u);
 *(volatile u32 *)0xE000ED0Cu = 0x05FA0004u; /* AIRCR = SYSRESETREQ */
            for (;;)
                ;
        case 0xE2u:
            send_resp(0x80u, 0, 0u);
            nf_delay(200000u);
 *(volatile u32 *)0x40608C20u |= 2u; /* PMIC reset */
            for (;;)
                ;

 /* ================= ★ 自定义调试命令 ================= */

 /* ★★★★★ 0xE3 = 写任意 NFI 寄存器（最多 8 组）
 *
 * payload: n × { offset(u16 BE), value(u32 BE) } n ≤ 8
 * 返回 : 8 × u32（写入后逐个回读，未写的填 0）
 *
 * 用途：上机后不重编译就能试 0x60-0x6C / 0x38 / 0x21C 各种组合。
 * 安全护栏：offset 必须 < 0x300，否则跳过（防手误写到别的外设）。
 */
        case 0xE3u: {
            unsigned char out[32];
            u32 n3 = len / 6u;
            u32 k;

            if (n3 > 8u)
                n3 = 8u;
            nand_phy_on();
            for (k = 0u; k < 8u; k++) {
                u32 v = 0u;
                if (k < n3) {
                    u32 off = ((u32)p_rx[4u + 6u * k] << 8) |
                              (u32)p_rx[5u + 6u * k];
                    u32 val = ((u32)p_rx[6u + 6u * k] << 24) |
                              ((u32)p_rx[7u + 6u * k] << 16) |
                              ((u32)p_rx[8u + 6u * k] << 8) |
                              (u32)p_rx[9u + 6u * k];
                    if (off < 0x300u) {
 *(volatile u32 *)(NFI_BASE + off) = val;
                        v = *(volatile u32 *)(NFI_BASE + off);
                    }
                }
                put_be32(out + 4u * k, v);
            }
            send_resp(0x80u, out, 32u);
            break;
        }

 /* ★★★★★ 0xE4 = ECC 配置预设（来自 boot0 阶段）
 *
 * payload: mode(u8) [+ 0x21C 值(u32 BE)，mode==5 时用]
 *
 * mode 0 = boot0 全套：0x60-0x6C + 0xB0-0xBC + 0x38|=2
 * mode 1 = 只写 0x60-0x6C（不动 FDL2 原有的 0xB0-0xBC） ← 首选
 * mode 2 = 只置 0x38 bit1
 * mode 3 = 只把 0xB0-0xBC 改成 boot0 的值
 * mode 4 = 0x60-0x6C + 0x38|=2
 * mode 5 = 全套 + 0x21C = 给定值
 *
 * boot0 阶段使用的值：
 * 0x60/0xB0 = 0x0000100D
 * 0x64/0xB4 = 0x00010004
 * 0x68/0xB8 = 0x00040013
 * 0x6C/0xBC = 0x00400010 (= ror(0x00010004, 12))
 * 0x38 |= 2
 * 控制器FDL2 的 NAND 初始化 只写 0xB0-0xBC = 0x1004/0x1004/0x4013，
 * 0x60-0x6C 与 0x21C 一次都没碰 —— 这正是差异所在。
 *
 * 返回: 10 × u32 = 0x60,0x64,0x68,0x6C,0x38,0xB0,0xB4,0xB8,0xBC,0x21C
 */
        case 0xE4u: {
            unsigned char out[40];
            u32 mode = (len > 0u) ? (u32)p_rx[4] : 0u;
            u32 extra = 0u;

            if (len >= 5u)
                extra = ((u32)p_rx[5] << 24) | ((u32)p_rx[6] << 16) |
                        ((u32)p_rx[7] << 8) | (u32)p_rx[8];
            nand_phy_on();

            if (mode == 0u || mode == 1u || mode == 4u || mode == 5u) {
 *(volatile u32 *)(NFI_BASE + 0x060u) = 0x0000100Du;
 *(volatile u32 *)(NFI_BASE + 0x064u) = 0x00010004u;
 *(volatile u32 *)(NFI_BASE + 0x068u) = 0x00040013u;
 *(volatile u32 *)(NFI_BASE + 0x06Cu) = 0x00400010u;
            }
            if (mode == 0u || mode == 3u || mode == 5u) {
 *(volatile u32 *)(NFI_BASE + 0x0B0u) = 0x0000100Du;
 *(volatile u32 *)(NFI_BASE + 0x0B4u) = 0x00010004u;
 *(volatile u32 *)(NFI_BASE + 0x0B8u) = 0x00040013u;
 *(volatile u32 *)(NFI_BASE + 0x0BCu) = 0x00400010u;
            }
            if (mode == 0u || mode == 2u || mode == 4u || mode == 5u) {
                u32 v38 = *(volatile u32 *)(NFI_BASE + 0x038u);
 *(volatile u32 *)(NFI_BASE + 0x038u) = v38 | 2u;
            }
            if (mode == 5u)
 *(volatile u32 *)(NFI_BASE + 0x21Cu) = extra;

            put_be32(out +  0u, *(volatile u32 *)(NFI_BASE + 0x060u));
            put_be32(out +  4u, *(volatile u32 *)(NFI_BASE + 0x064u));
            put_be32(out +  8u, *(volatile u32 *)(NFI_BASE + 0x068u));
            put_be32(out + 12u, *(volatile u32 *)(NFI_BASE + 0x06Cu));
            put_be32(out + 16u, *(volatile u32 *)(NFI_BASE + 0x038u));
            put_be32(out + 20u, *(volatile u32 *)(NFI_BASE + 0x0B0u));
            put_be32(out + 24u, *(volatile u32 *)(NFI_BASE + 0x0B4u));
            put_be32(out + 28u, *(volatile u32 *)(NFI_BASE + 0x0B8u));
            put_be32(out + 32u, *(volatile u32 *)(NFI_BASE + 0x0BCu));
            put_be32(out + 36u, *(volatile u32 *)(NFI_BASE + 0x21Cu));
            send_resp(0x80u, out, 40u);
            break;
        }

 /* ★★★★★ 0xEB = 调用【控制器FDL2】的任意函数，并读回一段内存
 *
 * 用途：该平台的下载器 已由 FDL1 加载到 0x80100000（只传不跳，见
 * test/probe_bml.py 的 load_only），此时自研 FDL2 在 0x80200000
 * 运行，可以直接调用控制器的初始化函数，让它自己把 BML 表
 * （0x802C08F8）建起来，然后把表读回来。
 *
 * payload: fn(4B BE) + arg(4B BE) + raddr(4B BE) + [rn(4B BE)，默认 32]
 * 返回 : ret(4B BE) + 读到的 rn 字节
 *
 * ⚠️ fn 必须确认过。地址错会直接挂死，只能断电重来。
 */
 /* ★★★ 0xE9 = 把一段内存清零（用于抹平控制器FDL2 的 BSS）
 *
 * 该平台的下载器 镜像只有 66KB（0x80100000~0x801103D0），但它的
 * BSS（dev 表 0x80173E00、DBUF 0x80174000、OBUF 0x80176000 等）
 * 在镜像之外，靠启动代码 memset。我们只传镜像不执行，
 * 所以那块全是上电垃圾 ⇒ 调控制器初始化必然挂死。
 *
 * payload: addr(4B BE) + len(4B BE)，上限 1 MB
 */
                case 0xF5u: {                   /* 只读两个**确定安全**的地址 */
            unsigned char out[8];

            put_be32(out + 0u, *(volatile u32 *)0x402E1000u);
            put_be32(out + 4u, *(volatile u32 *)0x40600024u);

            send_resp(0x80u, out, 8u);
            break;
        }

        case 0xF3u: {                   /* 第 2 步：ADI 解锁 */
            unsigned char out[8];

 *(volatile u32 *)0x40600008u = 0u;
 *(volatile u32 *)0x4060000Cu = 0u;
 *(volatile u32 *)0x40600024u |= 0x80000000u;
 *(volatile u32 *)0x40600020u &= ~0x40000000u;
            nf_delay(40000u);

            put_be32(out + 0u, *(volatile u32 *)0x40600024u);
            put_be32(out + 4u, *(volatile u32 *)0x40600020u);
            send_resp(0x80u, out, 8u);
            break;
        }

        case 0xF4u: {                   /* 第 3 步：时钟 + 复位 */
            unsigned char out[8];

 *(volatile u32 *)0x21500060u = 3u;
 *(volatile u32 *)0x21500024u |= 2u;
 *(volatile u32 *)0x20E00000u |= 0x200u;
 *(volatile u32 *)0x20E00004u |= 0x80u;
            nf_delay(40000u);
 *(volatile u32 *)0x20E00004u &= ~0x80u;
            nf_delay(40000u);

            put_be32(out + 0u, *(volatile u32 *)0x21500060u);
            put_be32(out + 4u, *(volatile u32 *)0x20E00004u);
            send_resp(0x80u, out, 8u);
            break;
        }

        case 0xF2u: {                   /* 阶段2：PHY+NFI，回读 NFI 寄存器 */
            unsigned char out[32];

            nand_phy_on();

            put_be32(out + 0u,  *(volatile u32 *)(NFI_BASE + 0x19Cu));
            put_be32(out + 4u,  *(volatile u32 *)(NFI_BASE + 0x004u));
            put_be32(out + 8u,  *(volatile u32 *)(NFI_BASE + 0x010u));
            put_be32(out + 12u, *(volatile u32 *)(NFI_BASE + 0x19Cu));
            put_be32(out + 16u, *(volatile u32 *)(NFI_BASE + 0x014u));
            put_be32(out + 20u, *(volatile u32 *)(NFI_BASE + 0x018u));
            put_be32(out + 24u, *(volatile u32 *)(NFI_BASE + 0x0F8u));
            put_be32(out + 28u, g_nfi_init_r);   /* nfi_init 返回值，0=成功 */

            send_resp(0x80u, out, 32u);
            break;
        }

        case 0xF1u: {                   /* 通用 NFI 执行器：8 个参数 */
            unsigned char out[64];
            u32 row, cfg, psiz, lenhi, col, mode, cmdset, smode, r;

 /* ★ 每个测试前强制重新初始化。
 * 一次失败会把控制器留在坏状态，之后所有命令都失败
 * （实测：第 3 组 cfg 写错后，连原本 OK 的配置也全挂）。*/
            g_nand_phy = 0u;
            nand_phy_on();

            row    = be32(p_rx + 4);
            cfg    = be32(p_rx + 8);
            psiz   = (len >= 12u) ? be32(p_rx + 12) : 0u;
            lenhi  = (len >= 16u) ? be32(p_rx + 16) : 0u;
            col    = (len >= 20u) ? be32(p_rx + 20) : 0u;
            mode   = (len >= 24u) ? be32(p_rx + 24) : 0u;
            cmdset = (len >= 28u) ? be32(p_rx + 28) : 0u;
            smode  = (len >= 32u) ? be32(p_rx + 32) : 0u;

 /* ★ 先把缓冲区前 64 字节标记成 0xAA 并刷进内存，
 * 这样回传的数据若仍是 0xAA，就说明 DMA 完全没写。
 *
 * ★★★ 但**写页**时这一步是致命的：DBUF 里装的就是要写进
 * flash 的页数据，标成 0xAA 等于把前 64 字节改掉。
 * 2026-11-03 实测：V0 生产路径写完后主区正好错 64 字节，
 * 头 200 字节读回是 `AA AA AA ...`。
 * ⇒ 写事务必须带 F_NOMARK。 */
            if ((mode & F_NOMARK) == 0u) {
                for (i = 0u; i < 64u; i++)
 *(volatile unsigned char *)(DBUF + i) = 0xAAu;
                {
                    u32 ca = DBUF & ~0x3Fu;
                    while (ca < (DBUF + 64u)) {
                        __asm__ volatile("mcr p15, 0, %0, c7, c10, 1"
                                         :: "r"(ca) : "memory");
                        ca += 64u;
                    }
                    __asm__ volatile("dsb sy" ::: "memory");
                }
            }

            r = (u32)nf_exec(cfg, psiz, lenhi, row, col, cmdset, smode, mode);

            cache_inv(DBUF, 2048u);

            put_be32(out + 0u,  r);
            put_be32(out + 4u,  g_stat_after);
            put_be32(out + 8u,  g_stat_before);          /* ★ 启动前的状态 */
            put_be32(out + 12u, NFI_START);
            put_be32(out + 16u, *(volatile u32 *)(NFI_BASE + 0x19Cu));
            put_be32(out + 20u, g_nfi_init_r);
            for (i = 0u; i < 32u; i++)
                out[24u + i] = *(volatile unsigned char *)(DBUF + i);
 /* ★ 数据有没有落到别的缓冲区；最后一格放状态轮询结果 */
            put_be32(out + 56u, *(volatile u32 *)0x80174000u);
            put_be32(out + 60u, g_poll_status);

            send_resp(0x80u, out, 64u);
            break;
        }

 case 0xF6u: { /* NFI 寄存器全量回读（对照控制器基线）*/
            unsigned char out[76];
            static const u16 offs[17] = {
                0x000, 0x004, 0x008, 0x00C, 0x010, 0x014, 0x018,
                0x034, 0x038, 0x0EC, 0x0F8, 0x190, 0x19C,
                0x200, 0x204, 0x210, 0x214
            };

            nand_phy_on();

            for (i = 0u; i < 17u; i++)
                put_be32(out + 4u * i,
 *(volatile u32 *)(NFI_BASE + (u32)offs[i]));
            put_be32(out + 68u, g_nfi_init_r);

            send_resp(0x80u, out, 72u);
            break;
        }

 /* ★ 0xEF 读任意 NFI 寄存器：参数 = 最多 8 个 u16 偏移（BE）。
 * 0xF6 的固定表里没有 ECC 相关的那几个（0x0DC/0x0E0/0x0E4/0x0E8），
 * 而控制器每个写页事务都会设 NFI[0x00C]（ECC 配置），这个必须能读。
 * （0xF3/0xF4/0xF5 已被 ADI 解锁那组占掉，所以用 0xEF。） */
        case 0xEFu: {
            unsigned char out[32];
            u32 n3 = (len / 2u > 8u) ? 8u : (len / 2u);
            u32 k;

            nand_phy_on();
            for (k = 0u; k < 8u; k++) {
                u32 v = 0u;
                if (k < n3)
                    v = *(volatile u32 *)(NFI_BASE +
                                          (u32)(((u32)p_rx[4u + 2u * k] << 8) |
                                                (u32)p_rx[5u + 2u * k]));
                put_be32(out + 4u * k, v);
            }
            send_resp(0x80u, out, 32u);
            break;
        }

 /* ★★★★★ 0xEA = **直接调控制器写页封装 **（不跑控制器初始化）
 *
 * 为什么这条路可行：`` 只依赖三样东西
 * ① 手工填好的 dev 结构（0x80173E00）
 * ② NFI 寄存器（我们的 FDL2 已经初始化好了）
 * ③ 主区 / 备用区缓冲区
 * 它内部会自己走：SET FEATURE(0xA0,0x38) → SET FEATURE(0xB0,9)
 * → PAGE READ → WREN → PROGRAM LOAD 主区 → PROGRAM LOAD 备用区 → EXECUTE
 *
 * **这是唯一能让我们看到「控制器怎么处理 OOB」的办法** ——
 * 不猜算法，让它自己跑一遍，然后读回 OOB 看结果。
 *
 * 参数：page_idx(4B) + 主区buf(4B) + OOB buf(4B，0 = 让控制器memset 0xFF)
 */
 /* ★★★ 0xEC = 撤销 PMIC 寄存器 0x40608C20 的 bit1（**补救命令**）
 *
 * 2026-11-04 事故：我加的 0xE2 命令写了 `0x40608C20 |= 2`，
 * 但文档只确认过 bit0 = 复位，**bit1 是什么根本不知道**。
 * 写完之后设备表现为「USB 供电能跑 BootROM，但电池/充电全断
 * （插充电线灯都不亮）」⇒ 极可能 bit1 就是电池/充电通路的控制位。
 *
 * 这条命令把它清掉，并把前后的值都回传，便于确认。
 */
        case 0xECu: {
            unsigned char out[8];
            u32 before, after;

            before = *(volatile u32 *)0x40608C20u;
 *(volatile u32 *)0x40608C20u = before & ~2u;
            after = *(volatile u32 *)0x40608C20u;

            put_be32(out + 0u, before);
            put_be32(out + 4u, after);
            send_resp(0x80u, out, 8u);
            break;
        }

 /* ★ 0xED 读 CPU 状态：SCTLR / TTBR0 / CPSR。
 * 用来确认 MMU 和 D-cache 到底开没开 —— 如果 MMU 开着，
 * DMA 用的物理地址可能和 CPU 看到的虚拟地址不是一回事。 */
        case 0xEEu: {                   /* ★ 设 OOB 写入字节数（1~128），回读确认 */
            unsigned char out[8];
            u32 n = 0u, i;

            if (len < 4u) {
                send_resp(0x84u, 0, 0u);
                break;
            }
            for (i = 0u; i < 4u; i++)
                n = (n << 8) | (u32)p_rx[4u + i];
            if (n == 0u || n > 128u)
                n = 64u;
            g_oob_wlen = n;
            put_be32(out + 0u, g_oob_wlen);
            put_be32(out + 4u, 0x3000u | ((n - 1u) & 0xFFu) | (((n - 1u) & 0xFFu) << 16));
            send_resp(0x80u, out, 8u);
            break;
        }

        case 0xEDu: {
            unsigned char out[12];
            u32 sctlr, ttbr0, cpsr;

            __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));
            __asm__ volatile("mrc p15, 0, %0, c2, c0, 0" : "=r"(ttbr0));
            __asm__ volatile("mrs %0, cpsr" : "=r"(cpsr));

            put_be32(out + 0u, sctlr);
            put_be32(out + 4u, ttbr0);
            put_be32(out + 8u, cpsr);
            send_resp(0x80u, out, 12u);
            break;
        }

        case 0xF7u: {                   /* 只发芯片复位序列（0xFF ...）*/
            unsigned char out[8];
            u32 r;

            nand_phy_on();

            FIFO_IDX = 0u;
            nfc(0xFFu); nfc(0xE000u); nfc(0x0Fu); nfc(0xC0u);
            nfc(0x1000u); nfc(0xE001u); nfc(0xF000u);
            NFI_STAT |= 0xF00u;
            NFI_START = 1u;
            r = (u32)nf_wait();

            put_be32(out + 0u, r);
            put_be32(out + 4u, NFI_STAT);
            send_resp(0x80u, out, 8u);
            break;
        }

 /* ★★★ 诊断命令：读回【控制器】初始化好的 dev 结构
 * 流程：先跑该平台的下载器（它填好 dev），再把本镜像加载到
 * 0x80200000 执行，此时 0x80173E00 原封不动。
 * dev 步长 104，两个 CE → 208 字节。
 * ★ 本命令不碰 NFI，不做任何初始化，纯读内存。 */
 /* ★ 诊断命令：读任意内存。参数 addr(4B BE) + len(4B BE)，len 上限 200 */
        case 0xF9u: {
            unsigned char out[200];
            u32 a = be32(p_rx + 4);
            u32 n2 = (len >= 8u) ? be32(p_rx + 8) : 0u;
            u32 k;

            if (n2 > 200u)
                n2 = 200u;
            for (k = 0u; k < n2; k++)
                out[k] = *(volatile unsigned char *)(a + k);

            send_resp(0x80u, out, n2);
            break;
        }

 /* ★★★ 诊断命令 0xFA：让【控制器】的几何填充函数替我们解码表项
 *
 * 前置条件：该平台的下载器 的代码已被加载到 0x80100000（只需加载，不必执行）。
 * 然后调用：
 * (dev, spec, _, ce) —— 控制器「从表项填 dev」的函数
 * dev = 0x80173E00
 * spec = = + 64*56 （ID 0xFDB0 那一项）
 *
 * 这样 dev 里的页大小/OOB/每块页数/各种移位，全部由**控制器代码**算出来，
 * 完全绕开我们自己猜偏移可能出错的问题。
 * 回传：返回值(4B) + dev 前 104 字节。 */
 /* ★★★ 0xFF WRITE_MEM：往任意 RAM 地址写数据。
 *
 * 为什么需要它：0x01 START_DATA 把 `addr >= 0x80000000` 一律当分区 ID
 * 处理，而该平台的下载器 的 imagebase 正好是 0x80100000 —— 落在那个
 * 区间里，永远走不到「写 RAM」分支。
 * 要用 p43b（基址 0x80200000）把控制器当**数据**加载到 0x80100000，
 * 再调它的 填 dev，就必须有这条命令。
 *
 * 参数：addr(4B BE) + 数据(len-4 字节)。单帧上限约 4KB。 */
        case 0xFFu: {
            u32 a, k;
            if (len >= 4u) {
                a = be32(p_rx + 4);
 /* ★★★ dcache_off 必须放在**拷贝之后**！
 *
 * dcache_off 里做的是「按 set/way clean+invalidate 全片
 * cache」。如果放在拷贝之前调用，它会把 p_rx（收包缓冲区，
 * 此时数据还在 cache 脏行里）一起清掉 —— 拷进目标地址的
 * 就全是垃圾。
 *
 * 2026-11-03 实测症状：4 个扇区里只有 col=0 那个全 FF，
 * 后 3 个完全正确。因为 2048 字节一次写完时，p_rx 前 512
 * 字节被清掉、后面部分恰好已经落盘。
 *
 * 先拷贝（此时 cache 还开着，p_rx 数据有效），
 * 再 dcache_off 把刚写的脏行 clean 回物理内存。
 */
                for (k = 4u; k < len; k++)
 *(volatile unsigned char *)(a + (k - 4u)) = p_rx[4u + k];
 /* ★ 2026-11-03 23:10 回退：这里不加任何 cache 操作。
 * 22:00 能工作的版本没有它。今天加的 cache 刷回全部无效，
 * 先退回原状，避免引入新的变量。 */
 /* ★★★ 写完立刻 clean 回物理内存。
 * 主机随后会用 NFI 的 DMA 从这里取数据写 flash；如果数据还
 * 停在 CPU cache 的脏行里，DMA 读到的是旧内容（全 0），
 * 写进 flash 的自然就是 0 —— 2026-11-03 实测就是这个症状：
 * 主区读回「00 00 00…」，1473/2048 不同。
 * 在 nf_one 里 clean 不够稳，这里必须做一次。 */
                cache_clean(a & ~0x3Fu, ((len - 4u) + 64u) & ~0x3Fu);
            }
            send_resp(0x80u, 0, 0u);
            break;
        }

        default:                        /* 未知命令 */
            send_resp(0x83u, 0, 0u);
            break;
        }
    }
}


