/*
 * * 目标设备：蜂助手 MIBOX-660XT（展锐 UIS8310，32 MiB NAND）
 * 运行模式：ARM 启动 → Thumb 主体
 */

#ifndef FDL2_H
#define FDL2_H

typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;

#define NULL ((void *)0)

/* ---------------------------------------------------------------- 版本 */
#define FDL2_NAME    "selfdl2"
#define FDL2_VERSION "0.1"

/* ---------------------------------------------------------------- 协议 */
#define HDLC_FLAG    0x7E
#define HDLC_ESC     0x7D
#define HDLC_ESC_XOR 0x20

/* 命令码（由实测确认） */
#define CMD_START_DATA    0x01
#define CMD_MIDST_DATA    0x02
#define CMD_END_DATA      0x03
#define CMD_NORMAL_RESET  0x05
#define CMD_READ_FLASH    0x06
#define CMD_READ_CHIP     0x07
#define CMD_ERASE_FLASH   0x0A
#define CMD_关机     0x17
#define CMD_NAND_BLOCK    0x29

/* 响应码 */
#define REP_ACK           0x80
#define REP_OPERATION_FAILED 0x84
#define REP_VERIFY_ERROR  0x8B
#define REP_READ_FLASH    0x93
#define REP_LEN_ERROR     0x8A

/* 见 docs/ */
#define READ_CHUNK_MAX    0x4000

/* ---------------------------------------------------------------- NAND */
#define NAND_BASE         0x40600000u
#define NAND_PAGE_SIZE    2048
#define NAND_PAGES_BLK    64
#define NAND_BLK_BYTES    (NAND_PAGE_SIZE * NAND_PAGES_BLK)   /* 128 KiB */
#define NAND_TOTAL_BLKS   256

/* 传输缓冲：一块 + 一页余量 */
#define XFER_BUF_SIZE     (NAND_BLK_BYTES + NAND_PAGE_SIZE)

/* ---------------------------------------------------------------- 帧缓冲 */
#define FRAME_MAX         0x4400          /* 4 KiB 数据 + 头 + 校验 + 转义余量 */
#define RX_BUF_SIZE       (FRAME_MAX * 2)
#define TX_BUF_SIZE       (FRAME_MAX * 2)

/* ---------------------------------------------------------------- 全局 */
extern u8 g_rx[RX_BUF_SIZE];
extern u8 g_tx[TX_BUF_SIZE];
extern u8 g_xfer[XFER_BUF_SIZE];

/* 见 docs/ */
struct write_session {
    u32 target_blk;      /* 目标起始块 */
    u32 expect;          /* START_DATA 声明的总长度 */
    u32 written;         /* 已写入字节数 */
    u8  active;
    u8  error;
};
extern struct write_session g_ws;

/* ---------------------------------------------------------------- 各模块 */
/* hdlc.c */
u16  crc16_ccitt(const u8 *p, u32 n);
u16  chk_add(const u8 *p, u32 n);
u32  hdlc_encode(u8 *out, u32 out_max, u8 cmd, const u8 *data, u32 len);
int  hdlc_decode(const u8 *in, u32 n, u8 *cmd, u8 **data, u32 *len);

/* part.c */
int  part_lookup(u32 part_id, u32 *start_blk, u32 *blocks);
int  blk_allowed(u32 blk, u32 count);

/* nand.c */
int  nand_init(void);
int  nand_read(u32 blk, u32 off, u32 len, u8 *out);
int  nand_write(u32 blk, u32 off, u32 len, const u8 *in);
int  nand_erase(u32 blk, u32 count);

/* cmd.c */
void cmd_dispatch(u8 cmd, u8 *data, u32 len);

/* 传输后端（usb.c / uart.c 二选一，FDL1 已初始化） */
int  link_init(void);
int  link_recv(u8 *buf, u32 max);          /* 返回帧长度，0 = 无数据，<0 = 错误 */
int  link_send(const u8 *buf, u32 len);

/* 工具 */
void mem_copy(void *dst, const void *src, u32 n);
void mem_set(void *dst, u8 v, u32 n);
void log_str(const char *s);

/* 复位 / 断电（不返回） */
void sys_reset(void);
void sys_关机(void);

#endif
