/*
 * linker.ld — 自研 FDL2 链接脚本
 *
 * 代码起点 0x80100200：DHTB 头占 0x200 字节，由 mkimage.py 在打包时补上。
 * 栈顶   0x80010000：与原版一致（原版 0x80100238 处取该值），说明该区域可用。
 *
 * RAM 布局（推断自原版）：
 *   0x80000000 ┌──────────┐
 *              │ 栈 64 KiB │ ← SP 初值 0x80010000，向下生长
 *   0x80010000 ├──────────┤
 *              │  保护带   │
 *   0x80100000 ├──────────┤
 *              │ DHTB 头   │ 0x200
 *   0x80100200 ├──────────┤
 *              │ 代码+数据 │
 *              ├──────────┤
 *              │   BSS     │
 *              └──────────┘
 */

ENTRY(_start)

SECTIONS
{
    . = 0x80100200;

    .text : {
        KEEP(*(.text.start))
        *(.text*)
    }

    .rodata : {
        *(.rodata*)
    }

    .data : {
        *(.data*)
    }

    . = ALIGN(4);
    __bss_start = .;
    .bss : {
        *(.bss*)
        *(COMMON)
    }
    . = ALIGN(4);
    __bss_end = .;

    __stack_top = 0x80010000;

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.extab*)
        *(.comment)
        *(.note*)
    }
}
