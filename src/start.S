@
@ start.S — 自研 FDL2 启动代码
@
@ 参考固件在 0x80100200 处的启动序列（逆向分析确定），非代码复制
@ 入口：0x80100200（DHTB 头 0x200 字节之后）
@ 栈顶：0x80010000（原版同值，说明该区域可用）
@
@ ⚠️ 这是 ARM 架构通用启动写法，非抄录原版代码。
@

    .syntax unified
    .arm
    .fpu   neon

    .section .text.start, "ax", %progbits
    .global _start
    .type   _start, %function

_start:
    @ ★ 先保存 BROM 传进来的 r0
    @   原版 0x80100268 处是 `ldm r0,{r1,r3,sp,pc}` —— 从 r0 指向的结构
    @   恢复上下文并跳转。说明 BROM 跳转时会传参，通道信息可能就在里面。
    @   必须在任何代码破坏 r0 之前存下来。
    ldr     r12, =__boot_arg
    str     r0, [r12]

    @ 关 I-cache
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #(1 << 7)
    mcr     p15, 0, r0, c1, c0, 0

    @ 允许协处理器 10/11 访问（NEON/VFP）
    mrc     p15, 0, r0, c1, c0, 1
    orr     r0, r0, #(1 << 6)
    mcr     p15, 0, r0, c1, c0, 1

    @ CPACR：放开 CP10/CP11
    mov     r0, #0x00F00000
    mcr     p15, 0, r0, c1, c0, 2

    @ 开 FPU（不用浮点也无害，与原版一致）
    mov     r0, #0x40000000
    vmsr    fpexc, r0
    isb

    @ 进 SVC 模式并关中断（0xD3 = SVC | I | F）
    mov     sl, #0
    mov     r0, #0xD3
    msr     cpsr_c, r0

    @ 设栈
    ldr     r1, =__stack_top
    mov     sp, r1

    @ 清 BSS
    ldr     r0, =__bss_start
    ldr     r1, =__bss_end
    mov     r2, #0
1:  cmp     r0, r1
    strlo   r2, [r0], #4
    blo     1b

    @ 进 C
    bl      main

    @ main 不应返回；真返回了就死循环
2:  b       2b
    .size   _start, . - _start

    .ltorg

    .end
