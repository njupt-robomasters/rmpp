/*
 * 在生成的链接脚本基础上追加 DMA 缓冲区，生成文件保持原样；由 toolchain.cmake 作为唯一 -T 传入
 * DMA1/DMA2 无法访问 DTCM，而生成脚本把 .data/.bss 放在 DTCMRAM；
 * 标记 __attribute__((section(".dma_buffer"))) 的变量放入 AXI SRAM（RAM_D1），NOLOAD，启动时不清零
 */
INCLUDE STM32H723XG_FLASH.ld

SECTIONS
{
  .dma_buffer (NOLOAD) : ALIGN(32)
  {
    *(.dma_buffer)
    *(.dma_buffer*)
  } >RAM_D1
}
INSERT AFTER .bss;
