/*
 * W25Q64 FAL 驱动：OCTOSPI1 间接模式，单线指令、地址与数据，CPU 轮询，不用 DMA 与内存映射
 * SCLK = HCLK 260 MHz / 6 = 43.3 MHz，低于普通读 0x03 的 50 MHz 上限
 * 只在 bsp_init 之后由 KvStore 首次读写触发，此时 MX_OCTOSPI1_Init 与 SysTick 均已就绪
 */

#include <fal.h>
#include "octospi.h"

#define PAGE_SIZE 256
#define SECTOR_SIZE 4096
#define TIMEOUT_MS 1000 // 单次擦写忙等上限；4 KiB 扇区擦除典型 45 ms、最大 400 ms

// 指令
#define CMD_READ 0x03
#define CMD_PAGE_PROGRAM 0x02
#define CMD_SECTOR_ERASE 0x20
#define CMD_WRITE_ENABLE 0x06
#define CMD_READ_STATUS 0x05
#define CMD_JEDEC_ID 0x9F
#define CMD_RESET_ENABLE 0x66
#define CMD_RESET 0x99
#define STATUS_BUSY 0x01

// 发送一条单线命令；has_address 为假时无地址阶段，size 为 0 时无数据阶段
static int command(uint8_t instruction, int has_address, uint32_t address, uint32_t size) {
    OSPI_RegularCmdTypeDef cmd = {
        .OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
        .FlashId = HAL_OSPI_FLASH_ID_1,
        .Instruction = instruction,
        .InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
        .InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
        .InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
        .Address = address,
        .AddressMode = has_address ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_NONE,
        .AddressSize = HAL_OSPI_ADDRESS_24_BITS,
        .AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,
        .AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
        .AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE,
        .DataMode = size ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_NONE,
        .NbData = size,
        .DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE,
        .DummyCycles = 0,
        .DQSMode = HAL_OSPI_DQS_DISABLE,
        .SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
    };
    return HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK ? 0 : -1;
}

// 轮询状态寄存器直到 BUSY 清零
static int wait_ready(void) {
    if (command(CMD_READ_STATUS, 0, 0, 1) != 0)
        return -1;
    OSPI_AutoPollingTypeDef cfg = {
        .Match = 0,
        .Mask = STATUS_BUSY,
        .MatchMode = HAL_OSPI_MATCH_MODE_AND,
        .AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE,
        .Interval = 0x10,
    };
    return HAL_OSPI_AutoPolling(&hospi1, &cfg, TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int write_enable(void) {
    return command(CMD_WRITE_ENABLE, 0, 0, 0);
}

// 软复位并核对 JEDEC ID（0xEF4017）
static int flash_init(void) {
    if (command(CMD_RESET_ENABLE, 0, 0, 0) != 0 || command(CMD_RESET, 0, 0, 0) != 0)
        return -1;
    HAL_Delay(1); // 复位恢复时间 30 μs

    uint8_t id[3] = {0};
    if (command(CMD_JEDEC_ID, 0, 0, sizeof(id)) != 0 || HAL_OSPI_Receive(&hospi1, id, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return -1;
    return id[0] == 0xEF && id[1] == 0x40 && id[2] == 0x17 ? 0 : -1;
}

static int flash_read(long offset, uint8_t* buf, size_t size) {
    if (size == 0)
        return 0;
    if (command(CMD_READ, 1, (uint32_t)offset, size) != 0 || HAL_OSPI_Receive(&hospi1, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return -1;
    return (int)size;
}

// 按页拆分编程，单次不跨 256 字节页边界
static int flash_write(long offset, const uint8_t* buf, size_t size) {
    size_t done = 0;
    while (done < size) {
        const uint32_t address = (uint32_t)offset + done;
        size_t chunk = PAGE_SIZE - address % PAGE_SIZE;
        if (chunk > size - done)
            chunk = size - done;

        if (write_enable() != 0 || command(CMD_PAGE_PROGRAM, 1, address, chunk) != 0)
            return -1;
        if (HAL_OSPI_Transmit(&hospi1, (uint8_t*)(buf + done), HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK || wait_ready() != 0)
            return -1;
        done += chunk;
    }
    return (int)size;
}

// 擦除覆盖 [offset, offset + size) 的全部 4 KiB 扇区
static int flash_erase(long offset, size_t size) {
    const uint32_t begin = (uint32_t)offset / SECTOR_SIZE * SECTOR_SIZE;
    const uint32_t end = (uint32_t)offset + size;
    for (uint32_t address = begin; address < end; address += SECTOR_SIZE) {
        if (write_enable() != 0 || command(CMD_SECTOR_ERASE, 1, address, 0) != 0 || wait_ready() != 0)
            return -1;
    }
    return (int)size;
}

const struct fal_flash_dev w25q64 = {
    .name = "w25q64",
    .addr = 0,
    .len = 8 * 1024 * 1024,
    .blk_size = SECTOR_SIZE,
    .ops = {flash_init, flash_read, flash_write, flash_erase},
    .write_gran = 1,
};
