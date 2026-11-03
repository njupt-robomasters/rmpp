#pragma once

// FAL 配置：板载 W25Q64（8 MiB），驱动见 fal_flash_w25q64.c

#define FAL_DEBUG 0
#define FAL_PRINTF(...) // 不输出日志，避免链入 printf
#define FAL_PART_HAS_TABLE_CFG

extern const struct fal_flash_dev w25q64;

#define FAL_FLASH_DEV_TABLE {&w25q64}

// KvStore 分区：从 0 起 8 KB，其余空间保留
#define FAL_PART_TABLE {{FAL_PART_MAGIC_WORD, "fdb_kvdb1", "w25q64", 0, 8 * 1024, 0}}
