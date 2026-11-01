#pragma once

// FlashDB 配置，完整选项见 third_party/FlashDB/inc/fdb_cfg_template.h

#define FDB_USING_KVDB
#define FDB_USING_FAL_MODE

// W25Q64 为 NOR Flash，写入粒度 1 位
#define FDB_WRITE_GRAN 1

// 不输出日志
#define FDB_PRINT(...)
