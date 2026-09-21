#pragma once

#ifdef __cplusplus
#include "board/board.hpp"
extern "C" {
#endif
void bsp_init(void);
void app_main(void);
#ifdef __cplusplus
}
#endif
