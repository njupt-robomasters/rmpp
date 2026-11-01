include_guard(GLOBAL)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/gcc-arm-none-eabi.cmake")

# 链接脚本：项目根目录 → boards/stm32f407_igh6/
string(REPLACE
    "${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld"
    "${CMAKE_CURRENT_LIST_DIR}/../STM32F407XX_FLASH.ld"
    CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}"
)

# map 文件：链接工作目录 → 构建根目录
string(REPLACE
    "${CMAKE_PROJECT_NAME}.map"
    "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map"
    CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}"
)
