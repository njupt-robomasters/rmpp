include_guard(GLOBAL)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/gcc-arm-none-eabi.cmake")

# 链接脚本：换成 bsp/dma_buffer.ld，它 INCLUDE 生成的脚本后追加 DMA 缓冲区；-L 提供 INCLUDE 搜索目录
string(REPLACE
    "${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld"
    "${CMAKE_CURRENT_LIST_DIR}/dma_buffer.ld\" -L \"${CMAKE_CURRENT_LIST_DIR}/.."
    CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}"
)

# map 文件：链接工作目录 → 构建根目录
string(REPLACE
    "${CMAKE_PROJECT_NAME}.map"
    "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map"
    CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}"
)
