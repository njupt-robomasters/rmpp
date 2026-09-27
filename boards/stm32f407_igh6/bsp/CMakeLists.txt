# 接入 CubeMX 生成的固件入口、HAL 和 USB
add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/stm32cubemx"
    "${CMAKE_CURRENT_BINARY_DIR}/stm32cubemx"
)

# 编译手写外设封装和 FlashDB
file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_bsp OBJECT ${BSP_SOURCES})

# FlashDB 内部使用 <flashdb.h> 等引用，需要显式提供头文件搜索目录
target_include_directories(rmpp_bsp PRIVATE flashdb)
