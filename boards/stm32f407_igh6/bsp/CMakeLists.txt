# 接入 stm32cubemx
add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/stm32cubemx"
        "${CMAKE_CURRENT_BINARY_DIR}/stm32cubemx"
)

# 编译 bsp
file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_bsp OBJECT ${BSP_SOURCES})
target_include_directories(rmpp_bsp PRIVATE flashdb) # FlashDB 内部使用 <flashdb.h> 等引用，需要显式提供头文件搜索目录

# bsp 依赖 stm32cubemx
target_link_libraries(rmpp_bsp PRIVATE stm32cubemx)
