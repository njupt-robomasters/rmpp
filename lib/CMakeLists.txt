# 编译业务源码和公共外设声明
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_lib OBJECT ${LIB_SOURCES})

# 公开业务组件和 BSP 接口的头文件目录
target_include_directories(rmpp_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# 仅对公共库源码屏蔽 packed 位域的新旧 GCC 布局兼容性提示
target_compile_options(rmpp_lib PRIVATE -Wno-packed-bitfield-compat)
