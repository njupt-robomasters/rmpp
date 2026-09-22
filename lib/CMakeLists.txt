# 业务源码与公共外设声明统一编译；硬件实现由 BSP 提供。
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_lib STATIC ${LIB_SOURCES})
target_include_directories(rmpp_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(rmpp_lib PUBLIC cxx_std_20)
target_compile_options(rmpp_lib PRIVATE -Wno-packed-bitfield-compat)
