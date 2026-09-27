cmake_minimum_required(VERSION 3.22)

# 先由应用选择板卡，工具链必须在 project 启用编译器之前指定
if(NOT DEFINED RMPP_APP OR RMPP_APP STREQUAL "")
    message(FATAL_ERROR "请通过 cmake --preset 选择应用")
endif()

include("app/${RMPP_APP}/board.cmake")
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/${RMPP_BOARD}/bsp/toolchain.cmake")

project(rmpp LANGUAGES C CXX ASM)

# 所有目标统一语言标准，保留 GNU 扩展
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# 导出编译命令，供 IDE 和代码分析工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 先创建固件目标，CubeMX 内层配置才能向它添加生成源码
add_executable(${CMAKE_PROJECT_NAME})

# 创建模块目标，待全部存在后统一声明依赖
add_subdirectory(lib)
add_subdirectory("boards/${RMPP_BOARD}/bsp" bsp)
add_subdirectory("app/${RMPP_APP}" app)

# 应用使用公共接口，板卡实现还需要 CubeMX 的头文件和宏
target_link_libraries(rmpp_app PRIVATE rmpp_lib)
target_link_libraries(rmpp_bsp PRIVATE rmpp_lib stm32cubemx)

# TODO: 解除公共库 EKF 对板卡 CMSIS-DSP 编译接口的依赖，届时删除此行
# DSP 二进制仍由 CubeMX 接入最终固件，此处只为公共库提供编译接口
target_link_libraries(rmpp_lib PRIVATE stm32cubemx)

# 显式组装三组对象文件；与 CubeMX 对同一固件目标使用的 plain 签名保持一致
target_link_libraries(${CMAKE_PROJECT_NAME}
    rmpp_app
    rmpp_bsp
    rmpp_lib
)
