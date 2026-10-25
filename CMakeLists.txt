cmake_minimum_required(VERSION 3.22)

# 指定 RMPP_APP 以选择应用
if (NOT DEFINED RMPP_APP OR RMPP_APP STREQUAL "")
    message(FATAL_ERROR "指定 RMPP_APP 以选择应用")
endif ()

# 设置工具链
include("app/${RMPP_APP}/board.cmake")
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/${RMPP_BOARD}/bsp/toolchain.cmake")

project(rmpp LANGUAGES C CXX ASM)

# 设置语言标准
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON) # 要求编译器支持指定标准
set(CMAKE_C_EXTENSIONS ON) # 使用 GNU 方言

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 要求编译器支持指定标准
set(CMAKE_CXX_EXTENSIONS ON) # 使用 GNU 方言

# 导出编译命令，供分析使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 先创建固件目标，stm32cubemx 内层配置会向它添加东西
add_executable(${CMAKE_PROJECT_NAME})

# 加载 app lib bsp 子目录
add_subdirectory("app/${RMPP_APP}" app)
add_subdirectory(lib)
add_subdirectory("boards/${RMPP_BOARD}/bsp" bsp)

# 声明依赖
target_link_libraries(rmpp_app PRIVATE rmpp_lib)
target_link_libraries(rmpp_bsp PRIVATE rmpp_lib)

# 组装 app lib bsp
target_link_libraries(${CMAKE_PROJECT_NAME}
        rmpp_app
        rmpp_lib
        rmpp_bsp
)
