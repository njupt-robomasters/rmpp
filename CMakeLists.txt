cmake_minimum_required(VERSION 3.22)

# 应用由 preset 指定；一个构建目录对应一个应用和板卡。
if(NOT DEFINED RMPP_APP OR RMPP_APP STREQUAL "")
    message(FATAL_ERROR "请通过 cmake --preset 选择应用")
endif()

set(RMPP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(RMPP_APP_DIR "${RMPP_ROOT}/app/${RMPP_APP}")
include("${RMPP_APP_DIR}/CMakeLists.txt")

# 工具链必须在 project() 识别编译器之前确定。
set(RMPP_BOARD_DIR "${RMPP_ROOT}/boards/${RMPP_BOARD}")
set(CMAKE_TOOLCHAIN_FILE "${RMPP_BOARD_DIR}/bsp/toolchain.cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(rmpp LANGUAGES C CXX ASM)

add_subdirectory(lib)
add_subdirectory("${RMPP_BOARD_DIR}/bsp" "${CMAKE_BINARY_DIR}/bsp")

file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS "${RMPP_APP_DIR}/*.*")
add_library(rmpp_app OBJECT ${APP_SOURCES})
target_link_libraries(rmpp_app PRIVATE rmpp_lib rmpp_bsp)
# CubeMX 创建固件目标；此处保持与生成文件一致的链接调用形式。
target_link_libraries(${RMPP_FIRMWARE_TARGET} rmpp_app rmpp_bsp rmpp_lib)
