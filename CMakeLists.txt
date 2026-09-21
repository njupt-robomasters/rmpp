cmake_minimum_required(VERSION 3.22)

set(RMPP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(RMPP_APP "hero" CACHE STRING "Application")
include("${RMPP_ROOT}/app/${RMPP_APP}/CMakeLists.txt")
project(rmpp_workspace LANGUAGES C CXX ASM)
rmpp_add_app()
