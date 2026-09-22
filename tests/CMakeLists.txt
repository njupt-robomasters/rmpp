cmake_minimum_required(VERSION 3.22)
project(rmpp_tests LANGUAGES C CXX)
enable_testing()

set(BSP "${CMAKE_CURRENT_SOURCE_DIR}/../boards/stm32f407_igh6/bsp")
set(LIB "${CMAKE_CURRENT_SOURCE_DIR}/../lib")

# 编译真实 BSP，实现中的 HAL 调用由 fake 目录替代。
add_executable(bsp_test bsp_test.cpp c_entry.c
    "${BSP}/bsp.cpp" "${BSP}/can.cpp" "${BSP}/uart.cpp" "${BSP}/cdc.cpp"
    "${BSP}/pwm.cpp" "${BSP}/timer.cpp" "${BSP}/spi.cpp"
    "${LIB}/rgb/rgb.cpp" "${LIB}/imu/lib/bmi088/bmi088_middleware.cpp")
target_include_directories(bsp_test PRIVATE fake "${BSP}" "${LIB}")

# 保留 FlashDB/FAL 原实现，仅将物理 Flash 替换为内存。
file(GLOB FLASHDB_SOURCES CONFIGURE_DEPENDS "${BSP}/storage/flashdb/*.c")
list(FILTER FLASHDB_SOURCES EXCLUDE REGEX "fal_flash_stm32f4\\.c$")
add_executable(storage_test storage_test.cpp fake/flash.cpp "${BSP}/storage/storage.cpp" ${FLASHDB_SOURCES})
target_include_directories(storage_test PRIVATE "${LIB}" "${BSP}/storage/flashdb")
target_compile_definitions(storage_test PRIVATE FDB_USING_NATIVE_ASSERT)

foreach(target bsp_test storage_test)
    target_compile_features(${target} PRIVATE cxx_std_20)
    target_compile_options(${target} PRIVATE -UNDEBUG)
    add_test(NAME ${target} COMMAND ${target})
    if(WIN32)
        get_filename_component(HOST_COMPILER_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
        set_tests_properties(${target} PROPERTIES ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${HOST_COMPILER_DIR}")
    endif()
endforeach()
