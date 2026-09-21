add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}/cubemx")

file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_bsp OBJECT ${BSP_SOURCES})
target_include_directories(rmpp_bsp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(rmpp_bsp PRIVATE rmpp_lib stm32cubemx)

# Platform configuration and DSP headers needed by the portable libraries.
target_include_directories(rmpp_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/flashdb")
target_include_directories(rmpp_lib SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/ST/ARM/DSP/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/../Drivers/CMSIS/Include")
target_compile_definitions(rmpp_lib PUBLIC ARM_MATH_CM4)
target_link_libraries(rmpp_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a")

target_include_directories(rmpp PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(rmpp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    ADDITIONAL_CLEAN_FILES "${CMAKE_BINARY_DIR}/rmpp.map")
set(RMPP_FIRMWARE_TARGET rmpp PARENT_SCOPE)
