file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_app OBJECT ${APP_SOURCES})

# 测试代码直接使用 HAL 的 htim12，共用曲目在 ../common，验证后迁入 lib/bsp
target_include_directories(rmpp_app PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../common")
target_link_libraries(rmpp_app PRIVATE stm32cubemx)
