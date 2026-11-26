# 共用上级 app.cpp 与 songs.hpp，SONG 指定曲目，EQUAL_MIX 为 1 时不区分旋律与伴奏
add_library(rmpp_app OBJECT "${CMAKE_CURRENT_SOURCE_DIR}/../app.cpp")
target_compile_definitions(rmpp_app PRIVATE SONG=ALWAYS_WITH_ME EQUAL_MIX=1)

# 测试代码直接使用 HAL 的 htim12，验证后迁入 lib/bsp
target_link_libraries(rmpp_app PRIVATE stm32cubemx)
