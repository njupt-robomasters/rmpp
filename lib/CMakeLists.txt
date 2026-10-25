# 编译 Eigen (header-only 库)
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.4.0"
)
target_compile_definitions(eigen INTERFACE
        EIGEN_DONT_VECTORIZE
        EIGEN_MAX_ALIGN_BYTES=0
)

# 编译 lib
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
list(FILTER LIB_SOURCES EXCLUDE REGEX "[/\\\\]third_party[/\\\\]") # 排除 third_party
add_library(rmpp_lib OBJECT ${LIB_SOURCES})
target_include_directories(rmpp_lib PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_options(rmpp_lib PRIVATE
        -Wno-packed-bitfield-compat # 屏蔽 packed 位域的新旧 GCC 布局兼容性提示
)

# lib 依赖 Eigen
target_link_libraries(rmpp_lib PUBLIC eigen)
