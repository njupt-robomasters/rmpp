file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_app OBJECT ${APP_SOURCES})
target_sources(rmpp_app PRIVATE ../../jelly_sentry/gimbal_fold.cpp)
