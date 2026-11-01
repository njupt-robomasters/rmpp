# 接入 stm32cubemx
add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/stm32cubemx"
        "${CMAKE_CURRENT_BINARY_DIR}/stm32cubemx"
)

# 编译 CherryUSB：设备栈、CDC ACM、DWC2 驱动及 ST 适配，配置见 usb_config.h
set(CHERRYUSB "${CMAKE_SOURCE_DIR}/lib/third_party/CherryUSB")
add_library(cherryusb OBJECT
        "${CHERRYUSB}/core/usbd_core.c"
        "${CHERRYUSB}/class/cdc/usbd_cdc_acm.c"
        "${CHERRYUSB}/port/dwc2/usb_dc_dwc2.c"
        "${CHERRYUSB}/port/dwc2/usb_glue_st.c"
)
target_include_directories(cherryusb PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}" # usb_config.h
        "${CHERRYUSB}/common"
        "${CHERRYUSB}/core"
        "${CHERRYUSB}/class/cdc"
        "${CHERRYUSB}/class/hub"
        "${CHERRYUSB}/port/dwc2"
)
target_link_libraries(cherryusb PRIVATE stm32cubemx)

# 编译 bsp
file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_bsp OBJECT ${BSP_SOURCES})
target_include_directories(rmpp_bsp PRIVATE flashdb) # FlashDB 内部使用 <flashdb.h> 等引用，需要显式提供头文件搜索目录

# bsp 依赖 stm32cubemx 与 CherryUSB
target_link_libraries(rmpp_bsp PRIVATE stm32cubemx cherryusb)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE $<TARGET_OBJECTS:cherryusb>)
