# 接入 stm32cubemx
add_subdirectory(
        "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/stm32cubemx"
        "${CMAKE_CURRENT_BINARY_DIR}/stm32cubemx"
)

# 编译 CherryUSB：设备栈、CDC ACM、DWC2 驱动及 ST 适配，配置见 usb_config.h
set(CHERRYUSB "${CMAKE_SOURCE_DIR}/third_party/CherryUSB")
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

# 编译 FlashDB：KVDB 与 FAL，配置 fdb_cfg.h、fal_cfg.h 及 Flash 驱动在 flashdb/
set(FLASHDB "${CMAKE_SOURCE_DIR}/third_party/FlashDB")
add_library(flashdb OBJECT
        "${FLASHDB}/src/fdb.c"
        "${FLASHDB}/src/fdb_kvdb.c"
        "${FLASHDB}/src/fdb_utils.c"
        "${FLASHDB}/port/fal/src/fal.c"
        "${FLASHDB}/port/fal/src/fal_flash.c"
        "${FLASHDB}/port/fal/src/fal_partition.c"
)
target_include_directories(flashdb PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/flashdb" # fdb_cfg.h、fal_cfg.h
        "${FLASHDB}/inc"
        "${FLASHDB}/port/fal/inc"
)

# 编译 bsp，含 flashdb/ 下的 Flash 驱动
file(GLOB_RECURSE BSP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.*")
add_library(rmpp_bsp OBJECT ${BSP_SOURCES})
target_include_directories(rmpp_bsp PRIVATE include) # 板卡专用接口 board/*.hpp

# bsp 依赖 stm32cubemx、CherryUSB 与 FlashDB
target_link_libraries(rmpp_bsp PRIVATE stm32cubemx cherryusb flashdb)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE $<TARGET_OBJECTS:cherryusb> $<TARGET_OBJECTS:flashdb>)
