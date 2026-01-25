file(GLOB_RECURSE USER_SOURCES
        "lib/*.*"

        # 遥控器
        # "examples/rc/*.*"

        # 底盘
        # "examples/motor/m3508/*.*"
        # "examples/module/chassis/chassis_omni/*.*"

        # 云台
        # "examples/imu/*.*"
        # "examples/motor/gm6020/*.*"
        # "examples/motor/dm4310/*.*"
        # "examples/module/gimbal/gimbal_classic/*.*"

        # 发射结构
        # "examples/motor/vesc/*.*"
        # "examples/motor/m2006/*.*"
        # "examples/motor/m3508_shoot/*.*"
        # "examples/module/shooter/shooter_17mm_m2006/*.*"
        "examples/module/shooter/shooter_17mm_m3508/*.*"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        lib
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wno-packed-bitfield-compat)
