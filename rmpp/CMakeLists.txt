file(GLOB_RECURSE USER_SOURCES
        "lib/*.*"

        # 遥控器
        # "examples/rc/fsi6x/*.*"

        # IMU
        # "examples/imu/*.*"

        # 底盘
        # "examples/motor/m3508/*.*"
        "examples/module/chassis/chassis_omni/*.*"
        # "examples/module/chassis/chassis_mecanum/*.*"
        # "examples/module/chassis/chassis_dualsteer/*.*"

        # 云台
        # "examples/imu/*.*"
        # "examples/motor/mf9025/*.*"
        # "examples/motor/gm6020/*.*"
        # "examples/motor/dm4310/*.*"
        # "examples/module/gimbal/gimbal_classic/*.*"

        # 发射机构
        # "examples/motor/vesc/*.*"
        # "examples/motor/m2006/*.*"
        # "examples/motor/m3508_shoot/*.*"
        # "examples/module/shooter/shooter_17mm_m2006/*.*"
        # "examples/module/shooter/shooter_17mm_m3508/*.*"

        # 裁判系统
        # "examples/referee/*.*"

        # 整车代码
        # "app/infantry/*.*"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        lib
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wno-packed-bitfield-compat)
