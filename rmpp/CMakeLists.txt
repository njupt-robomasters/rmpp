file(GLOB_RECURSE USER_SOURCES
        "lib/*.*"

        # "examples/module/led/*.*"

        # 遥控器
        # "examples/rc/fsi6x/*.*"
        # "examples/rc/vt13/*.*"

        # IMU
        # "examples/imu/*.*"

        # 底盘
        # "examples/motor/m3508_chassis/*.*"
        # "examples/module/chassis/chassis_omni/*.*"
        # "examples/module/chassis/chassis_mecanum_hero/*.*"
        # "examples/module/chassis/chassis_mecanum_engineer/*.*"
        # "examples/module/chassis/chassis_dualsteer/*.*"

        # 云台
        # "examples/motor/mf9025_yaw/*.*"
        # "examples/motor/gm6020_yaw/*.*"
        # "examples/motor/gm6020_yaw_limit/*.*"
        # "examples/motor/dm4310_pitch/*.*"
        # "examples/motor/dm6006_pitch/*.*"
        # "examples/module/gimbal/gimbal_classic_infantry/*.*"
        # "examples/module/gimbal/gimbal_classic_hero/*.*"
        # "examples/module/gimbal/gimbal_dualyaw/*.*"

        # 发射机构
        # "examples/motor/vesc_2rub/*.*"
        # "examples/motor/vesc_4rub/*.*"
        # "examples/motor/m2006_shoot/*.*"
        # "examples/motor/m3508_shoot/*.*"
        # "examples/motor/dm4310_shoot/*.*"
        # "examples/module/shooter/shooter_17mm_m2006/*.*"
        # "examples/module/shooter/shooter_17mm_m3508/*.*"
        # "examples/module/shooter/shooter_42mm_dm4310/*.*"

        # 裁判系统
        # "examples/referee/*.*"

        # 整车代码
        # "app/infantry/*.*"
        # "app/sentry/*.*"
        "app/hero/*.*"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        lib
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wno-packed-bitfield-compat)
