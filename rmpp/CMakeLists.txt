file(GLOB_RECURSE USER_SOURCES
        "lib/*.*"

        # 步兵
        # "app/infantry/debug/led.cpp"
        # "app/infantry/debug/rc.cpp"
        # "app/infantry/debug/imu.cpp"
        # "app/infantry/debug/referee.cpp"
        # "app/infantry/debug/chassis_motor.cpp"
        # "app/infantry/debug/chassis.cpp"
        # "app/infantry/debug/gimbal_yaw.cpp"
        # "app/infantry/debug/gimbal_pitch.cpp"
        # "app/infantry/debug/gimbal.cpp"
        # "app/infantry/debug/shooter_rub.cpp"
        # "app/infantry/debug/shooter_shoot.cpp"
        # "app/infantry/debug/shooter.cpp"
        "app/infantry/app.cpp"

        # 哨兵
        # "app/sentry/debug/led.cpp"
        # "app/sentry/debug/rc.cpp"
        # "app/sentry/debug/imu.cpp"
        # "app/sentry/debug/referee.cpp"
        # "app/sentry/debug/chassis_motor.cpp"
        # "app/sentry/debug/chassis.cpp"
        # "app/sentry/debug/gimbal_yaw1.cpp"
        # "app/sentry/debug/gimbal_yaw2.cpp"
        # "app/sentry/debug/gimbal_pitch.cpp"
        # "app/sentry/debug/gimbal.cpp"
        # "app/sentry/debug/shooter_rub.cpp"
        # "app/sentry/debug/shooter_shoot.cpp"
        # "app/sentry/debug/shooter.cpp"
        # "app/sentry/app.cpp"

        # 英雄
        # "app/hero/debug/led.cpp"
        # "app/sentry/debug/rc.cpp"
        # "app/sentry/debug/imu.cpp"
        # "app/sentry/debug/referee.cpp"
        # "app/hero/debug/chassis_motor.cpp"
        # "app/hero/debug/chassis.cpp"
        # "app/hero/debug/gimbal.cpp"
        # "app/hero/debug/gimbal_yaw2.cpp"
        # "app/hero/debug/gimbal_pitch.cpp"
        # "app/hero/debug/shooter_rub.cpp"
        # "app/hero/debug/shooter_shoot.cpp"
        # "app/hero/debug/shooter.cpp"
        # "app/hero/app.cpp"

        # 工程
        # "app/engineer/debug/led.cpp"
        # "app/engineer/debug/rc.cpp"
        # "app/sentry/debug/referee.cpp"
        # "app/engineer/debug/chassis_motor.cpp"
        # "app/engineer/debug/chassis.cpp"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        lib
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wno-packed-bitfield-compat)
