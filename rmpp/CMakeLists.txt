file(GLOB_RECURSE USER_SOURCES
        "lib/*.*"

        # 整车代码
        "app/hero/app.cpp"
        # "app/infantry/app.cpp"
        # "app/sentry/app.cpp"

        # 调试代码
        # "app/debug/motor.cpp"
        # "app/debug/led.cpp"
        # "app/debug/buzzer.cpp"
        # "app/debug/rc.cpp"
        # "app/debug/imu.cpp"
        # "app/debug/referee.cpp"
        # "app/debug/chassis_motor.cpp"
        # "app/debug/chassis.cpp"
        # "app/debug/gimbal_motor.cpp"
        # "app/debug/gimbal.cpp"
        # "app/debug/shooter_rub2.cpp"
        # "app/debug/shooter_rub4.cpp"
        # "app/debug/shooter_shoot.cpp"
        # "app/debug/shooter.cpp"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        lib
        lib/flashdb/lib
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wno-packed-bitfield-compat)
