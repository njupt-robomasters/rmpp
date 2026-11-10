add_library(rmpp_app OBJECT ${CMAKE_CURRENT_SOURCE_DIR}/app.cpp)

# EKF_OPTIONS：仅给 quaternion_ekf.cpp 追加的编译选项（分号分隔），lib 在 app 之后加入，故延迟到根目录末尾设置
if (DEFINED EKF_OPTIONS)
    cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL
            set_source_files_properties ${CMAKE_SOURCE_DIR}/lib/attitude/quaternion_ekf.cpp
            DIRECTORY ${CMAKE_SOURCE_DIR}/lib PROPERTIES COMPILE_OPTIONS "${EKF_OPTIONS}")
endif ()
