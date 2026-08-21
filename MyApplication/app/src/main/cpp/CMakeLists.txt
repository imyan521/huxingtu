cmake_minimum_required(VERSION 3.22.1)
project(wallpipeline)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================
# 绝对正确的 OpenCV 路径
# 对应你的路径：cpp/opencv/include
# ==============================
include_directories(${CMAKE_SOURCE_DIR}/opencv/include)

# 导入 OpenCV 库
add_library(opencv_java4 SHARED IMPORTED)
set_target_properties(opencv_java4 PROPERTIES
        IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/opencv/libs/${ANDROID_ABI}/libopencv_java4.so
)

# 编译你的墙面拟合 C++ 代码
add_library(
        wallpipeline
        SHARED
        native-lib.cpp
        map_clean.cpp
        linefitter.cpp
)

# 链接所有库
target_link_libraries(
        wallpipeline
        opencv_java4
        android
        log

)
