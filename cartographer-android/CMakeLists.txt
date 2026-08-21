cmake_minimum_required(VERSION 3.22.1)
project(cartographer-jni)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTIONS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti -fexceptions -fPIC")

if(NOT ANDROID_ABI)
    set(ANDROID_ABI "arm64-v8a")
endif()

# Cartographer：你当前在源码树内构建时，静态库在 build-android/（多 ABI 时应为各 ABI 独立 build 目录）
set(CARTO_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../cartographer)
set(CARTO_BUILD_DIR ${CARTO_SOURCE_DIR}/build-android)
set(CARTO_STATIC_LIB ${CARTO_BUILD_DIR}/libcartographer.a)
if(NOT EXISTS "${CARTO_STATIC_LIB}")
    message(FATAL_ERROR "找不到 Cartographer 静态库: ${CARTO_STATIC_LIB}\n请先在 ${CARTO_BUILD_DIR} 用 NDK 工具链配置并编译 Cartographer。")
endif()

# 第三方：与 NDK 交叉编译安装前缀一致
set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/install)

# libcartographer.a 必须与当前 third_party/install/abseil 同一套头文件编译。
# 若 Cartographer 在「无 lts_20230802 内联命名空间」的旧 Abseil 头下编成，而 install 里是 LTS 带 lts_20230802 的 .a，
# 会出现大量 undefined absl::container_internal::*（再多 GLOB absl 也链不上）。
set(_ABSL_RAWHS "${THIRD_PARTY}/abseil/lib/libabsl_raw_hash_set.a")
set(_NM_TOOL "")
if(DEFINED ANDROID_TOOLCHAIN_ROOT AND EXISTS "${ANDROID_TOOLCHAIN_ROOT}/bin/llvm-nm")
    set(_NM_TOOL "${ANDROID_TOOLCHAIN_ROOT}/bin/llvm-nm")
elseif(DEFINED ANDROID_NDK)
    if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
        set(_NM_TOOL "${ANDROID_NDK}/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-nm")
    elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
        set(_NM_TOOL "${ANDROID_NDK}/toolchains/llvm/prebuilt/darwin-x86_64/bin/llvm-nm")
    endif()
endif()
if(_NM_TOOL AND EXISTS "${_NM_TOOL}" AND EXISTS "${_ABSL_RAWHS}")
    execute_process(
        COMMAND /bin/sh -c "\"${_NM_TOOL}\" -g \"${CARTO_STATIC_LIB}\" | grep -q \"U _ZN4absl18container_internal\""
        RESULT_VARIABLE _carto_old_absl_nm
        ERROR_QUIET)
    execute_process(
        COMMAND /bin/sh -c "\"${_NM_TOOL}\" -g \"${_ABSL_RAWHS}\" | grep -q \"lts_20230802\""
        RESULT_VARIABLE _absl_is_lts_nm
        ERROR_QUIET)
    if(_carto_old_absl_nm EQUAL 0 AND _absl_is_lts_nm EQUAL 0)
        message(FATAL_ERROR
            "libcartographer.a 与 third_party/install/abseil 的 C++ 符号命名不一致：\n"
            "  当前 Cartographer 静态库是在「旧 Abseil 头（符号形如 absl::container_internal::…，无 lts_20230802）」下编译的；\n"
            "  而 third_party/install 里的 abseil 为 LTS（库内符号带 lts_20230802），二者不能一起链接。\n\n"
            "请删除 build 目录后，用「与 JNI 相同的」${THIRD_PARTY}/abseil/include 重新配置并完整编译 Cartographer，再编 app。\n"
            "  rm -rf \"${CARTO_BUILD_DIR}\" && mkdir -p \"${CARTO_BUILD_DIR}\" && cd \"${CARTO_BUILD_DIR}\" && \\\n"
            "    cmake \"${CARTO_SOURCE_DIR}\" -DCMAKE_TOOLCHAIN_FILE=\\$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake \\\n"
            "      -DANDROID_ABI=${ANDROID_ABI} -DANDROID_PLATFORM=android-34 \\\n"
            "      -DCMAKE_PREFIX_PATH=\"${THIRD_PARTY}/abseil;...\"（其余依赖路径按你原先 Cartographer 工程填写）\n"
            "    && cmake --build . --parallel\n")
    endif()
endif()

# 可选：若曾 cmake --install 到 cartographer-android/libs/<ABI>，仍可用其中的头文件
set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${ANDROID_ABI})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/jni
    ${LIBS_DIR}/include                  # <--- 重点修改：去掉了后面的 /cartographer
    ${THIRD_PARTY}/eigen/include/eigen3
    ${THIRD_PARTY}/abseil/include
    ${THIRD_PARTY}/lua/include
    ${THIRD_PARTY}/boost/include
    ${THIRD_PARTY}/ceres/include
    ${THIRD_PARTY}/glog/include
    ${THIRD_PARTY}/protobuf/include
    ${CARTO_SOURCE_DIR}
    ${CARTO_BUILD_DIR}
)

add_library(cartographer-jni SHARED
    jni/cartographer_jni.cpp
)

find_library(log-lib log)

# Floor plan fitting: reuse the C++ port of the Python 0702 pipeline and link
# against the Android OpenCV package already present in this workspace.
set(FLOORPLAN_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../floorplan_cpp)
set(OPENCV_ANDROID_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../MyApplication/app/src/main/cpp/opencv)
set(OPENCV_JAVA4_SO ${OPENCV_ANDROID_DIR}/libs/${ANDROID_ABI}/libopencv_java4.so)
if(NOT EXISTS "${OPENCV_JAVA4_SO}")
    message(FATAL_ERROR "找不到 Android OpenCV: ${OPENCV_JAVA4_SO}")
endif()

add_library(opencv_java4 SHARED IMPORTED)
set_target_properties(opencv_java4 PROPERTIES
    IMPORTED_LOCATION "${OPENCV_JAVA4_SO}"
)

add_library(floorplan-core STATIC
    ${FLOORPLAN_CPP_DIR}/floor_plan.cpp
)
target_include_directories(floorplan-core PUBLIC
    ${FLOORPLAN_CPP_DIR}
    ${OPENCV_ANDROID_DIR}/include
)
target_link_libraries(floorplan-core PUBLIC opencv_java4)

add_library(floorplan-jni SHARED
    jni/floorplan_jni.cpp
)
target_include_directories(floorplan-jni PRIVATE
    ${FLOORPLAN_CPP_DIR}
    ${OPENCV_ANDROID_DIR}/include
)
target_link_libraries(floorplan-jni PRIVATE
    floorplan-core
    opencv_java4
    log
    android
)
target_link_options(floorplan-jni PRIVATE
    "-Wl,-z,max-page-size=16384"
    "-Wl,--gc-sections"
    "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/jni/native.exports.map"
)

# 链入 install 目录下全部 Abseil 静态库（Cartographer 会用到 raw_hash_set / mutex / str_cat 等，子集链接会缺符号）
file(GLOB ABSL_LIBS "${THIRD_PARTY}/abseil/lib/libabsl_*.a")
list(SORT ABSL_LIBS)

# 静态库循环依赖：用 linker group 让 lld 反复解析
target_link_libraries(
    cartographer-jni
    PRIVATE
    -Wl,--start-group
    ${CARTO_STATIC_LIB}
    ${THIRD_PARTY}/ceres/lib/libceres.a
    ${THIRD_PARTY}/glog/lib/libglog.a
    ${THIRD_PARTY}/lua/lib/liblua.a
    ${THIRD_PARTY}/protobuf/lib/libprotobuf.a
    ${THIRD_PARTY}/boost/lib/libboost_iostreams.a
    ${ABSL_LIBS}
    -Wl,--end-group
    log
    android
    z
    atomic
    m
)

target_link_options(cartographer-jni PRIVATE
    "-Wl,-z,max-page-size=16384"
    "-Wl,--gc-sections"
    "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/jni/native.exports.map"
)
