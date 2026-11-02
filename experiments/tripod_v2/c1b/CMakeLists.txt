cmake_minimum_required(VERSION 3.18)
project(GeminiC1BNative LANGUAGES CXX)

# Isolated portable C1-B target; no changes to the product build.
add_library(gemini_v2c_c1b SHARED
    ../C1NativeTileLK.cpp
    ../C1NativeAsyncLK.cpp)
target_include_directories(gemini_v2c_c1b PRIVATE ..)
target_compile_features(gemini_v2c_c1b PUBLIC cxx_std_17)

# 20 physical RS Gyro rows with fine feature-only spatial mesh.
add_library(gemini_v2c_c1_prewarp SHARED ../C1NativeRsPrewarpFine.cpp)
target_include_directories(gemini_v2c_c1_prewarp PRIVATE .. ../../../include)
target_compile_features(gemini_v2c_c1_prewarp PUBLIC cxx_std_17)

# C1-B2 Camera YUV luma 2560x1440 stride-aware area8 + native 20 Row RS19
# feature prewarp. Portable C++17; SHADOW analysis, NOT final GPU Warp.
add_library(gemini_v2c_c1b2_yuv SHARED
    ../C1B2NativeYuv.cpp
    ../C1NativeRsPrewarpFine.cpp)
target_include_directories(gemini_v2c_c1b2_yuv PRIVATE .. ../../../include)
target_compile_features(gemini_v2c_c1b2_yuv PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(gemini_v2c_c1b PRIVATE /W4)
    target_compile_options(gemini_v2c_c1_prewarp PRIVATE /W4)
    target_compile_options(gemini_v2c_c1b2_yuv PRIVATE /W4)
else()
    target_compile_options(gemini_v2c_c1b PRIVATE -Wall -Wextra)
    target_compile_options(gemini_v2c_c1_prewarp PRIVATE -Wall -Wextra)
    target_compile_options(gemini_v2c_c1b2_yuv PRIVATE -Wall -Wextra)
endif()
