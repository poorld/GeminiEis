cmake_minimum_required(VERSION 3.18)
project(gemini_eis_android_jni LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GEMINI_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

add_library(gemini_eis_jni SHARED
    GeminiLiveJni.cpp
    ${GEMINI_ROOT}/src/api/gemini_eis_c_api.cpp
    ${GEMINI_ROOT}/src/core/gemini_eis_core.cpp
    ${GEMINI_ROOT}/src/control/constrained_mpc.cpp
    ${GEMINI_ROOT}/src/control/intent_scheduler.cpp
    ${GEMINI_ROOT}/src/state/motion_state_est.cpp
    ${GEMINI_ROOT}/src/visual/async_visual_worker.cpp
    ${GEMINI_ROOT}/src/warp/rs19_one_warp.cpp
)

target_include_directories(gemini_eis_jni PRIVATE
    ${GEMINI_ROOT}/include
    ${GEMINI_ROOT}/include/api
    ${GEMINI_ROOT}/adapters
)

target_compile_options(gemini_eis_jni PRIVATE -O3 -Wall -Wextra)
target_link_libraries(gemini_eis_jni log android GLESv3 EGL)
