cmake_minimum_required(VERSION 3.22.1)
project(gemini_eis_jni LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(GEMINI_EIS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../.." ABSOLUTE)

add_library(gemini_eis_jni SHARED
    ${GEMINI_EIS_ROOT}/src/api/gemini_eis_c_api.cpp
    ${GEMINI_EIS_ROOT}/src/core/gemini_eis_core.cpp
    ${GEMINI_EIS_ROOT}/src/control/constrained_mpc.cpp
    ${GEMINI_EIS_ROOT}/src/control/intent_scheduler.cpp
    ${GEMINI_EIS_ROOT}/src/state/motion_state_est.cpp
    ${GEMINI_EIS_ROOT}/src/visual/async_visual_worker.cpp
    ${GEMINI_EIS_ROOT}/src/warp/rs19_one_warp.cpp
    ${GEMINI_EIS_ROOT}/adapters/android/GeminiLiveJni.cpp
)

target_include_directories(gemini_eis_jni PRIVATE
    ${GEMINI_EIS_ROOT}/include
    ${GEMINI_EIS_ROOT}/include/api
    ${GEMINI_EIS_ROOT}/adapters/android
)

target_compile_definitions(gemini_eis_jni PRIVATE GEMINI_EIS_STATIC)
target_compile_options(gemini_eis_jni PRIVATE -O3 -Wall -Wextra)

find_library(log-lib log)
find_library(gles3-lib GLESv3)

target_link_libraries(gemini_eis_jni PRIVATE
    ${log-lib}
    ${gles3-lib}
)
