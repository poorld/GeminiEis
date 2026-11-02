cmake_minimum_required(VERSION 3.16)
project(GeminiEis VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(GEMINI_EIS_SOURCES
    src/api/gemini_eis_c_api.cpp
    src/core/gemini_eis_core.cpp
    src/control/constrained_mpc.cpp
    src/control/intent_scheduler.cpp
    src/state/motion_state_est.cpp
    src/visual/async_visual_worker.cpp
    src/warp/rs19_one_warp.cpp
)

add_library(gemini_eis_core STATIC ${GEMINI_EIS_SOURCES})
target_include_directories(gemini_eis_core PUBLIC include)
target_compile_definitions(gemini_eis_core PUBLIC GEMINI_EIS_STATIC)
set_target_properties(gemini_eis_core PROPERTIES POSITION_INDEPENDENT_CODE ON OUTPUT_NAME gemini_eis_core)

add_library(gemini_eis_core_shared SHARED ${GEMINI_EIS_SOURCES})
target_include_directories(gemini_eis_core_shared PUBLIC include)
target_compile_definitions(gemini_eis_core_shared PRIVATE GEMINI_EIS_BUILDING_LIBRARY)
set_target_properties(gemini_eis_core_shared PROPERTIES OUTPUT_NAME gemini_eis_core)

foreach(target gemini_eis_core gemini_eis_core_shared)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endforeach()

if(NOT ANDROID)
    add_executable(gemini_eis_sim sim/main.cpp)
    add_executable(gemini_eis_synth sim/synthetic_tests.cpp)
    add_executable(gemini_eis_c_api_test sim/c_api_tests.cpp)
    add_executable(gemini_eis_replay sim/replay_dataset.cpp)

    target_link_libraries(gemini_eis_sim PRIVATE gemini_eis_core)
    target_link_libraries(gemini_eis_synth PRIVATE gemini_eis_core)
    target_link_libraries(gemini_eis_c_api_test PRIVATE gemini_eis_core)
    target_include_directories(gemini_eis_c_api_test PRIVATE adapters)
    target_link_libraries(gemini_eis_replay PRIVATE gemini_eis_core)

    enable_testing()
    add_test(NAME gemini_eis_smoke COMMAND gemini_eis_sim --self-test)
    add_test(NAME gemini_eis_synth COMMAND gemini_eis_synth)
    add_test(NAME gemini_eis_c_api COMMAND gemini_eis_c_api_test)
endif()
