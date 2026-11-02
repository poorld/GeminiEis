cmake_minimum_required(VERSION 3.16)
project(GeminiEisAndroid LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(GEMINI_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

add_library(gemini_eis_core STATIC IMPORTED)
set_target_properties(gemini_eis_core PROPERTIES
    IMPORTED_LOCATION "${GEMINI_ROOT}/dist/android/${ANDROID_ABI}/libgemini_eis_core.a"
    INTERFACE_INCLUDE_DIRECTORIES "${GEMINI_ROOT}/include"
)

add_library(gemini_eis_jni SHARED gemini_eis_jni.cpp)
target_include_directories(gemini_eis_jni PRIVATE "${GEMINI_ROOT}/include")
target_compile_definitions(gemini_eis_jni PRIVATE GEMINI_EIS_STATIC)
target_link_libraries(gemini_eis_jni PRIVATE gemini_eis_core log)
