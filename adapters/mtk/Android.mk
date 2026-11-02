LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := libgemini_eis_mtk
LOCAL_MODULE_TAGS := optional
LOCAL_MULTILIB := both
LOCAL_PROPRIETARY_MODULE := true

LOCAL_SRC_FILES := \
    GeminiMtkWpeAdapter.cpp \
    ../../src/api/gemini_eis_c_api.cpp \
    ../../src/core/gemini_eis_core.cpp \
    ../../src/control/constrained_mpc.cpp \
    ../../src/control/intent_scheduler.cpp \
    ../../src/state/motion_state_est.cpp \
    ../../src/visual/async_visual_worker.cpp \
    ../../src/warp/rs19_one_warp.cpp

LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/../../include \
    $(LOCAL_PATH)/../../include/api \
    $(LOCAL_PATH)/../../adapters

LOCAL_CPPFLAGS += -std=c++17 -O3 -Wall -Wextra -fPIC
include $(BUILD_SHARED_LIBRARY)
