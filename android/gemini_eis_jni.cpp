#include <jni.h>
#include <cstdint>
#include <new>

#include "api/gemini_eis_c_api.h"

namespace {

struct AndroidContext {
    GeminiEisHandle handle = nullptr;
    GeminiEisResult last{};
};

AndroidContext* from(jlong h) {
    return reinterpret_cast<AndroidContext*>(static_cast<intptr_t>(h));
}

}  // namespace

extern "C" JNIEXPORT jlong JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativeCreate(
        JNIEnv*, jclass, jint inW, jint inH, jint outW, jint outH,
        jdouble fx, jdouble fy, jdouble cx, jdouble cy, jlong cameraToGyroOffsetNs) {
    auto* ctx = new (std::nothrow) AndroidContext();
    if (!ctx) return 0;
    GeminiEisConfig cfg{};
    GeminiEisDefaultConfig(&cfg);
    cfg.input_width = static_cast<uint32_t>(inW);
    cfg.input_height = static_cast<uint32_t>(inH);
    cfg.output_width = static_cast<uint32_t>(outW);
    cfg.output_height = static_cast<uint32_t>(outH);
    cfg.fx = fx;
    cfg.fy = fy;
    cfg.cx = cx;
    cfg.cy = cy;
    cfg.camera_to_gyro_offset_ns = static_cast<int64_t>(cameraToGyroOffsetNs);
    if (GeminiEisInit(&ctx->handle, &cfg) != GEMINI_EIS_OK) {
        delete ctx;
        return 0;
    }
    return static_cast<jlong>(reinterpret_cast<intptr_t>(ctx));
}

extern "C" JNIEXPORT void JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativeDestroy(JNIEnv*, jclass, jlong h) {
    auto* ctx = from(h);
    if (!ctx) return;
    GeminiEisUninit(&ctx->handle);
    delete ctx;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativeSetViewport(
        JNIEnv*, jclass, jlong h, jint bufferW, jint bufferH, jint outW, jint outH,
        jdouble x, jdouble y, jdouble w, jdouble height) {
    auto* ctx = from(h);
    if (!ctx) return GEMINI_EIS_EINVAL;
    GeminiEisViewport v{};
    v.buffer_width = static_cast<uint32_t>(bufferW);
    v.buffer_height = static_cast<uint32_t>(bufferH);
    v.output_width = static_cast<uint32_t>(outW);
    v.output_height = static_cast<uint32_t>(outH);
    v.roi_x = x;
    v.roi_y = y;
    v.roi_width = w;
    v.roi_height = height;
    return GeminiEisSetViewport(&ctx->handle, &v);
}

extern "C" JNIEXPORT jint JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativePushGyro(
        JNIEnv*, jclass, jlong h, jlong ts, jdouble x, jdouble y, jdouble z) {
    auto* ctx = from(h);
    if (!ctx) return GEMINI_EIS_EINVAL;
    const GeminiEisGyroSample s{static_cast<int64_t>(ts), x, y, z};
    return GeminiEisPushGyro(&ctx->handle, &s, 1);
}

extern "C" JNIEXPORT jint JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativePushVisual(
        JNIEnv*, jclass, jlong h, jlong ts, jdouble tx, jdouble ty,
        jdouble rot, jdouble scale, jdouble trust, jint inliers) {
    auto* ctx = from(h);
    if (!ctx) return GEMINI_EIS_EINVAL;
    GeminiEisVisualMotion v{};
    v.timestamp_ns = static_cast<int64_t>(ts);
    v.tx_px = tx;
    v.ty_px = ty;
    v.rotation_rad = rot;
    v.scale = scale;
    v.trust = trust;
    v.inliers = inliers;
    return GeminiEisPushVisual(&ctx->handle, &v);
}

extern "C" JNIEXPORT jint JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativeProcessFrame(
        JNIEnv*, jclass, jlong h, jlong ts, jlong exposure, jlong skew, jdouble zoom) {
    auto* ctx = from(h);
    if (!ctx) return GEMINI_EIS_EINVAL;
    GeminiEisFrameInfo f{};
    f.sensor_timestamp_ns = static_cast<int64_t>(ts);
    f.exposure_time_ns = static_cast<int64_t>(exposure);
    f.rolling_shutter_skew_ns = static_cast<int64_t>(skew);
    f.zoom = zoom;
    return GeminiEisProcessFrame(&ctx->handle, &f, &ctx->last);
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativeGetCorrection(JNIEnv* env, jclass, jlong h) {
    auto* ctx = from(h);
    if (!ctx) return nullptr;
    jdouble values[9] = {
        ctx->last.correction_rad[0], ctx->last.correction_rad[1], ctx->last.correction_rad[2],
        ctx->last.target_angle_rad[0], ctx->last.target_angle_rad[1], ctx->last.target_angle_rad[2],
        ctx->last.pan_confidence, ctx->last.oscillation_score, ctx->last.visual_trust
    };
    jdoubleArray out = env->NewDoubleArray(9);
    if (out) env->SetDoubleArrayRegion(out, 0, 9, values);
    return out;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativeGetMesh(JNIEnv* env, jclass, jlong h) {
    auto* ctx = from(h);
    if (!ctx) return nullptr;
    const jsize n = static_cast<jsize>(ctx->last.mesh_vertex_count * 2);
    jdoubleArray out = env->NewDoubleArray(n);
    if (!out) return nullptr;
    jdouble values[GEMINI_EIS_MESH_VERTEX_COUNT * 2];
    for (uint32_t i = 0; i < ctx->last.mesh_vertex_count; ++i) {
        values[i * 2] = ctx->last.mesh[i].u;
        values[i * 2 + 1] = ctx->last.mesh[i].v;
    }
    env->SetDoubleArrayRegion(out, 0, n, values);
    return out;
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_yuy_geminieis_GeminiEisNative_nativeGetBuildId(JNIEnv* env, jclass) {
    return env->NewStringUTF(GeminiEisGetBuildId());
}
