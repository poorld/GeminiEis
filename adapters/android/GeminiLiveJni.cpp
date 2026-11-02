#include <jni.h>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include "api/gemini_eis_c_api.h"

namespace {

struct JniSession {
    std::mutex mtx;
    GeminiEisHandle handle = nullptr;
    GeminiEisConfig cfg{};
    GeminiEisResult lastResult{};
    bool hasValidMesh = false;
    int64_t lastSolvedSensorTsNs = 0;
    int64_t lastPushedGyroTsNs = 0;
    int orientationDeg = 180;
};

void fillIdentityMesh(GeminiEisResult* res) {
    for (uint32_t r = 0; r < GEMINI_EIS_MESH_ROWS_VERTS; ++r) {
        for (uint32_t c = 0; c < GEMINI_EIS_MESH_COLS_VERTS; ++c) {
            const uint32_t idx = r * GEMINI_EIS_MESH_COLS_VERTS + c;
            res->mesh[idx].u = static_cast<double>(c) / (GEMINI_EIS_MESH_COLS_VERTS - 1);
            res->mesh[idx].v = static_cast<double>(r) / (GEMINI_EIS_MESH_ROWS_VERTS - 1);
        }
    }
    res->mesh_vertex_count = GEMINI_EIS_MESH_VERTEX_COUNT;
}

} // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_yuy_y3realtime_LiveNativeSession_nativeCreate(
    JNIEnv* env,
    jclass /*clazz*/,
    jint inputW,
    jint inputH,
    jint outputW,
    jint outputH,
    jdouble fx,
    jdouble fy,
    jdouble cx,
    jdouble cy,
    jlong readoutNs,
    jlong camToGyroOffsetNs,
    jdoubleArray gyroToCam9,
    jint orientationDeg) {

    auto* sess = new (std::nothrow) JniSession();
    if (!sess) return 0;

    GeminiEisDefaultConfig(&sess->cfg);
    sess->cfg.input_width = static_cast<uint32_t>(inputW);
    sess->cfg.input_height = static_cast<uint32_t>(inputH);
    sess->cfg.output_width = static_cast<uint32_t>(outputW);
    sess->cfg.output_height = static_cast<uint32_t>(outputH);
    sess->cfg.fx = fx;
    sess->cfg.fy = fy;
    sess->cfg.cx = cx;
    sess->cfg.cy = cy;
    sess->cfg.default_readout_ns = readoutNs > 0 ? readoutNs : 29000000LL;
    sess->cfg.camera_to_gyro_offset_ns = camToGyroOffsetNs;
    sess->cfg.default_zoom = 1.12; // Constant Tripod Zoom 1.12x
    sess->orientationDeg = orientationDeg;
    fillIdentityMesh(&sess->lastResult);

    if (gyroToCam9 && env->GetArrayLength(gyroToCam9) >= 9) {
        jdouble buf[9]{};
        env->GetDoubleArrayRegion(gyroToCam9, 0, 9, buf);
        for (int i = 0; i < 9; ++i) sess->cfg.gyro_to_camera[i] = buf[i];
    }

    if (GeminiEisInit(&sess->handle, &sess->cfg) != GEMINI_EIS_OK || !sess->handle) {
        delete sess;
        return 0;
    }
    return reinterpret_cast<jlong>(sess);
}

JNIEXPORT void JNICALL
Java_com_yuy_y3realtime_LiveNativeSession_nativeDestroy(
    JNIEnv* /*env*/,
    jclass /*clazz*/,
    jlong ptr) {
    auto* sess = reinterpret_cast<JniSession*>(ptr);
    if (!sess) return;
    {
        std::lock_guard<std::mutex> lock(sess->mtx);
        GeminiEisUninit(&sess->handle);
    }
    delete sess;
}

JNIEXPORT void JNICALL
Java_com_yuy_y3realtime_LiveNativeSession_nativeReset(
    JNIEnv* /*env*/,
    jclass /*clazz*/,
    jlong ptr) {
    auto* sess = reinterpret_cast<JniSession*>(ptr);
    if (!sess) return;
    std::lock_guard<std::mutex> lock(sess->mtx);
    GeminiEisReset(&sess->handle);
    sess->hasValidMesh = false;
    sess->lastSolvedSensorTsNs = 0;
    fillIdentityMesh(&sess->lastResult);
}

JNIEXPORT void JNICALL
Java_com_yuy_y3realtime_LiveNativeSession_nativeSetOrientationDeg(
    JNIEnv* /*env*/,
    jclass /*clazz*/,
    jlong ptr,
    jint orientationDeg) {
    auto* sess = reinterpret_cast<JniSession*>(ptr);
    if (!sess) return;
    std::lock_guard<std::mutex> lock(sess->mtx);
    sess->orientationDeg = orientationDeg;
}

JNIEXPORT jlong JNICALL
Java_com_yuy_y3realtime_LiveNativeSession_nativeGetLastGyroTimestampNs(
    JNIEnv* /*env*/,
    jclass /*clazz*/,
    jlong ptr) {
    auto* sess = reinterpret_cast<JniSession*>(ptr);
    if (!sess) return 0;
    std::lock_guard<std::mutex> lock(sess->mtx);
    return sess->lastPushedGyroTsNs;
}

JNIEXPORT jint JNICALL
Java_com_yuy_y3realtime_LiveNativeSession_nativePushGyroBatch(
    JNIEnv* env,
    jclass /*clazz*/,
    jlong ptr,
    jlongArray timestampsNs,
    jfloatArray omegaXyzRadS,
    jint count) {
    auto* sess = reinterpret_cast<JniSession*>(ptr);
    if (!sess || !timestampsNs || !omegaXyzRadS || count <= 0) return GEMINI_EIS_EINVAL;

    std::vector<jlong> ts(static_cast<size_t>(count));
    std::vector<jfloat> w(static_cast<size_t>(count * 3));
    env->GetLongArrayRegion(timestampsNs, 0, count, ts.data());
    env->GetFloatArrayRegion(omegaXyzRadS, 0, count * 3, w.data());

    std::vector<GeminiEisGyroSample> batch(static_cast<size_t>(count));
    int64_t maxTs = 0;
    for (int i = 0; i < count; ++i) {
        batch[static_cast<size_t>(i)].timestamp_ns = ts[static_cast<size_t>(i)];
        batch[static_cast<size_t>(i)].x_rad_s = w[static_cast<size_t>(i * 3 + 0)];
        batch[static_cast<size_t>(i)].y_rad_s = w[static_cast<size_t>(i * 3 + 1)];
        batch[static_cast<size_t>(i)].z_rad_s = w[static_cast<size_t>(i * 3 + 2)];
        if (ts[static_cast<size_t>(i)] > maxTs) maxTs = ts[static_cast<size_t>(i)];
    }

    std::lock_guard<std::mutex> lock(sess->mtx);
    if (maxTs > sess->lastPushedGyroTsNs) {
        sess->lastPushedGyroTsNs = maxTs;
    }
    return GeminiEisPushGyro(&sess->handle, batch.data(), count);
}

JNIEXPORT jint JNICALL
Java_com_yuy_y3realtime_LiveNativeSession_nativeBuildGlesIndices(
    JNIEnv* env,
    jclass /*clazz*/,
    jobject outDirectShortBuffer3648) {
    if (!outDirectShortBuffer3648) return GEMINI_EIS_EINVAL;
    auto* dst = static_cast<uint16_t*>(env->GetDirectBufferAddress(outDirectShortBuffer3648));
    const jlong capElements = env->GetDirectBufferCapacity(outDirectShortBuffer3648);
    if (!dst || capElements < static_cast<jlong>(GEMINI_EIS_GLES_INDEX_COUNT)) {
        return GEMINI_EIS_EINVAL;
    }
    return GeminiEisBuildGlesIndices(dst);
}

JNIEXPORT jint JNICALL
Java_com_yuy_y3realtime_LiveNativeSession_nativeProcessFrameToGlesMesh(
    JNIEnv* env,
    jclass /*clazz*/,
    jlong ptr,
    jobject yPlaneDirectBuffer,
    jint yWidth,
    jint yHeight,
    jint yRowStride,
    jlong sensorTsNs,
    jlong exposureNs,
    jlong readoutNs,
    jfloatArray stMatrix16,
    jobject outVertexFloatBuf) {

    auto* sess = reinterpret_cast<JniSession*>(ptr);
    if (!sess || !outVertexFloatBuf) return GEMINI_EIS_EINVAL;

    auto* outFloats = static_cast<float*>(env->GetDirectBufferAddress(outVertexFloatBuf));
    const jlong capElements = env->GetDirectBufferCapacity(outVertexFloatBuf);
    if (!outFloats || capElements < static_cast<jlong>(GEMINI_EIS_MESH_VERTEX_COUNT * 4)) {
        return GEMINI_EIS_EINVAL;
    }

    const uint8_t* yPtr = nullptr;
    if (yPlaneDirectBuffer) {
        yPtr = static_cast<const uint8_t*>(env->GetDirectBufferAddress(yPlaneDirectBuffer));
    }

    float stMat[16]{};
    const float* stMatPtr = nullptr;
    if (stMatrix16 && env->GetArrayLength(stMatrix16) >= 16) {
        env->GetFloatArrayRegion(stMatrix16, 0, 16, stMat);
        stMatPtr = stMat;
    }

    std::lock_guard<std::mutex> lock(sess->mtx);

    // P1 Fix: Do not re-step stateful filters if called with duplicate or older timestamp
    if (sess->hasValidMesh && sensorTsNs > 0 && sensorTsNs <= sess->lastSolvedSensorTsNs) {
        GeminiEisExportGlesMesh(&sess->lastResult, stMatPtr, sess->orientationDeg, outFloats);
        return GEMINI_EIS_OK;
    }

    GeminiEisFrameInfo fi{};
    fi.sensor_timestamp_ns = sensorTsNs;
    fi.exposure_time_ns = exposureNs > 0 ? exposureNs : 10000000LL;
    fi.rolling_shutter_skew_ns = readoutNs > 0 ? readoutNs : sess->cfg.default_readout_ns;
    fi.zoom = 1.12;

    GeminiEisResult candidateResult{};
    int rc = GEMINI_EIS_OK;
    if (yPtr && yWidth > 0 && yHeight > 0 && yRowStride >= yWidth) {
        rc = GeminiEisProcessLiveYPlane(&sess->handle, &fi, yPtr, yWidth, yHeight, yRowStride, &candidateResult);
    } else {
        rc = GeminiEisProcessFrame(&sess->handle, &fi, &candidateResult);
    }

    if (rc == GEMINI_EIS_OK && candidateResult.valid) {
        sess->lastResult = candidateResult;
        sess->hasValidMesh = true;
        sess->lastSolvedSensorTsNs = sensorTsNs;
    } else if (!sess->hasValidMesh) {
        // Only use identity mesh during initial warmup before any valid frame is solved
        fillIdentityMesh(&sess->lastResult);
    }
    // P1 Fix: If rc != GEMINI_EIS_OK after warmup (e.g. transient ECOVERAGE),
    // sess->lastResult is preserved (smooth pose hold) rather than snapping to Identity Mesh!

    GeminiEisExportGlesMesh(&sess->lastResult, stMatPtr, sess->orientationDeg, outFloats);
    return rc;
}

} // extern "C"
