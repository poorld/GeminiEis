package com.yuy.y3realtime;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

/**
 * Unified JNI Bridge to libgemini_eis_jni.so (sharing exact C ABI with MTK HAL3 WPE).
 */
public final class LiveNativeSession {
    static {
        System.loadLibrary("gemini_eis_jni");
    }

    private long nativePtr = 0;

    public boolean init(
            int inputW,
            int inputH,
            int outputW,
            int outputH,
            double fx,
            double fy,
            double cx,
            double cy,
            long readoutNs,
            long camToGyroOffsetNs,
            double[] gyroToCam9,
            int orientationDeg) {
        release();
        nativePtr = nativeCreate(
                inputW, inputH, outputW, outputH,
                fx, fy, cx, cy,
                readoutNs, camToGyroOffsetNs, gyroToCam9, orientationDeg);
        return nativePtr != 0;
    }

    public void reset() {
        if (nativePtr != 0) {
            nativeReset(nativePtr);
        }
    }

    public void setOrientationDeg(int orientationDeg) {
        if (nativePtr != 0) {
            nativeSetOrientationDeg(nativePtr, orientationDeg);
        }
    }

    public long getLastGyroTimestampNs() {
        if (nativePtr == 0) return 0L;
        return nativeGetLastGyroTimestampNs(nativePtr);
    }

    public int pushGyroBatch(long[] timestampsNs, float[] omegaXyzRadS, int count) {
        if (nativePtr == 0) return -1;
        return nativePushGyroBatch(nativePtr, timestampsNs, omegaXyzRadS, count);
    }

    public int processFrameToGlesMesh(
            ByteBuffer yPlaneDirectBuffer,
            int yWidth,
            int yHeight,
            int yRowStride,
            long sensorTsNs,
            long exposureNs,
            long readoutNs,
            float[] stMatrix16,
            FloatBuffer outVertexFloatBuf) {
        if (nativePtr == 0) return -1;
        return nativeProcessFrameToGlesMesh(
                nativePtr,
                yPlaneDirectBuffer,
                yWidth,
                yHeight,
                yRowStride,
                sensorTsNs,
                exposureNs,
                readoutNs,
                stMatrix16,
                outVertexFloatBuf);
    }

    public long getNativeHandle() {
        return nativePtr;
    }

    public void release() {
        if (nativePtr != 0) {
            nativeDestroy(nativePtr);
            nativePtr = 0;
        }
    }

    public static native long nativeCreate(
            int inputW,
            int inputH,
            int outputW,
            int outputH,
            double fx,
            double fy,
            double cx,
            double cy,
            long readoutNs,
            long camToGyroOffsetNs,
            double[] gyroToCam9,
            int orientationDeg);

    public static native void nativeDestroy(long ptr);

    public static native void nativeReset(long ptr);

    public static native void nativeSetOrientationDeg(long ptr, int orientationDeg);

    public static native long nativeGetLastGyroTimestampNs(long ptr);

    public static native int nativePushGyroBatch(
            long ptr,
            long[] timestampsNs,
            float[] omegaXyzRadS,
            int count);

    public static native int nativeBuildGlesIndices(ShortBuffer outDirectShortBuffer3648);

    public static native int nativeProcessFrameToGlesMesh(
            long ptr,
            ByteBuffer yPlaneDirectBuffer,
            int yWidth,
            int yHeight,
            int yRowStride,
            long sensorTsNs,
            long exposureNs,
            long readoutNs,
            float[] stMatrix16,
            FloatBuffer outVertexFloatBuf);
}
