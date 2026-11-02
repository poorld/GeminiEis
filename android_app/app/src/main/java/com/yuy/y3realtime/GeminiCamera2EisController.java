package com.yuy.y3realtime;

import android.annotation.SuppressLint;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.media.MediaMuxer;
import android.opengl.EGL14;
import android.opengl.EGLConfig;
import android.opengl.EGLContext;
import android.opengl.EGLDisplay;
import android.opengl.EGLExt;
import android.opengl.EGLSurface;
import android.opengl.GLES11Ext;
import android.opengl.GLES30;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.view.Surface;
import android.view.WindowManager;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.microedition.khronos.opengles.GL10;

/**
 * End-to-End Real-Time Causal Camera2 + Gyroscope + Y-Plane 8DoF/RBF + GLES 3.0 ONE-WARP Pipeline (Golden Release).
 *
 * Key Guarantees:
 *   1. OES SurfaceTexture.getTimestamp() (oesTsNs) is the authoritative display & recording frame identity.
 *   2. CaptureResult metadata and downsampled 640x360 Y-Plane are matched by exact oesTsNs (tolerance <= 3.0ms).
 *   3. Default upright preview orientation on Y1 (AR_6769) is 270 deg (cameraSensorOrientation - displayRotation + manualOffset),
 *      and any user rotation adjustment (+90 deg) is persisted in SharedPreferences ("gemini_eis_prefs").
 *   4. Supports zero-copy hardware H.264 MP4 recording of the stabilized ONE-WARP OpenGL ES 3.0 mesh output via EGL14 + MediaCodec + MediaMuxer.
 */
public final class GeminiCamera2EisController implements GLSurfaceView.Renderer, SensorEventListener {
    private static final String TAG = "GeminiEIS";
    private static final long TS_MATCH_TOLERANCE_NS = 3_000_000L; // 3.0ms same-frame tolerance
    private static final String PREFS_NAME = "gemini_eis_prefs";
    private static final String PREF_KEY_OFFSET_DEG = "preview_manual_offset_deg_v2";

    public interface TelemetryListener {
        void onFrameTelemetry(
                double solveMs,
                int status,
                long frameIdx,
                boolean eisEnabled,
                long yMatchCount,
                long yMissCount,
                long capMatchCount,
                double gyroLeadMs,
                int effectiveRotDeg,
                boolean isRecording,
                long recordedFrames,
                String recordStatusText);
    }

    // Exact Y1 (AR_6769) verified gyroscope-to-camera extrinsic matrix from YuyEIS2-pc-reference-Y3
    public static final double[] Y1_SENSOR_TO_CAMERA = new double[]{
        0.0, 1.0,  0.0,
        1.0, 0.0,  0.0,
        0.0, 0.0, -1.0
    };
    public static final long Y1_CAMERA_TO_GYRO_OFFSET_NS = 4_000_000L; // Calibrated +4.0ms camera-to-gyro offset on Y1
    public static final double Y1_BASE_FX_2560 = 972.0;               // Calibrated raw-sensor optical fx on Y1 2560x1440
    public static final double Y1_BASE_FY_1440 = 976.0;               // Calibrated raw-sensor optical fy on Y1 2560x1440

    private static final int DEFAULT_SENSOR_W = 2560;
    private static final int DEFAULT_SENSOR_H = 1440;
    private static final int OUTPUT_W = 1920;
    private static final int OUTPUT_H = 1080;
    private static final int Y_TRACK_W = 640;
    private static final int Y_TRACK_H = 360;

    // Timestamp-indexed ring buffer for CaptureResult metadata
    private static final int META_RING_SIZE = 24;
    private static final class CaptureMetaSlot {
        long sensorTsNs = 0;
        long exposureNs = 10_000_000L;
        long readoutNs = 29_000_000L;
    }
    private final CaptureMetaSlot[] metaRing = new CaptureMetaSlot[META_RING_SIZE];
    private int metaWriteIdx = 0;
    private long latestCaptureTsNs = 0;
    private long lastKnownExposureNs = 28_000_000L; // Realistic indoor 30fps default exposure (~28ms) when CaptureResult lags
    private long lastKnownReadoutNs = 29_190_000L;  // Exact Y1 IMX476 rolling shutter skew (29.19ms)

    // Timestamp-indexed ring buffer for YUV_420_888 Y-plane frames
    private static final int Y_RING_SIZE = 8;
    private static final class YPlaneSlot {
        long sensorTsNs = 0;
        ByteBuffer yBuf = ByteBuffer.allocateDirect(Y_TRACK_W * Y_TRACK_H * 2).order(ByteOrder.nativeOrder());
        int width = 0;
        int height = 0;
        int rowStride = 0;
    }
    private final YPlaneSlot[] yRing = new YPlaneSlot[Y_RING_SIZE];
    private int yWriteIdx = 0;
    private long latestYTsNs = 0;

    private final Object syncRingLock = new Object();

    // Same-frame synchronization diagnostic counters
    private long yMatchedFrames = 0;
    private long yMissedFrames = 0;
    private long capMatchedFrames = 0;
    private long capMissedFrames = 0;
    private long duplicateOesSkipped = 0;
    private long coverageHoldCount = 0;

    private final Context context;
    private final SharedPreferences prefs;
    private final GLSurfaceView glSurfaceView;
    private final TelemetryListener telemetryListener;

    private final LiveNativeSession nativeSession = new LiveNativeSession();
    private final GeminiGlesMeshRenderer meshRenderer = new GeminiGlesMeshRenderer();
    private final AtomicBoolean eisEnabled = new AtomicBoolean(true);
    // Tracks bypass transitions on the GL thread.
    private boolean eisWasEnabledOnGlThread = true;

    private int activeSensorW = DEFAULT_SENSOR_W;
    private int activeSensorH = DEFAULT_SENSOR_H;
    private int cameraSensorOrientation = 270;
    private int manualPreviewOffsetDeg = 0;

    private int oesTextureId = 0;
    private SurfaceTexture oesSurfaceTexture;
    private Surface oesSurface;
    private final float[] stMatrix = new float[16];
    private volatile boolean frameAvailable = false;
    private long lastSolvedOesTsNs = 0;
    private float lastCenterU = -1f;
    private float lastCenterV = -1f;
    private double windowMaxStepPx = 0.0;

    private int viewportW = OUTPUT_W;
    private int viewportH = OUTPUT_H;
    private long frameCounter = 0;

    // Hardware H.264 MP4 Video Recorder state (owned by GL thread)
    private final AtomicBoolean recordingRequested = new AtomicBoolean(false);
    private boolean encoderActive = false;
    private MediaCodec videoEncoder;
    private MediaMuxer mediaMuxer;
    private boolean muxerStarted = false;
    private int videoTrackIndex = -1;
    private Surface encoderInputSurface;
    private EGLSurface encoderEglSurface = EGL14.EGL_NO_SURFACE;
    private int recordWidth = 1080;
    private int recordHeight = 1920;
    private long recordedFrameCount = 0;
    private long lastEncodedPtsUs = -1;
    private String lastRecordStatusText = "";
    private final MediaCodec.BufferInfo encoderBufferInfo = new MediaCodec.BufferInfo();

    // Telemetry ring for latency stats (30-frame window)
    private final double[] solveMsWindow = new double[30];
    private int windowIdx = 0;
    private long windowStartNs = 0;

    // Gyro batch ring buffer
    private SensorManager sensorManager;
    private Sensor gyroSensor;
    private static final int GYRO_BATCH_CAP = 32;
    private final long[] gyroTsBatch = new long[GYRO_BATCH_CAP];
    private final float[] gyroOmegaBatch = new float[GYRO_BATCH_CAP * 3];
    private int gyroBatchCount = 0;
    private long totalGyroSamples = 0;
    private long lastGyroTsNs = 0;
    private volatile int estimatedGyroHz = 0;
    private final Object gyroLock = new Object();

    // Camera2 background threads
    private HandlerThread camThread;
    private Handler camHandler;
    private HandlerThread yuvThread;
    private Handler yuvHandler;
    private HandlerThread gyroThread;
    private Handler gyroHandler;
    private CameraDevice cameraDevice;
    private CameraCaptureSession captureSession;
    private ImageReader yuvReader;

    public GeminiCamera2EisController(Context ctx, GLSurfaceView glView, TelemetryListener listener) {
        this.context = ctx;
        this.glSurfaceView = glView;
        this.telemetryListener = listener;
        this.prefs = ctx.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        // Load persisted manual preview offset (defaults to 0 deg, which yields 270 deg upright on Y1!)
        int savedOffset = prefs.getInt(PREF_KEY_OFFSET_DEG, 0);
        this.manualPreviewOffsetDeg = ((savedOffset % 360) + 360) % 360;
        for (int i = 0; i < META_RING_SIZE; ++i) metaRing[i] = new CaptureMetaSlot();
        for (int i = 0; i < Y_RING_SIZE; ++i) yRing[i] = new YPlaneSlot();
    }

    public void setEisEnabled(boolean enabled) {
        eisEnabled.set(enabled);
        glSurfaceView.requestRender();
    }

    public boolean isEisEnabled() {
        return eisEnabled.get();
    }

    public boolean toggleRecording() {
        boolean next = !recordingRequested.get();
        recordingRequested.set(next);
        glSurfaceView.requestRender();
        return next;
    }

    public boolean isRecording() {
        return recordingRequested.get();
    }

    public int getEffectivePreviewRotationDeg() {
        return computeEffectivePreviewRotationDeg();
    }

    /**
     * Cycles preview orientation by +90 deg, persists to SharedPreferences, and resets anchor.
     * @return The new effective preview orientation in degrees (0, 90, 180, 270).
     */
    public int cyclePreviewOrientation() {
        manualPreviewOffsetDeg = (manualPreviewOffsetDeg + 90) % 360;
        prefs.edit().putInt(PREF_KEY_OFFSET_DEG, manualPreviewOffsetDeg).apply();
        final int effRot = computeEffectivePreviewRotationDeg();
        glSurfaceView.queueEvent(() -> {
            nativeSession.setOrientationDeg(effRot);
            nativeSession.reset();
        });
        glSurfaceView.requestRender();
        Log.i(TAG, String.format(Locale.US,
                "Preview orientation updated & persisted: manualOffset=%d deg -> effectiveRot=%d deg",
                manualPreviewOffsetDeg, effRot));
        return effRot;
    }

    private int displayRotationDegrees() {
        WindowManager wm = (WindowManager) context.getSystemService(Context.WINDOW_SERVICE);
        if (wm == null || wm.getDefaultDisplay() == null) return 0;
        switch (wm.getDefaultDisplay().getRotation()) {
            case Surface.ROTATION_90: return 90;
            case Surface.ROTATION_180: return 180;
            case Surface.ROTATION_270: return 270;
            default: return 0;
        }
    }

    private int computeEffectivePreviewRotationDeg() {
        // On Y1 (AR_6769), cameraSensorOrientation = 270 and displayRotation = 0.
        // Without the old +180 inversion, (270 - 0 + 0) % 360 = 270 deg, which is upright out-of-the-box!
        int deg = (cameraSensorOrientation - displayRotationDegrees() + manualPreviewOffsetDeg) % 360;
        return deg < 0 ? deg + 360 : deg;
    }

    public void start() {
        camThread = new HandlerThread("GeminiEisCameraThread");
        camThread.start();
        camHandler = new Handler(camThread.getLooper());

        yuvThread = new HandlerThread("GeminiEisYuvThread");
        yuvThread.start();
        yuvHandler = new Handler(yuvThread.getLooper());

        gyroThread = new HandlerThread("GeminiEisGyroThread");
        gyroThread.start();
        gyroHandler = new Handler(gyroThread.getLooper());

        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager != null) {
            gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
            if (gyroSensor == null) {
                gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            }
            if (gyroSensor != null) {
                sensorManager.registerListener(this, gyroSensor, SensorManager.SENSOR_DELAY_FASTEST, gyroHandler);
                Log.i(TAG, "Gyroscope registered: " + gyroSensor.getName() + " on device=" + Build.DEVICE);
            }
        }
    }

    public void stop() {
        recordingRequested.set(false);
        glSurfaceView.queueEvent(this::stopVideoEncoderOnGlThread);
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
        closeCamera();
        if (camThread != null) {
            camThread.quitSafely();
            try { camThread.join(1000); } catch (InterruptedException ignored) {}
            camThread = null;
            camHandler = null;
        }
        if (yuvThread != null) {
            yuvThread.quitSafely();
            try { yuvThread.join(1000); } catch (InterruptedException ignored) {}
            yuvThread = null;
            yuvHandler = null;
        }
        if (gyroThread != null) {
            gyroThread.quitSafely();
            try { gyroThread.join(1000); } catch (InterruptedException ignored) {}
            gyroThread = null;
            gyroHandler = null;
        }
        nativeSession.release();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() != Sensor.TYPE_GYROSCOPE &&
            event.sensor.getType() != Sensor.TYPE_GYROSCOPE_UNCALIBRATED) {
            return;
        }
        float wx = event.values[0];
        float wy = event.values[1];
        float wz = event.values[2];
        if (event.values.length >= 6) {
            wx -= event.values[3];
            wy -= event.values[4];
            wz -= event.values[5];
        }
        if (lastGyroTsNs > 0 && event.timestamp > lastGyroTsNs) {
            int instHz = (int) (1_000_000_000L / (event.timestamp - lastGyroTsNs));
            estimatedGyroHz = (estimatedGyroHz * 7 + Math.min(instHz, 1000)) / 8;
        }
        lastGyroTsNs = event.timestamp;

        synchronized (gyroLock) {
            totalGyroSamples++;
            int idx = gyroBatchCount;
            if (idx < GYRO_BATCH_CAP) {
                gyroTsBatch[idx] = event.timestamp;
                gyroOmegaBatch[idx * 3] = wx;
                gyroOmegaBatch[idx * 3 + 1] = wy;
                gyroOmegaBatch[idx * 3 + 2] = wz;
                gyroBatchCount++;
            }
            if (gyroBatchCount >= 4) {
                nativeSession.pushGyroBatch(gyroTsBatch, gyroOmegaBatch, gyroBatchCount);
                gyroBatchCount = 0;
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    private void initOrReinitNativeSession(int srcW, int srcH, int sensorOrientDeg) {
        activeSensorW = srcW;
        activeSensorH = srcH;
        cameraSensorOrientation = sensorOrientDeg;
        int effRot = computeEffectivePreviewRotationDeg();
        double fx = Y1_BASE_FX_2560 * (srcW / 2560.0);
        double fy = Y1_BASE_FY_1440 * (srcH / 1440.0);
        double cx = srcW * 0.5;
        double cy = srcH * 0.5;
        boolean ok = nativeSession.init(
                srcW, srcH, OUTPUT_W, OUTPUT_H,
                fx, fy, cx, cy,
                29_000_000L, Y1_CAMERA_TO_GYRO_OFFSET_NS, Y1_SENSOR_TO_CAMERA, effRot);
        Log.i(TAG, String.format(Locale.US,
                "NativeSession init=%b device=%s src=%dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f offsetNs=%d sensorOrient=%d manualOffset=%d effRot=%d",
                ok, Build.DEVICE, srcW, srcH, fx, fy, cx, cy, Y1_CAMERA_TO_GYRO_OFFSET_NS, sensorOrientDeg, manualPreviewOffsetDeg, effRot));
    }

    @Override
    public void onSurfaceCreated(GL10 gl, javax.microedition.khronos.egl.EGLConfig config) {
        initOrReinitNativeSession(DEFAULT_SENSOR_W, DEFAULT_SENSOR_H, 270);
        meshRenderer.initGl(nativeSession.getNativeHandle());
        oesTextureId = createOesTexture();
        oesSurfaceTexture = new SurfaceTexture(oesTextureId);
        oesSurfaceTexture.setDefaultBufferSize(activeSensorW, activeSensorH);
        oesSurfaceTexture.setOnFrameAvailableListener(st -> {
            frameAvailable = true;
            glSurfaceView.requestRender();
        }, new Handler(Looper.getMainLooper()));
        oesSurface = new Surface(oesSurfaceTexture);

        if (camHandler != null) {
            camHandler.post(this::openCamera);
        }
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        this.viewportW = width;
        this.viewportH = height;
        nativeSession.setOrientationDeg(computeEffectivePreviewRotationDeg());
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        if (oesSurfaceTexture == null) return;
        if (frameAvailable) {
            frameAvailable = false;
            oesSurfaceTexture.updateTexImage();
            oesSurfaceTexture.getTransformMatrix(stMatrix);
        }

        // Handle start/stop recording transitions on GL thread
        boolean wantRec = recordingRequested.get();
        if (wantRec && !encoderActive) {
            setupVideoEncoderOnGlThread();
        } else if (!wantRec && encoderActive) {
            stopVideoEncoderOnGlThread();
        }

        final long oesTsNs = oesSurfaceTexture.getTimestamp();
        if (oesTsNs <= 0) return;

        // If onDrawFrame was called without a new OES camera frame, draw cached VBO directly
        if (oesTsNs == lastSolvedOesTsNs && frameCounter > 0) {
            duplicateOesSkipped++;
            meshRenderer.uploadMeshAndDraw(oesTextureId, viewportW, viewportH);
            return;
        }

        // Flush any pending gyro samples before solving frame
        long latestGyroNs;
        synchronized (gyroLock) {
            if (gyroBatchCount > 0) {
                nativeSession.pushGyroBatch(gyroTsBatch, gyroOmegaBatch, gyroBatchCount);
                gyroBatchCount = 0;
            }
            latestGyroNs = lastGyroTsNs;
        }

        long t0 = System.nanoTime();
        int rc = 0;
        long fIdx = ++frameCounter;
        FloatBuffer vboBuf = meshRenderer.getDirectVertexBuffer();

        // Look up exact same-frame CaptureResult and Y-Plane by oesTsNs
        long matchedCapTsNs = 0;
        long matchedYTsNs = 0;
        long expNs = lastKnownExposureNs;
        long skewNs = lastKnownReadoutNs;
        ByteBuffer matchedYBuf = null;
        int matchedYW = 0, matchedYH = 0, matchedYStride = 0;

        synchronized (syncRingLock) {
            long waitDeadlineMs = System.currentTimeMillis() + 5;
            while ((latestYTsNs < oesTsNs - TS_MATCH_TOLERANCE_NS || latestCaptureTsNs < oesTsNs - TS_MATCH_TOLERANCE_NS)
                    && System.currentTimeMillis() < waitDeadlineMs) {
                try {
                    syncRingLock.wait(2);
                } catch (InterruptedException ignored) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
            long bestCapDiff = Long.MAX_VALUE;
            for (int i = 0; i < META_RING_SIZE; ++i) {
                CaptureMetaSlot s = metaRing[i];
                if (s.sensorTsNs > 0) {
                    long diff = Math.abs(s.sensorTsNs - oesTsNs);
                    if (diff <= TS_MATCH_TOLERANCE_NS && diff < bestCapDiff) {
                        bestCapDiff = diff;
                        matchedCapTsNs = s.sensorTsNs;
                        expNs = s.exposureNs;
                        skewNs = s.readoutNs;
                    }
                }
            }
            if (matchedCapTsNs != 0) {
                capMatchedFrames++;
            } else {
                capMissedFrames++;
            }

            long bestYDiff = Long.MAX_VALUE;
            for (int i = 0; i < Y_RING_SIZE; ++i) {
                YPlaneSlot ys = yRing[i];
                if (ys.sensorTsNs > 0 && ys.width > 0 && ys.height > 0) {
                    long diff = Math.abs(ys.sensorTsNs - oesTsNs);
                    if (diff <= TS_MATCH_TOLERANCE_NS && diff < bestYDiff) {
                        bestYDiff = diff;
                        matchedYTsNs = ys.sensorTsNs;
                        matchedYBuf = ys.yBuf;
                        matchedYW = ys.width;
                        matchedYH = ys.height;
                        matchedYStride = ys.rowStride;
                    }
                }
            }
            if (matchedYTsNs != 0) {
                yMatchedFrames++;
            } else {
                yMissedFrames++;
            }
        }

        final float[] texTransform = "AR_6769".equals(Build.DEVICE) ? null : stMatrix;
        final int effRotDeg = computeEffectivePreviewRotationDeg();

        final boolean solveEis = eisEnabled.get();
        if (solveEis) {
            if (!eisWasEnabledOnGlThread) {
                // OFF bypasses native frames, so the old timestamp would cause
                // a persistent ESTATE after a long pause. Restart the causal
                // solver before accepting the first new ON frame.
                nativeSession.reset();
                lastCenterU = -1f;
                lastCenterV = -1f;
                Log.i(TAG, "EIS OFF->ON: reset native causal frame state");
            }
            eisWasEnabledOnGlThread = true;
            rc = nativeSession.processFrameToGlesMesh(
                    matchedYBuf,
                    matchedYW,
                    matchedYH,
                    matchedYStride,
                    oesTsNs,
                    expNs,
                    skewNs,
                    texTransform,
                    vboBuf);
            if (rc != 0) {
                coverageHoldCount++;
            }
            lastSolvedOesTsNs = oesTsNs;
        } else {
            eisWasEnabledOnGlThread = false;
            populateIdentityMesh(vboBuf, texTransform, effRotDeg);
            lastSolvedOesTsNs = oesTsNs;
        }

        double jniSolveMs = (System.nanoTime() - t0) * 1e-6;
        int centerFloatIdx = (10 * GeminiGlesMeshRenderer.MESH_COLS + 16) * 4;
        float curU = vboBuf.get(centerFloatIdx + 2);
        float curV = vboBuf.get(centerFloatIdx + 3);
        if (lastCenterU >= 0f) {
            double stepPx = Math.hypot((curU - lastCenterU) * activeSensorW, (curV - lastCenterV) * activeSensorH);
            if (stepPx > windowMaxStepPx) windowMaxStepPx = stepPx;
        }
        lastCenterU = curU;
        lastCenterV = curV;

        // 1. Render stabilized mesh to screen preview
        meshRenderer.uploadMeshAndDraw(oesTextureId, viewportW, viewportH);

        // 2. If recording is active, render stabilized mesh in upright 1920x1080 landscape to MediaCodec EGLSurface
        if (encoderActive && encoderEglSurface != EGL14.EGL_NO_SURFACE) {
            renderFrameToEncoderOnGlThread(oesTsNs, vboBuf);
        }

        double gyroLeadMs = latestGyroNs > 0 ? (latestGyroNs - oesTsNs) * 1e-6 : 0.0;

        if (windowStartNs == 0) windowStartNs = System.nanoTime();
        solveMsWindow[windowIdx++] = jniSolveMs;
        if (windowIdx >= solveMsWindow.length) {
            double[] sorted = Arrays.copyOf(solveMsWindow, solveMsWindow.length);
            Arrays.sort(sorted);
            double sum = 0.0;
            for (double v : sorted) sum += v;
            double avgMs = sum / sorted.length;
            double p50Ms = sorted[sorted.length / 2];
            double p90Ms = sorted[(int) (sorted.length * 0.9)];
            double maxMs = sorted[sorted.length - 1];
            double elapsedS = Math.max(1e-3, (System.nanoTime() - windowStartNs) * 1e-9);
            double fps = sorted.length / elapsedS;
            long yDeltaUs = matchedYTsNs > 0 ? (matchedYTsNs - oesTsNs) / 1000L : -999999L;
            long capDeltaUs = matchedCapTsNs > 0 ? (matchedCapTsNs - oesTsNs) / 1000L : -999999L;
            Log.i(TAG, String.format(Locale.US,
                    "SYNC_PERF frame=%d fps=%.1f solve_avg=%.2fms p50=%.2fms p90=%.2fms max=%.2fms status=%d " +
                    "OES_TS=%d yMatch=%d/%d(dUs=%d) capMatch=%d/%d(dUs=%d) gyroLead=%.1fms hold=%d dupSkip=%d gyroHz=%d rot=%d centerUV=(%.4f,%.4f) maxStepPx=%.3f rec=%b(%d)",
                    fIdx, fps, avgMs, p50Ms, p90Ms, maxMs, rc,
                    oesTsNs, yMatchedFrames, (yMatchedFrames + yMissedFrames), yDeltaUs,
                    capMatchedFrames, (capMatchedFrames + capMissedFrames), capDeltaUs,
                    gyroLeadMs, coverageHoldCount, duplicateOesSkipped, estimatedGyroHz,
                    effRotDeg, curU, curV, windowMaxStepPx, encoderActive, recordedFrameCount));
            windowIdx = 0;
            windowMaxStepPx = 0.0;
            windowStartNs = System.nanoTime();
        }

        if (telemetryListener != null && (fIdx % 10 == 0)) {
            telemetryListener.onFrameTelemetry(
                    jniSolveMs, rc, fIdx, eisEnabled.get(),
                    yMatchedFrames, yMissedFrames, capMatchedFrames, gyroLeadMs,
                    effRotDeg, encoderActive, recordedFrameCount, lastRecordStatusText);
        }
    }

    private void setupVideoEncoderOnGlThread() {
        try {
            // Always record in native 1920x1080 (16:9 landscape) matching the 2560x1440 sensor aspect ratio
            recordWidth = OUTPUT_W;
            recordHeight = OUTPUT_H;

            File baseDir = context.getExternalFilesDir(Environment.DIRECTORY_MOVIES);
            if (baseDir == null) baseDir = context.getFilesDir();
            File outDir = new File(baseDir, "GeminiEIS");
            if (!outDir.exists()) outDir.mkdirs();

            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File outFile = new File(outDir, "GeminiEIS_" + stamp + ".mp4");

            MediaFormat format = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, recordWidth, recordHeight);
            format.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface);
            format.setInteger(MediaFormat.KEY_BIT_RATE, 12_000_000);
            format.setInteger(MediaFormat.KEY_FRAME_RATE, 30);
            format.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1);

            videoEncoder = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC);
            videoEncoder.configure(format, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);
            encoderInputSurface = videoEncoder.createInputSurface();
            videoEncoder.start();

            EGLDisplay eglDisplay = EGL14.eglGetCurrentDisplay();
            int[] attribList = new int[]{
                EGL14.EGL_RED_SIZE, 8,
                EGL14.EGL_GREEN_SIZE, 8,
                EGL14.EGL_BLUE_SIZE, 8,
                EGL14.EGL_ALPHA_SIZE, 8,
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                0x3142, 1, // EGL_RECORDABLE_ANDROID
                EGL14.EGL_NONE
            };
            EGLConfig[] configs = new EGLConfig[1];
            int[] numConfigs = new int[1];
            if (!EGL14.eglChooseConfig(eglDisplay, attribList, 0, configs, 0, 1, numConfigs, 0) || numConfigs[0] <= 0) {
                throw new RuntimeException("eglChooseConfig for encoder failed");
            }
            encoderEglSurface = EGL14.eglCreateWindowSurface(
                    eglDisplay, configs[0], encoderInputSurface, new int[]{EGL14.EGL_NONE}, 0);
            if (encoderEglSurface == null || encoderEglSurface == EGL14.EGL_NO_SURFACE) {
                throw new RuntimeException("eglCreateWindowSurface for encoder failed");
            }

            mediaMuxer = new MediaMuxer(outFile.getAbsolutePath(), MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4);
            muxerStarted = false;
            videoTrackIndex = -1;
            recordedFrameCount = 0;
            lastEncodedPtsUs = -1;
            encoderActive = true;
            lastRecordStatusText = "REC -> " + outFile.getName();
            Log.i(TAG, "Started stabilized MP4 recording: " + outFile.getAbsolutePath() + " (" + recordWidth + "x" + recordHeight + ")");
        } catch (Exception e) {
            Log.e(TAG, "Failed to start video encoder", e);
            recordingRequested.set(false);
            lastRecordStatusText = "REC ERR: " + e.getMessage();
            stopVideoEncoderOnGlThread();
        }
    }

    private static void rotateMeshNdc90InPlace(FloatBuffer vboBuf, boolean ccw) {
        final int totalFloats = GeminiGlesMeshRenderer.VERTEX_COUNT * 4;
        for (int idx = 0; idx < totalFloats; idx += 4) {
            float x = vboBuf.get(idx);
            float y = vboBuf.get(idx + 1);
            if (ccw) {
                vboBuf.put(idx, -y);
                vboBuf.put(idx + 1, x);
            } else {
                vboBuf.put(idx, y);
                vboBuf.put(idx + 1, -x);
            }
        }
        vboBuf.position(0);
    }

    private void renderFrameToEncoderOnGlThread(long oesTsNs, FloatBuffer vboBuf) {
        EGLDisplay eglDisplay = EGL14.eglGetCurrentDisplay();
        EGLContext eglContext = EGL14.eglGetCurrentContext();
        EGLSurface prevDraw = EGL14.eglGetCurrentSurface(EGL14.EGL_DRAW);
        EGLSurface prevRead = EGL14.eglGetCurrentSurface(EGL14.EGL_READ);
        // Rotate portrait preview NDC by 90 deg CCW (270 -> 180) so landscape recording (1920x1080) is upright
        rotateMeshNdc90InPlace(vboBuf, true);
        try {
            if (EGL14.eglMakeCurrent(eglDisplay, encoderEglSurface, encoderEglSurface, eglContext)) {
                meshRenderer.uploadMeshAndDraw(oesTextureId, recordWidth, recordHeight);
                EGLExt.eglPresentationTimeANDROID(eglDisplay, encoderEglSurface, oesTsNs);
                EGL14.eglSwapBuffers(eglDisplay, encoderEglSurface);
                recordedFrameCount++;
            }
        } catch (Exception e) {
            Log.e(TAG, "renderFrameToEncoderOnGlThread error", e);
        } finally {
            rotateMeshNdc90InPlace(vboBuf, false);
            EGL14.eglMakeCurrent(eglDisplay, prevDraw, prevRead, eglContext);
            GLES30.glViewport(0, 0, viewportW, viewportH);
        }
        drainEncoderOnGlThread(false);
    }

    private void drainEncoderOnGlThread(boolean endOfStream) {
        if (videoEncoder == null) return;
        try {
            if (endOfStream) {
                videoEncoder.signalEndOfInputStream();
            }
            while (true) {
                int outIdx = videoEncoder.dequeueOutputBuffer(encoderBufferInfo, endOfStream ? 10_000 : 0);
                if (outIdx == MediaCodec.INFO_TRY_AGAIN_LATER) {
                    if (!endOfStream) break;
                } else if (outIdx == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    if (!muxerStarted && mediaMuxer != null) {
                        MediaFormat newFormat = videoEncoder.getOutputFormat();
                        videoTrackIndex = mediaMuxer.addTrack(newFormat);
                        mediaMuxer.start();
                        muxerStarted = true;
                    }
                } else if (outIdx >= 0) {
                    ByteBuffer encodedData = videoEncoder.getOutputBuffer(outIdx);
                    if ((encoderBufferInfo.flags & MediaCodec.BUFFER_FLAG_CODEC_CONFIG) != 0) {
                        encoderBufferInfo.size = 0;
                    }
                    if (encoderBufferInfo.size > 0 && muxerStarted && encodedData != null) {
                        if (encoderBufferInfo.presentationTimeUs <= lastEncodedPtsUs) {
                            encoderBufferInfo.presentationTimeUs = lastEncodedPtsUs + 1000L;
                        }
                        lastEncodedPtsUs = encoderBufferInfo.presentationTimeUs;
                        encodedData.position(encoderBufferInfo.offset);
                        encodedData.limit(encoderBufferInfo.offset + encoderBufferInfo.size);
                        mediaMuxer.writeSampleData(videoTrackIndex, encodedData, encoderBufferInfo);
                    }
                    videoEncoder.releaseOutputBuffer(outIdx, false);
                    if ((encoderBufferInfo.flags & MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) {
                        break;
                    }
                } else {
                    break;
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "drainEncoderOnGlThread exception", e);
        }
    }

    private void stopVideoEncoderOnGlThread() {
        if (!encoderActive && videoEncoder == null && mediaMuxer == null) return;
        try {
            if (encoderActive && videoEncoder != null) {
                drainEncoderOnGlThread(true);
            }
        } catch (Exception ignored) {}

        EGLDisplay eglDisplay = EGL14.eglGetCurrentDisplay();
        if (encoderEglSurface != EGL14.EGL_NO_SURFACE && eglDisplay != EGL14.EGL_NO_DISPLAY) {
            try { EGL14.eglDestroySurface(eglDisplay, encoderEglSurface); } catch (Exception ignored) {}
            encoderEglSurface = EGL14.EGL_NO_SURFACE;
        }
        if (encoderInputSurface != null) {
            try { encoderInputSurface.release(); } catch (Exception ignored) {}
            encoderInputSurface = null;
        }
        if (videoEncoder != null) {
            try { videoEncoder.stop(); } catch (Exception ignored) {}
            try { videoEncoder.release(); } catch (Exception ignored) {}
            videoEncoder = null;
        }
        if (mediaMuxer != null) {
            if (muxerStarted) {
                try { mediaMuxer.stop(); } catch (Exception ignored) {}
            }
            try { mediaMuxer.release(); } catch (Exception ignored) {}
            mediaMuxer = null;
        }
        muxerStarted = false;
        videoTrackIndex = -1;
        if (encoderActive) {
            lastRecordStatusText = "SAVED (" + recordedFrameCount + "f)";
            Log.i(TAG, "Stopped MP4 recording: frames=" + recordedFrameCount);
        }
        encoderActive = false;
    }

    private static void populateIdentityMesh(FloatBuffer vboBuf, float[] stMat, int orientationDeg) {
        vboBuf.position(0);
        for (int r = 0; r < GeminiGlesMeshRenderer.MESH_ROWS; ++r) {
            float vScreen = (float) r / (GeminiGlesMeshRenderer.MESH_ROWS - 1);
            for (int c = 0; c < GeminiGlesMeshRenderer.MESH_COLS; ++c) {
                float uScreen = (float) c / (GeminiGlesMeshRenderer.MESH_COLS - 1);
                float ndcX = uScreen * 2.0f - 1.0f;
                float ndcY = 1.0f - vScreen * 2.0f;
                if (orientationDeg == 180) {
                    ndcX = -ndcX; ndcY = -ndcY;
                } else if (orientationDeg == 90) {
                    float tx = ndcX; ndcX = ndcY; ndcY = -tx;
                } else if (orientationDeg == 270) {
                    float tx = ndcX; ndcX = -ndcY; ndcY = tx;
                }
                float tu = uScreen;
                float tv = vScreen;
                if (stMat != null) {
                    tu = stMat[0] * uScreen + stMat[4] * vScreen + stMat[12];
                    tv = stMat[1] * uScreen + stMat[5] * vScreen + stMat[13];
                }
                vboBuf.put(ndcX).put(ndcY).put(tu).put(tv);
            }
        }
        vboBuf.position(0);
    }

    @SuppressLint("MissingPermission")
    private void openCamera() {
        try {
            CameraManager mgr = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
            if (mgr == null) return;
            String targetCamId = null;
            for (String id : mgr.getCameraIdList()) {
                CameraCharacteristics cc = mgr.getCameraCharacteristics(id);
                Integer facing = cc.get(CameraCharacteristics.LENS_FACING);
                if (facing != null && facing == CameraCharacteristics.LENS_FACING_BACK) {
                    if (targetCamId == null || "0".equals(id)) targetCamId = id;
                    if ("0".equals(id)) break;
                }
            }
            if (targetCamId == null && mgr.getCameraIdList().length > 0) {
                targetCamId = mgr.getCameraIdList()[0];
            }
            if (targetCamId == null) return;

            CameraCharacteristics cc = mgr.getCameraCharacteristics(targetCamId);
            Integer sensorOrient = cc.get(CameraCharacteristics.SENSOR_ORIENTATION);
            Integer tsSource = cc.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE);
            StreamConfigurationMap map = cc.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);

            Size chosenOes = new Size(DEFAULT_SENSOR_W, DEFAULT_SENSOR_H);
            if (map != null && map.getOutputSizes(SurfaceTexture.class) != null) {
                Size[] sizes = map.getOutputSizes(SurfaceTexture.class);
                boolean has2560 = false;
                Size best16x9 = null;
                Size bestAny = sizes[0];
                for (Size s : sizes) {
                    if (s.getWidth() == 2560 && s.getHeight() == 1440) {
                        has2560 = true;
                        break;
                    }
                    if (s.getWidth() <= 2560 && Math.abs(s.getWidth() * 9 - s.getHeight() * 16) <= 16) {
                        if (best16x9 == null || (s.getWidth() * s.getHeight() > best16x9.getWidth() * best16x9.getHeight())) {
                            best16x9 = s;
                        }
                    }
                    if (s.getWidth() <= 2560 && s.getWidth() * s.getHeight() > bestAny.getWidth() * bestAny.getHeight()) {
                        bestAny = s;
                    }
                }
                if (!has2560) {
                    chosenOes = (best16x9 != null) ? best16x9 : bestAny;
                }
            }

            final int selW = chosenOes.getWidth();
            final int selH = chosenOes.getHeight();
            final int orientDeg = (sensorOrient != null) ? sensorOrient : 270;
            Log.i(TAG, String.format(Locale.US,
                    "Camera2 id=%s device=%s chosenOes=%dx%d sensorOrient=%d tsSource=%s",
                    targetCamId, Build.DEVICE, selW, selH, orientDeg, String.valueOf(tsSource)));

            glSurfaceView.queueEvent(() -> {
                initOrReinitNativeSession(selW, selH, orientDeg);
                if (oesSurfaceTexture != null) {
                    oesSurfaceTexture.setDefaultBufferSize(selW, selH);
                }
            });

            yuvReader = ImageReader.newInstance(Y_TRACK_W, Y_TRACK_H, ImageFormat.YUV_420_888, 4);
            yuvReader.setOnImageAvailableListener(reader -> {
                Image img = reader.acquireLatestImage();
                if (img == null) return;
                try {
                    Image.Plane yPlane = img.getPlanes()[0];
                    ByteBuffer src = yPlane.getBuffer();
                    int w = img.getWidth();
                    int h = img.getHeight();
                    int stride = yPlane.getRowStride();
                    long imgTsNs = img.getTimestamp();
                    int rem = src.remaining();
                    synchronized (syncRingLock) {
                        YPlaneSlot slot = yRing[yWriteIdx];
                        yWriteIdx = (yWriteIdx + 1) % Y_RING_SIZE;
                        if (slot.yBuf.capacity() < rem) {
                            slot.yBuf = ByteBuffer.allocateDirect(rem).order(ByteOrder.nativeOrder());
                        }
                        slot.yBuf.clear();
                        slot.yBuf.put(src);
                        slot.yBuf.position(0);
                        slot.width = w;
                        slot.height = h;
                        slot.rowStride = stride;
                        slot.sensorTsNs = imgTsNs;
                        latestYTsNs = imgTsNs;
                        syncRingLock.notifyAll();
                    }
                } finally {
                    img.close();
                }
            }, yuvHandler != null ? yuvHandler : camHandler);

            mgr.openCamera(targetCamId, new CameraDevice.StateCallback() {
                @Override
                public void onOpened(CameraDevice camera) {
                    cameraDevice = camera;
                    createCaptureSession();
                }
                @Override
                public void onDisconnected(CameraDevice camera) {
                    camera.close();
                    cameraDevice = null;
                }
                @Override
                public void onError(CameraDevice camera, int error) {
                    Log.e(TAG, "CameraDevice onError=" + error);
                    camera.close();
                    cameraDevice = null;
                }
            }, camHandler);
        } catch (CameraAccessException e) {
            Log.e(TAG, "openCamera exception", e);
        }
    }

    private void createCaptureSession() {
        if (cameraDevice == null || oesSurface == null || yuvReader == null) return;
        try {
            final CaptureRequest.Builder reqBuilder =
                    cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
            reqBuilder.addTarget(oesSurface);
            reqBuilder.addTarget(yuvReader.getSurface());
            reqBuilder.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
                    CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF);
            reqBuilder.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
                    CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF);

            CameraManager mgr = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
            if (mgr != null) {
                CameraCharacteristics cc = mgr.getCameraCharacteristics(cameraDevice.getId());
                Range<Integer>[] ranges = cc.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
                if (ranges != null) {
                    for (Range<Integer> r : ranges) {
                        if (r.getLower() == 30 && r.getUpper() == 30) {
                            reqBuilder.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, r);
                            break;
                        }
                    }
                }
                // Enable Continuous Video Autofocus so fine details stay optically locked in focus
                int[] afModes = cc.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES);
                if (afModes != null) {
                    for (int mode : afModes) {
                        if (mode == CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO) {
                            reqBuilder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO);
                            break;
                        }
                    }
                }
                // Prefer faster shutter speed (-1 EV step when supported) to reduce intra-frame exposure motion blur during shake
                Range<Integer> aeCompRange = cc.get(CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE);
                if (aeCompRange != null && aeCompRange.contains(-1)) {
                    reqBuilder.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, -1);
                }
                reqBuilder.set(CaptureRequest.CONTROL_AE_ANTIBANDING_MODE, CaptureRequest.CONTROL_AE_ANTIBANDING_MODE_AUTO);
                reqBuilder.set(CaptureRequest.EDGE_MODE, CaptureRequest.EDGE_MODE_FAST);
                reqBuilder.set(CaptureRequest.NOISE_REDUCTION_MODE, CaptureRequest.NOISE_REDUCTION_MODE_FAST);
            }

            cameraDevice.createCaptureSession(
                    Arrays.asList(oesSurface, yuvReader.getSurface()),
                    new CameraCaptureSession.StateCallback() {
                        @Override
                        public void onConfigured(CameraCaptureSession session) {
                            captureSession = session;
                            try {
                                session.setRepeatingRequest(reqBuilder.build(),
                                        new CameraCaptureSession.CaptureCallback() {
                                            @Override
                                            public void onCaptureCompleted(
                                                    CameraCaptureSession s,
                                                    CaptureRequest request,
                                                    TotalCaptureResult result) {
                                                Long ts = result.get(CaptureResult.SENSOR_TIMESTAMP);
                                                Long exp = result.get(CaptureResult.SENSOR_EXPOSURE_TIME);
                                                Long skew = result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW);
                                                if (ts != null && ts > 0) {
                                                    synchronized (syncRingLock) {
                                                        CaptureMetaSlot slot = metaRing[metaWriteIdx];
                                                        metaWriteIdx = (metaWriteIdx + 1) % META_RING_SIZE;
                                                        slot.sensorTsNs = ts;
                                                        slot.exposureNs = (exp != null && exp > 0) ? exp : lastKnownExposureNs;
                                                        slot.readoutNs = (skew != null && skew > 0) ? skew : lastKnownReadoutNs;
                                                        lastKnownExposureNs = slot.exposureNs;
                                                        lastKnownReadoutNs = slot.readoutNs;
                                                        latestCaptureTsNs = ts;
                                                        syncRingLock.notifyAll();
                                                    }
                                                }
                                            }
                                        }, camHandler);
                                Log.i(TAG, "CameraCaptureSession repeating request started");
                            } catch (CameraAccessException e) {
                                Log.e(TAG, "setRepeatingRequest failed", e);
                            }
                        }
                        @Override
                        public void onConfigureFailed(CameraCaptureSession session) {
                            Log.e(TAG, "CameraCaptureSession onConfigureFailed");
                        }
                    }, camHandler);
        } catch (CameraAccessException e) {
            Log.e(TAG, "createCaptureSession failed", e);
        }
    }

    private void closeCamera() {
        if (captureSession != null) {
            captureSession.close();
            captureSession = null;
        }
        if (cameraDevice != null) {
            cameraDevice.close();
            cameraDevice = null;
        }
        if (yuvReader != null) {
            yuvReader.close();
            yuvReader = null;
        }
        if (oesSurface != null) {
            oesSurface.release();
            oesSurface = null;
        }
        if (oesSurfaceTexture != null) {
            oesSurfaceTexture.release();
            oesSurfaceTexture = null;
        }
    }

    private static int createOesTexture() {
        int[] tex = new int[1];
        GLES30.glGenTextures(1, tex, 0);
        GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, tex[0]);
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE);
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE);
        return tex[0];
    }
}
