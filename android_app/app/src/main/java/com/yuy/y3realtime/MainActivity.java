package com.yuy.y3realtime;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.view.Gravity;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.util.Locale;

/**
 * Fullscreen Live Camera2 + OpenGL ES 3.0 ONE-WARP Stabilization Activity (Golden Release).
 * Features:
 *   - Persistent preview orientation (default 270 deg upright on Y1 AR_6769)
 *   - Real-time H.264 MP4 hardware video recording of the stabilized ONE-WARP mesh output
 *   - Compact 2-row overlay HUD tailored for Y1 display
 */
public class MainActivity extends Activity {
    private static final int REQ_CAMERA_PERMISSION = 1001;

    private GLSurfaceView glSurfaceView;
    private TextView hudTextView;
    private Button toggleButton;
    private Button rotateButton;
    private Button recordButton;
    private GeminiCamera2EisController controller;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        FrameLayout root = new FrameLayout(this);
        glSurfaceView = new GLSurfaceView(this);
        glSurfaceView.setEGLContextClientVersion(3);

        hudTextView = new TextView(this);
        hudTextView.setTextColor(Color.GREEN);
        hudTextView.setTextSize(9f);
        hudTextView.setBackgroundColor(0x88000000);
        hudTextView.setPadding(10, 6, 10, 6);
        hudTextView.setText("GeminiEIS 2.1 Golden (Same-Frame Sync + 8DoF + 20x33 RBF)");

        toggleButton = new Button(this);
        toggleButton.setTextSize(10f);
        toggleButton.setText("EIS: ON");
        toggleButton.setOnClickListener(v -> {
            if (controller != null) {
                boolean next = !controller.isEisEnabled();
                controller.setEisEnabled(next);
                toggleButton.setText(next ? "EIS: ON" : "EIS: OFF");
            }
        });

        rotateButton = new Button(this);
        rotateButton.setTextSize(10f);
        rotateButton.setOnClickListener(v -> {
            if (controller != null) {
                int effDeg = controller.cyclePreviewOrientation();
                rotateButton.setText("ROT:" + effDeg + "°");
            }
        });

        recordButton = new Button(this);
        recordButton.setTextSize(10f);
        recordButton.setText("REC");
        recordButton.setTextColor(Color.RED);
        recordButton.setOnClickListener(v -> {
            if (controller != null) {
                boolean rec = controller.toggleRecording();
                recordButton.setText(rec ? "STOP" : "REC");
                recordButton.setTextColor(rec ? Color.YELLOW : Color.RED);
            }
        });

        controller = new GeminiCamera2EisController(
                this,
                glSurfaceView,
                (solveMs, status, frameIdx, enabled, yMatch, yMiss, capMatch, gyroLeadMs, effRotDeg, isRec, recFrames, recInfo) ->
                        runOnUiThread(() -> {
                            rotateButton.setText("ROT:" + effRotDeg + "°");
                            recordButton.setText(isRec ? "STOP(" + (recFrames / 30) + "s)" : "REC");
                            recordButton.setTextColor(isRec ? Color.YELLOW : Color.RED);
                            hudTextView.setText(String.format(Locale.US,
                                    "F#%d | %s | %.2fms | St:%d | Rot:%d°%s\n" +
                                    "Sync Y:%d/%d Cap:%d | Gyro:%+.1fms",
                                    frameIdx,
                                    enabled ? "8DoF+RBF" : "BYPASS",
                                    solveMs,
                                    status,
                                    effRotDeg,
                                    (recInfo != null && !recInfo.isEmpty()) ? (" | " + recInfo) : "",
                                    yMatch,
                                    (yMatch + yMiss),
                                    capMatch,
                                    gyroLeadMs));
                        }));

        rotateButton.setText("ROT:" + controller.getEffectivePreviewRotationDeg() + "°");

        glSurfaceView.setRenderer(controller);
        glSurfaceView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);

        root.addView(glSurfaceView, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT, FrameLayout.LayoutParams.MATCH_PARENT));

        LinearLayout overlay = new LinearLayout(this);
        overlay.setOrientation(LinearLayout.VERTICAL);
        overlay.setGravity(Gravity.TOP | Gravity.START);
        overlay.setPadding(8, 8, 8, 8);

        LinearLayout btnRow = new LinearLayout(this);
        btnRow.setOrientation(LinearLayout.HORIZONTAL);
        btnRow.addView(toggleButton);
        btnRow.addView(rotateButton);
        btnRow.addView(recordButton);

        overlay.addView(btnRow);
        overlay.addView(hudTextView);
        root.addView(overlay, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.WRAP_CONTENT, FrameLayout.LayoutParams.WRAP_CONTENT));

        setContentView(root);

        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.CAMERA}, REQ_CAMERA_PERMISSION);
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            controller.start();
            glSurfaceView.onResume();
        }
    }

    @Override
    protected void onPause() {
        glSurfaceView.onPause();
        controller.stop();
        super.onPause();
    }
}
