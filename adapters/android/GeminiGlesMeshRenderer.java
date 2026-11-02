package com.yuy.y3realtime;

import android.opengl.GLES11Ext;
import android.opengl.GLES30;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

/**
 * Single-Pass ONE-WARP 33x20 (660 vertices, 1216 triangles) OpenGL ES 3.0 Hardware Mesh Renderer.
 * Renders stabilized OES camera texture directly to both Preview Surface and MediaCodec Encoder Surface.
 */
public final class GeminiGlesMeshRenderer {
    public static final int MESH_COLS = 33;
    public static final int MESH_ROWS = 20;
    public static final int VERTEX_COUNT = MESH_COLS * MESH_ROWS; // 660
    public static final int INDEX_COUNT = (MESH_COLS - 1) * (MESH_ROWS - 1) * 6; // 3648

    private static final String VERTEX_SHADER_ES30 =
            "#version 300 es\n" +
            "layout(location = 0) in vec2 aPosition;\n" +
            "layout(location = 1) in vec2 aTexCoord;\n" +
            "out vec2 vTexCoord;\n" +
            "void main() {\n" +
            "    gl_Position = vec4(aPosition, 0.0, 1.0);\n" +
            "    vTexCoord = aTexCoord;\n" +
            "}\n";

    private static final String FRAGMENT_SHADER_ES30 =
            "#version 300 es\n" +
            "#extension GL_OES_EGL_image_external_essl3 : require\n" +
            "precision highp float;\n" +
            "in vec2 vTexCoord;\n" +
            "uniform samplerExternalOES uTex;\n" +
            "out vec4 outColor;\n" +
            "void main() {\n" +
            "    outColor = texture(uTex, clamp(vTexCoord, vec2(0.001, 0.001), vec2(0.999, 0.999)));\n" +
            "}\n";

    private int programId = 0;
    private int vaoId = 0;
    private int vboId = 0;
    private int iboId = 0;
    private int uTexLoc = -1;

    private final FloatBuffer vertexBuffer;
    private final ShortBuffer indexBuffer;

    public GeminiGlesMeshRenderer() {
        vertexBuffer = ByteBuffer.allocateDirect(VERTEX_COUNT * 4 * 4)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer();
        indexBuffer = ByteBuffer.allocateDirect(INDEX_COUNT * 2)
                .order(ByteOrder.nativeOrder())
                .asShortBuffer();
    }

    public FloatBuffer getDirectVertexBuffer() {
        vertexBuffer.position(0);
        return vertexBuffer;
    }

    public ShortBuffer getDirectIndexBuffer() {
        indexBuffer.position(0);
        return indexBuffer;
    }

    public void initGl(long nativeSessionHandle) {
        int vs = compileShader(GLES30.GL_VERTEX_SHADER, VERTEX_SHADER_ES30);
        int fs = compileShader(GLES30.GL_FRAGMENT_SHADER, FRAGMENT_SHADER_ES30);
        programId = GLES30.glCreateProgram();
        GLES30.glAttachShader(programId, vs);
        GLES30.glAttachShader(programId, fs);
        GLES30.glLinkProgram(programId);
        GLES30.glDeleteShader(vs);
        GLES30.glDeleteShader(fs);
        uTexLoc = GLES30.glGetUniformLocation(programId, "uTex");

        // Populate static 3648 triangle indices via C++ JNI
        indexBuffer.position(0);
        LiveNativeSession.nativeBuildGlesIndices(indexBuffer);
        indexBuffer.position(0);

        int[] vaos = new int[1];
        int[] bufs = new int[2];
        GLES30.glGenVertexArrays(1, vaos, 0);
        GLES30.glGenBuffers(2, bufs, 0);
        vaoId = vaos[0];
        vboId = bufs[0];
        iboId = bufs[1];

        GLES30.glBindVertexArray(vaoId);

        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, vboId);
        GLES30.glBufferData(GLES30.GL_ARRAY_BUFFER, VERTEX_COUNT * 4 * 4, null, GLES30.GL_DYNAMIC_DRAW);

        GLES30.glEnableVertexAttribArray(0);
        GLES30.glVertexAttribPointer(0, 2, GLES30.GL_FLOAT, false, 4 * 4, 0);

        GLES30.glEnableVertexAttribArray(1);
        GLES30.glVertexAttribPointer(1, 2, GLES30.GL_FLOAT, false, 4 * 4, 2 * 4);

        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, iboId);
        GLES30.glBufferData(GLES30.GL_ELEMENT_ARRAY_BUFFER, INDEX_COUNT * 2, indexBuffer, GLES30.GL_STATIC_DRAW);

        GLES30.glBindVertexArray(0);
    }

    /**
     * Uploads the latest 660-vertex mesh (computed by nativeProcessFrameToGlesMesh) and renders
     * a single-pass ONE-WARP frame to the currently bound EGLSurface (Preview or MediaCodec).
     */
    public void uploadMeshAndDraw(int oesTextureId, int viewportW, int viewportH) {
        GLES30.glViewport(0, 0, viewportW, viewportH);
        GLES30.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT);

        GLES30.glUseProgram(programId);
        GLES30.glActiveTexture(GLES30.GL_TEXTURE0);
        GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, oesTextureId);
        GLES30.glUniform1i(uTexLoc, 0);

        GLES30.glBindVertexArray(vaoId);
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, vboId);
        vertexBuffer.position(0);
        GLES30.glBufferSubData(GLES30.GL_ARRAY_BUFFER, 0, VERTEX_COUNT * 4 * 4, vertexBuffer);

        GLES30.glDrawElements(GLES30.GL_TRIANGLES, INDEX_COUNT, GLES30.GL_UNSIGNED_SHORT, 0);
        GLES30.glBindVertexArray(0);
    }

    public void releaseGl() {
        if (vaoId != 0) {
            GLES30.glDeleteVertexArrays(1, new int[]{vaoId}, 0);
            vaoId = 0;
        }
        if (vboId != 0 || iboId != 0) {
            GLES30.glDeleteBuffers(2, new int[]{vboId, iboId}, 0);
            vboId = 0;
            iboId = 0;
        }
        if (programId != 0) {
            GLES30.glDeleteProgram(programId);
            programId = 0;
        }
    }

    private static int compileShader(int type, String src) {
        int shader = GLES30.glCreateShader(type);
        GLES30.glShaderSource(shader, src);
        GLES30.glCompileShader(shader);
        return shader;
    }
}