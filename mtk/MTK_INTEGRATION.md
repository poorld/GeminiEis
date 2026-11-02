# GeminiEis three-end integration

Single algorithm: GeminiEisCore C++17 with stable C ABI.
PC, Android App JNI and MTK Streaming use the same algorithm source.

## MTK mapping
SensorProvider BOOTTIME gyro -> PushGyro; P1 FRAME_START_TIMESTAMP_BOOT -> frame timestamp; exposure/skew -> FrameInfo; mISrcZoomROI -> SetViewport; visual residual -> PushVisual; mesh -> MTK renderer.

## Validation
Host three-adapter parity passed for correction and 660 mesh vertices. Android ARM64 and ARMv7 archives/JNI compiled. MTK HAL compilation and device validation remain pending.

## Safety
Keep gyro coordinate transform, time offset, RS19, target, crop and warp inside core. Do not modify legacy s_yuyeis2 production plugin before isolated verification.
