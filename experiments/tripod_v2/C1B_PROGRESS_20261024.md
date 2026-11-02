# Gemini Tripod V2C — C1-B Native LK / Asynchronous Frame Deadline (2026-10-24)

## 当前阶段 Gate / 结论

**C1-A 已通过 Host 因果视觉输入；C1-B 可移植C++多区域LK已通过四段原RAW与原OpenCV前端parity。C1-B Native单任务异步worker、按SourceID/截止时间fail-closed及取消/回收Host测试通过。**

**但是「从原RAW软件解码开始算到同一帧视觉结果交付」的严格Host帧预算 FAIL，不能进入C2正式补偿，更不能宣称Android真实0前瞻30fps。** 目前C1B始终为纯SHADOW；Gemini V1画质黄金、src/include/高清RS19 ONE-WARP/C2 Target/Zoom不变。

### 核心问题已被拆成两层

1. 已具备特征图时，独立C++ Native LK/线程大多可在20ms内完成；真实四段合计 **1158/1160** 当前帧及时得到结果（20ms只是HOST已准备好特征输入的条件Gate，不包括Camera/YUV/预稳）。
2. 原RAW的Windows软件解码、灰度缩小、当前/前帧RS19 prewarp和Native线程加起来，要求从每个Host帧解码起点算33.78ms，四段合计仅 **101/1160** 在截止前完成，**严格Host Pipeline Gate FAIL**。这里用的是软件读取MP4，不是真机Camera3直接给YUV；不应把Host失败误归因到算法算力不足，也不能把第1项的成功偷换为Android录制实时PASS。

## C1-B已写的核心实现

- experiments/tripod_v2/C1NativeTileLK.h/.cpp：标准C++17，原生3×3独立区域角点、3层金字塔前后LK、FB过滤、RANSAC背景共模、中心优先/外围非零weighted 4DoF similarity和信任门禁。所有C++方法有可复用C ABI，不依赖Python/OpenCV C++动态库。
- experiments/tripod_v2/c1b_native_visual.py：ctypes只复制/传入320×180预稳定及有效mask，Native C++核内完成完整角点/LK/背景拟合。与C1-A对比记录每帧center dx/dy、rotation/logscale、9Tile支持和CPU用时。
- experiments/tripod_v2/C1NativeAsyncLK.h/.cpp：独立 C++ std::thread、最多1个inflight任务、SourceID及sensorTS严格单调、固定大小输入拷贝/锁保护、每个当前SourceID的monotonic deadline、非阻塞submit/poll、busy reject、late reject、cancel与join/destroy。
- experiments/tripod_v2/c1b_native_async.py：Host FFI包装；本阶段没有任何一帧旧的视觉结果被用于下一SourceID，过期帧直接拒绝，无法在当前帧时限内完成的场景仍保留纯Gyro控制权。
- experiments/tripod_v2/c1b_replay.py：冻结四段真实原RAW共1164帧与已验C1-A相同SourceID/SensorTS/20row RS19，用新Native LK替换Python OpenCV，记录四段前端parity和耗时。
- experiments/tripod_v2/c1b_async_replay.py：四片RAW两类Host wallclock截止时间复验，真实原始SourceID/PTS与Gyro前缀；迟到的C1结果会被丢弃。模式 full 是从原RAW软件解码起点到同一帧算法结果33.78ms，feature 是当前320×180特征就绪后至Native结果20ms。二者都非Android Camera实测。
- experiments/tripod_v2/test_c1b_async.py：队列Busy、错误SourceID/TS、forced 85ms delay导致25ms超时、旧结果不复用、显式cancel、Stop/Join安全回收Host测试。
- experiments/tripod_v2/c1b/CMakeLists.txt：独立构建两个共享C++库（多区域视觉+异步，物理RS预稳），实测Windows MinGW独立CMake双目标编译PASS；Android NDK库编译和真机未做。
- experiments/tripod_v2/c1b_generate_professional_report.py：每段10张实际数据SVG，按输入→识别→控制→几何→输出→性能六个环节组织。PC Chrome已验证总览和024630明细。

## 四段原录制的C1-A / C1-B 算法parity和性能

原视频每段291帧，帧间视觉pair每段290/290全部接受。统一的原RAW和SensorTS，Native结果不对最终V1成品施加任何补偿。

| 录制 | Native vs OpenCV dx绝对差 p90 (320px域) | Native LK p90 | C1-B处理p90不含解码 | MP4软件解码+C1-B p90 |
|---|---:|---:|---:|---:|
| 024605 | 0.034 px | 11.6ms | 22.4ms | 43.4ms |
| 024618 | 0.038 px | 12.2ms | 25.2ms | 44.6ms |
| 024630 | 0.058 px | 12.8ms | 29.2ms | 46.7ms |
| 024647 | 0.077 px | 12.6ms | 27.1ms | 44.7ms |

四段logscale与原OpenCV差绝对值p90约0.00052 / 0.00081 / 0.00191 / 0.00144。C1-A原Python OpenCV视觉处理p90分别24.6 / 27.7 / 33.6 / 29.3ms；C1-B有小幅优化但不是零成本，也不能表述为已解决30fps全链路。

## 异步调度—真实时间证据必须明确区分

| 录制 | 严格Host整帧33.78ms按时SourceID | 仅已准备好feature后Native20ms按时SourceID | 是否可给C2实时产品 |
|---|---:|---:|---|
| 024605 | 78 / 290 | 290 / 290 | 否 |
| 024618 | 10 / 290 | 289 / 290 | 否 |
| 024630 | 9 / 290 | 290 / 290 | 否 |
| 024647 | 4 / 290 | 289 / 290 | 否 |
| 合计 | 101 / 1160（8.7%） | 1158 / 1160（99.83%） | 否 |

- Host full测试从cv2.VideoCapture软件解码开始的Host monotonic时间计算33.78ms截止；C++ worker严格校验本时限，迟到的旧SourceID不会被误当新帧，失败时视觉门禁OFF。由于Host软件解码本身不等于Android Camera ImageReader/YUV真实输入，这个时间不能外推为真机必然FAIL。
- Host feature-ready测试故意从320x180预稳图已存在后开始计时20ms，**不包含原RAW软件解码、预稳定/RS19准备/最后高清GPU Warp和Encoder**，所以它只是C++异步执行器在已准备好输入下的能力，不是完整同帧交付证明。
- 全Host同步处理的p90可以超过33.78ms；严格full模式逐帧拒绝迟到，所以其“从source到决策耗时p90”可能被deadline截断，不能当真实全阶段计算耗时。
- Native故障注入：过载SourceID不得排队补交，重新提交倒序帧会拒绝，手动取消与自动超时都会禁止后续帧拿旧结果；进程退出join后资源释放。
- 合成场景中心动态物体11px而背景2px时Native全图结果约2.0007px，外围8个Tile提供支持；无特征图直接拒绝。Native适用于多场景，不是仅识别“电脑屏幕”。

## 当前下一步：C1-B2 真实Camera/YUV准备与设备Deadline

1. 原生零复制或最小复制 Camera YUV/低分辨率前端替代Host软件MP4→BGR→灰度缩小链路。Native RS19 320x180预稳与C++ Async LK共用一份Buffer；不得造第二次高清Warp。
2. 在Android Y1设备上记录SourceID、SensorTS、曝光+RS readout、当前帧图像到达时间、Gyro覆盖截止、前处理/NativeLK完成、最后Warp提交截止、Encoder PTS、30fps丢帧、STOP/Flush/cancel、发热。真实Deadline必须来自Camera/渲染执行时序，不可复用Host 20ms假设。
3. 严格当前帧：视觉过时则fail-closed、绝不偷用上一个SourceID；若高负载下频繁过时则必须继续优化预处理和LK，而不能强行提升Visual gain或放宽Zoom/裁剪掩盖。
4. 只有达到C1-B2同帧交付和真实性能/时序Gate，再接C2唯一C++ Target owner，联合qVirtual + Visual路径 + Zoom + RS19几何预算，完整原子提交。C1-B所有输出目前仍SHADOW，未产生新稳定录像，不能称边缘扭动已改善。

## 资料入口

- reports/gemini_tripod_v2/c1b_native/index.html — 四段专业图表总览（4*10张SVG）。
- reports/gemini_tripod_v2/c1b_native/{case}/index.html — 同源Native/OpenCV对比、时间序列、同帧Deadline fail-closed。
- reports/gemini_tripod_v2/c1b_native/{case}/native_ab.json、frames.jsonl — Native每帧测量。
- reports/gemini_tripod_v2/c1b_native/{case}/async_deadline_summary.json、async_frame_deadline.jsonl — 严格33.78ms Host结果。
- reports/gemini_tripod_v2/c1b_native/feature_ready/{case}/async_deadline_summary.json — 已准备好feature后的20ms测试。
- reports/gemini_tripod_v2/c1b_native/async_failure_injection.json — 异步错序/超时/取消/退出测试。
- reports/gemini_tripod_v2/c1b_native/c1b_professional_evidence.json — 所有Host Gate结论。

工作目录 D:/hongda/app/GeminiEis（master和原有dirty文件必须保留），新增只在experiments/tripod_v2及reports/gemini_tripod_v2/c1b_native；未push/reset/clean/commit；Y1 Android/MTK/ALPS未修改，原Gemini V1的Golden H264、原core/source未覆盖。
