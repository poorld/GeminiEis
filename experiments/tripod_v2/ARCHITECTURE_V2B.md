> **最新C1状态（2026-10-24）**：四段真实原RAW的因果Gyro+RS19＋中心加权Tile LK Host输入Gate PASS；Host真实完整30fps与Android Device Gate仍BLOCKED/NOT_TESTED；结果严格SHADOW，没有声称修复边缘变形。C1源码/测试和专业HTML请见 experiments/tripod_v2/C1_PROGRESS_20261024.md 与 reports/gemini_tripod_v2/c1_causal/index.html。V2B的失败结果保留供下一阶段C2唯一轨迹求解器设计，不可继续末端Zoom补丁。

# Gemini Tripod V2B → V2C：从末端补丁转为前提交统一虚拟相机轨迹（2026-10-24）

## 0. 架构决策（当前最新）

**Gemini V1 PC离线三脚架保持为唯一画质黄金版。** V2固定Zoom+Scale限速在四段视频实测 Gate 0/4 PASS。

V2B用一个实际 C++ Core、原物理 Rs19OneWarp 20×33 网格、同源291帧RAW实现了两种预算路径：
- unified_joint：在最终Warp前优化，并把结果Zoom回写旧Core的8秒退场状态。024630仅12帧直接接受，其余210帧因为历史反馈也不同于V1。中心运动p90 0.718→0.701，边缘反向伸缩p90 0.868→1.044，极值12.55→5.52px，效果FAIL。
- unified_output_only：在最终Warp前做预算选择，但不写回旧Core的长期状态。024630接受32帧、其余259帧完整20×33网格逐顶点与V1 bit-exact。中心p90 0.718→0.685，全图p90 0.747→0.718，但边缘伸缩p90变成3.229、极值9.70px，输出不连续造成新的边缘摆动，效果FAIL。
- 两种PC试验全部经过原始291帧RAW 2560×1440、相同SensorTS/RS19几何、180°显示归正、真实H264、独立最终图像FB-LK。源UV越界0；几何合法不能代替最终画质。
- 最新六环节9图三联真实视频：reports/gemini_tripod_v2/unified_budget_stage/index.html，Chrome渲染PASS。
- 纯C++数值/时间测试：normal帧完全比特等价、PAN veto、无未来样本、完整20×33RS安全门禁PASS。PC核心实测约0.8–1.2ms/帧，不含视觉LK/GPU/EGL/编码，绝非Android 30fps结论。

**已经证实的根因：当前GeminiCore是先计算原qTarget和视觉、再计算Zoom/几何不足时末端SLERP回拉，最后Warp。无论在末端回写Zoom还是不回写，都会破坏不同时间范围内的边缘连续性。因此下一阶段必须改变Target/Visual/Zoom的先后决策顺序，而不是再做一档Zoom或Scale阈值。**

## 1. V2C真正的新数据通路

1. Camera raw原始2560×1440 +真实SourceID/SensorTS/Exposure/Readout +相机域400Hz Gyro。
2. 当前帧低分辨率Gyro预稳定（如192×108或320×180），只供当前帧视觉特征；不属于最终高清重复Warp。
3. 当前帧 vs 已存在Keyframe/前帧的 3×3～5×5 Tile FB-LK，中心优先但是外围非零；记录Visual trust、拒绝、背景共模、场景动态污染。
4. 基于历史/当前Gyro和多Tile累计路径的因果转向与半周期识别：早期Yaw反向、自然PAN、stop、局部背景置信度。不能将“电脑屏幕固定ROI”当算法前提。
5. **唯一的C++虚拟相机轨迹规划器**同时持有状态：qVirtual、目标角速度/角加速度、Visual平移/旋转/Scale、Zoom、上一帧20×33 RS Margin、正常PAN/反转状态、已采纳完整Target。
6. 在当前同一个决定点联合选择 qTarget、Visual和Zoom，使用完整原物理RS19安全门禁，确保MaxZoom1.14、安全margin≥2px、无新增UV/黑边/裁剪。为中心、左右相对中心变形、时间连续性/jerk及自然PAN建立同一个有界成本函数。只允许当前/过去真实帧；模型预测不得偷看未来实际视频。
7. 对整个联合状态原子提交到下一帧，再输出一次全分辨率RS19 ONE-WARP。**不能先把Zoom写入8s旧状态，再在末端补丁层选择别的Scale；也不能只选择当前Warp不处理时间连续性。**
8. 输出帧、Encoder、preview使用同一真实SensorTS和单一C++目标；停止/离开相机后完整资源释放。

## 2. 关键控制语义

- **普通状态黄金优先**：纯Gyro/CVC已经很稳、画面/RS余量充分、Tile背景残余不可靠时保持原V1控制意图，不能因为固定Zoom floor让全段呼吸。
- **半周期状态提前介入**：从Yaw历史半周期趋势、上一帧几何余量变化和背景Scale异常识别极值风险，不使用未来真实样本。“第一帧看不出未来反向”是物理因果限制；不可用整段median或双向Gaussian冒充当前已知。
- **真实RS19预算不能凭中心推断**：19行区间/20×33顶点，逐行/外围源点物理安全、无需降级Crop或盲目放Zoom；安全余量不足允许缓慢跟随，不得突然把qTarget与Visual拉回。
- **连续性不是补偿后门禁**：上一帧实际采纳的qTarget/Scale/Zoom以及速度参与当前联合求解，输出与内部状态原子更新；两次基准+候选独立运行会产生状态分叉，不是产品结构。
- **空间残余尚未证明需要Mesh**：024630实际V1右侧空间留出特征误差p90，4DoF约5.34px、自由8DoF约7.98px。禁止直接打开自由Homography/8DoF。只有证明跨景深无法用受约束投影解释且留出点真实改善，才考虑极弱正则化局部网格。

## 3. 离线骗局必须明确拆掉

原V1好画质是真实的，但上游有明确非零前瞻：run_tripod_lock_pipeline.py用未来帧20:100初始化anchor、整段gaussian_filter1d双向平滑；oracle_matrices.f64是离线4DoF整段准备的运动轨迹；replay_dataset.cpp预送IMU到RS行时间+50ms。因此V1效果可作为PC质量上限，不可宣称Android真正Zero Lookahead。即使本轮UnifiedBudget/CausalScaleGovernor类本身只读当前及历史，整段管线仍是HOST_OFFLINE。后续C1必须用真实当前帧预稳定LK替换这些输入。

## 4. 隔离实验真正完成项

- experiments/tripod_v2/UnifiedTripodBudget.h：单核内C++有界预算探索 + 20×33全网格真实安全，当前/历史SensorTS语义。
- experiments/tripod_v2/build_unified_v2.py：使用原src与include的只读快照在reports目录生成实验C++/头文件，独立编译Core+C API，原C++核心不修改。
- experiments/tripod_v2/test_unified_budget.cpp：RS19真值、无前瞻、普通帧完全比特等价、PAN否决、真实几何等Host单元测试。
- experiments/tripod_v2/evaluate_unified_joint.py：真实原SOURCE图像、RS19、同源H264、三区域FB-LK。
- experiments/tripod_v2/generate_unified_budget_report.py：真实6模块9图三联视频专业HTML，物理网格、历史Zoom反馈与最终光流指标。
- Native四录像安全证据：024605反馈/无反馈均0帧采用、024618均2帧、024630反馈12/无反馈32、024647反馈10/无反馈26；原生20×33均0UV越界。024647反馈引起133个额外非直接采用帧差异。**四录像这里只证明Native几何，V2B真H264三个模式的最终像素仅024630，不得声称其余三个V2B视频已验收。**
- V2阶段前一轮固定Zoom＋Scale限速在四录像的真实H264全部评测不达黄金，详见 experiments/tripod_v2/README.md 和 reports/gemini_tripod_v2/index.html。

## 5. 下一四个Gate（禁止跳）

C1 真因果视觉：Camera SourceID/SensorTS→当前帧低分辨率Gyro预稳定→当前/过去Keyframe多Tile LK；没有未来Gyro/Visual/帧。HOST causal test及真实原视频四段对齐PASS才进入C2。

C2 唯一Target+Zoom+Visual联合规划：把旧GeminiCore的 qTarget 之后独立Zoom及末端回拉拆掉，联合状态与真实RS19同一个求解器预提交。保住自然PAN与V1中心锁定，冻结Zoom限幅/RS19安全语义。历史状态仅在整条联合候选通过安全门禁后原子更新。

C3 四段同源效果：按原Raw+SensorTS+RS19+Orientation真实H264输出，分别看全图Motion、中心dx、左右relative-center p90和半周期max、Zoom速率、jerk、UV/黑边和PAN。不得退化V1日常稳定，极值必须明显改善。输出六环节图表并保留失败候选。

C4 Android真机：单摄2560×1440源→1920×1080输出，Camera→因果Tile LK→C++唯一Target→RS19物理Mesh→GPU ONE-WARP→Encoder，29.6–30fps和无67ms跳帧、所有资源释放/温升实测。PC 0.8ms不可推断真机30fps；必须Device PASS。

## 6. Git/设备边界

D:\hongda\app\GeminiEis master分支原已有大量dirty代码，这些不能改动、reset或clean。本轮只增加experiments/tripod_v2与reports/gemini_tripod_v2隔离试验文件；未提交、push、reset、clean、未修改src/include原版和黄金视频；Android/MTK/ALPS均未改。V2B两分支Gate FAIL，不能用于产品。
