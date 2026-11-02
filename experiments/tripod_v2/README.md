> **2026-10-24 最新 C2-C3 Global Path（仅PC SHADOW，产品OFF）**：四片各291原RAW真实H264与中心/左右/全局Roll/Scale拆解验证。先找到全图Global Rotation残余是C2未采纳Native C1-B2可见Rotation，相关0.757～0.914；加入C3 C++唯一qVirtual+Visual+Zoom的全局4DoF前提交预算后，四片Global Roll p90明显下降（024630：0.102°→0.042°，V1黄金0.039°），024647最大边缘极值6.016px→1.132px。但是四段全图真实motion p90仍高于V1，**综合质量0/4 FAIL**，不切换黄金；Host C1+C3联合算力也超出29.6fps余量。已试验因果Yaw半周期/物理RS19预测：完整未来余量版本因Zoom过高而FAIL；冻结该版本。唯一owner、全660网格源UV安全与无未来60帧因果测试PASS。**完整记录 experiments/tripod_v2/C3_PROGRESS_20261024.md；56张真实SVG＋四片V1/C2/C3 H264专业报告 reports/gemini_tripod_v2/c3_global_path/index.html**。不修改Android/MTK/ALPS、原产品src/include、原V1黄金，不commit/push/reset/clean。

> **2026-10-24 历史阶段 C2 PC Shadow（用户不在公司）**：已真正实现独立C++唯一qVirtual+Visual+Zoom Joint Owner，四片2560×1440同源原RAW各291帧、C1-B2 Native Global Visual、按Camera TS推送400Hz Gyro、完整物理20×33 RS19边界≥2px后原子提交；两种校准候选各生成4个真实1280×720 H264，对V1黄金做独立真实像素左中右FB-LK，产出40张专业SVG/三路视频同步播放。**Causal+RS19 HOST PASS，但整体画质Gate 0/4 FAIL，产品继续OFF。** 中心Visual按源像素8:1恢复，Zoom边缘变化≤2输出px/帧后，024618/024630/024647边缘最坏峰值分别7.21→2.10、12.55→2.98、6.02→1.89px；然而四段全图Motion p90均比V1更高，必须进入C2-C3全局路径+边界因果预测，不继续末端Zoom补丁。完整记录：**experiments/tripod_v2/C2_PROGRESS_20261024.md**；真实画质图表及视频：**reports/gemini_tripod_v2/c2_shadow_cal/index.html**；设备未测，不改V1 Golden、Android/MTK/ALPS，不commit/push/reset/clean。

> **2026-10-24 历史阶段C1-B2：V2C C1-B2 Native Camera-Y输入 Host PASS（用户不在公司，未测DEVICE）**。真正原MP4的解码YUV420p亮度平面直接交可移植C++：2560×1440按Camera Y-plane rowStride/pixelStride缩至320×180 → 真20行物理RS19两幅特征预稳 → C++3×3 Tile FB-LK + 当前SourceID的单任务异步。4×291真RAW、1160/1160个SourceID视觉在Host从Y-plane ready起33.78ms内有效交付，p90 20.4–21.7ms；更严20ms预算仅886/1160。原V1/C2始终OFF/不改，实际Android Camera/GPU/编码30fps NOT_TESTED。完整独立代码和负面真实预算：**experiments/tripod_v2/C1B2_PROGRESS_20261024.md**；新专业10×4 SVG总入口：**reports/gemini_tripod_v2/c1b2_yuv/index.html**。下一项应PC隔离准备C2唯一轨迹控制器，同时保留Device Gate待回公司验收。

> **2026-10-24 历史阶段C1-B：V2C C1-B Native FB-LK + C++单任务异步**：四段原RAW 1160个视觉pair均通过Native视觉parity；每段Native LK p90约11.6–12.8ms；严格Host原软件解码到当帧结果33.78ms Gate仅101/1160按时，而“特征已准备好”后Native20ms Gate1158/1160按时。因此算法Native化有真实成果，但完整Host当帧30fps仍FAIL、Android Device NOT_TESTED、C2 Target OFF，所有迟到帧视觉强制拒绝。**C1B_PROGRESS_20261024.md** 记录全部C++/异步/超时证据；4×10真实图表在 **reports/gemini_tripod_v2/c1b_native/index.html**。原C1-A和V1黄金均冻结保留。下一项是Camera原生YUV直达/Native RS预稳的真实同帧时序，然后才接qVirtual+Visual+Zoom唯一目标控制器。

> **2026-10-24 历史阶段C1-A：V2C C1 Host CAUSAL INPUT PASS（4×291帧）／Host完整30fps性能BLOCKED／DEVICE NOT_TESTED。** 已新增真正不读取未来Camera帧和离线Oracle的400Hz Gyro＋20行RS19＋Native低分辨率预稳＋3×3中心加权全局LK＋过去Keyframe观察器，四段RAW运动p90分别抑制78.6%／78.4%／74.4%／81.9%，全部只在SHADOW中观测。完整实现与负面性能证据见 **experiments/tripod_v2/C1_PROGRESS_20261024.md**；专业图表见 **reports/gemini_tripod_v2/c1_causal/index.html**。之前V2B末端控制两条失败路线仅为历史证据，不应当作当前方案。原V1 GOLDEN冻结。

> **输入因果加强**：Host现改用CausalCSVGyroFeeder顺序读取400Hz Gyro数据并只提交到当前帧截止时间；与此前加载CSV再按前缀提交的前80帧结果逐项完全相同。完整四案例和四片Gyro before/after报告已用真正流式版本重跑，详情见上述C1_PROGRESS。

> **2026-10-24 历史阶段V2B**：已经写出并编译单核内统一几何预算控制器，024630同源真实H264三联表明：Zoom回写会拖累210个未采纳帧，不回写会造成32帧边缘跳变，二者均FAIL。四视频原生物理RS19均PASS，但不代表四视频画质已通过。新的工程路线已定为 **V2C：当前帧因果Tile Vision → 单一qTarget+Visual+Zoom轨迹规划（提交前一起求解）→ 全RS19 → 一次高清Warp**。技术证据、4阶段Gate及9张图/三联视频入口在 **experiments/tripod_v2/ARCHITECTURE_V2B.md** 与 reports/gemini_tripod_v2/unified_budget_stage/index.html。V1黄金冻结、未推送/提交。

# Gemini Tripod V2 — 全画幅几何稳定 (2026-10-24)

## 目标与状态

保留原 Gemini 离线三脚架已经接近定住的中心画面，只针对大幅Yaw到极值时左右两边反向伸缩进行几何优化。PC V1 GOLDEN 不动。实验全部限定隔离目录 experiments/tripod_v2/ 和 reports/gemini_tripod_v2/。

**当前产品状态：HOST_OFFLINE / DEVICE_NOT_TESTED。不能称 Android 零前瞻。**
现阶段所有候选的Scale状态机是严格因果的，但其上游Oracle和V1视觉三脚架不是因果的。

## Gemini V1 的真实非因果依赖

- run_tripod_lock_pipeline.py 第一遍全部291帧先预稳定、追踪并累计Similarity2D路径 sim_path。
- 第二遍开始前，anchor[0] = median(params[20:100]) 使用未来20～99帧来初始化画面锚点。
- gaussian_filter1d(anchor, sigma=6, mode="nearest") 作用于整段视觉锚点轨迹，属于双向平滑，使用未来帧。
- oracle_matrices.f64 是离线4DoF处理数据；将它按当前SensorTS作为 pushVisual 输入 PC 重放，不代表 Android 同帧能产生一样的视觉信号。
- sim/replay_dataset.cpp 预先推Gyro到当前RS行＋50ms，这也是重放，不等于 Android 30fps deadline。
- 因此 Gemini 视频可作为PC效果参照，但原框架不得被描述成 Android 零前瞻实况。不可直接复制到 Y1 HAL 后宣称已经同效。

## 根因和四路原始证据

同一真实SOURCE 2560×1440、Sensor index4～294、RS19 20×33 ONE-WARP、最后180°显示归正，全部291帧。
原Gemini V1真实视频 024630：Motion p90≈0.747px，中心dx p90≈0.718px，左右反向伸缩p90≈0.868px、最严重第67帧≈12.545px。
第67帧 Stage2视觉纠正矩阵Scale变化−3.254%，原Oracle仅−0.096%，说明后级Similarity估计异常放大了边缘伸缩。
当前C++ core动态Zoom是需要时立刻增加、恢复时tau8秒衰减；几何边缘不足2px会连同qTarget和Visual用SLERP/线性缩放回拉。Zoom和视觉Scale是相互耦合的，不是只有一个坏变量。

前一阶段2×2实验:
- 动态Zoom＋原Scale：全图Motion p90≈0.747、伸缩max≈12.545。
- 动态Zoom＋关闭后级Scale：Motion≈0.724、max≈15.877（单独删除Scale反而更差）。
- 固定Zoom 1.14＋原Scale：Motion≈0.774、max≈11.894。
- 固定Zoom 1.14＋关闭后级Scale：Motion≈0.794、max≈2.516，但中心/日常p90恶化。

## V2 可执行隔离算法

入口 experiments/tripod_v2/zoom_scale_causal_governor.py:
- 读取原始Oracle与V1合成Visual（输入仍离线）；分解Stage2新增视觉Similarity矩阵 E = Visual @ inv(Oracle)。
- 针对 E 的log(scale)使用单方向逐帧Causal Slew Rate或异常大跃迁后才启动的Outlier-Only控制。
- 保留E的旋转，缩放2×2后，平移项通过同一Oracle后的光学中心齐次映射精确重定位；独立测试同一中心差异 <1e-8，保证不会平白移动稳定中心。
- 与Zoom下限联合扫查：dynamic、1.08、1.10、1.12、1.14；通过独立 PC replay副本调用**原版 GeminiEisCore和Rs19OneWarp**生成20×33网格。Zoom floor 1.08/1.10/1.12是来自基础Replay的复制文件（仅改fi.zoom），不直接修改主源码。
- 最终生成的是原raw传感器→单次RS19 remap→180°输出→H.264编码。不能把网格更平滑当画面效果。
- 独立纯C++移植候选 experiments/tripod_v2/CausalScaleGovernor.h，只包含当前和历史SensorTS，不读取未来帧；测试test_causal_governor.cpp全PASS。该状态机PC优化编译独立开销≈0.21us/帧；不是整条链路性能。

## 全链路图表与证据 Gate（依Y3专业图表规范）

1. 输入：原raw 2560×1440、Camera SourceID4～294、Camera/IMU/RS行时间、编码帧的身份/方向。
2. 识别：中心和左右背景LK、FB置信度，V1 Stage2新增Scale与Oracle分离、突变帧定位。
3. 控制：因果Zoom floor与Scale rate/outlier值、当前qTarget、边界回拉、有效总缩放及最大跃迁。
4. 几何：RS19 20×33（19行区间）网格，所有660顶点和外围插点、UV/margin、无新增黑边和非线性畸变。
5. 输出：真实编码MP4的官方track_pair motion p90/RMS、中心ROI p90、左右相对中心p90、反向伸缩p90及max、半周期极值事件；所有四视频必须同源A/B。
6. 性能：Host native测量与视觉提取/渲染/编码分开；Android CPU/GPU/30fps/发热尚未进行。

**黄金晋升硬门槛**：全部四段H264正常、0新增UV越界/坏帧；总体Motion p90不高于旧版5%，中心运动不高于5%，左右反向伸缩日常p90不高于5%，极值max尽可能下降超过一半；PAN不允许生硬、Zoom不能高频抽动。不达即产品OFF。

## V2已测初版参数（024630）

| 参数 | Motion p90 | 中心dx p90 | 边缘反向伸缩 p90 | 极值max | 黄金Gate |
|---|---:|---:|---:|---:|---|
| Gemini V1原版 | 0.747 | 0.718 | 0.868 | 12.545 | GOLDEN |
| 固定1.14＋Scale对数每帧限0.15% | 0.724 | 0.812 | 0.951 | 2.273 | FAIL |
| 固定1.14＋每帧限0.30% | 0.784 | 0.812 | 1.122 | 2.913 | FAIL |
| Zoom最低1.08＋每帧限0.15% | 0.752 | 0.724 | 0.914 | 6.015 | FAIL（日常边缘p90略超5%） |
| Zoom最低1.10＋每帧限0.15% | 0.780 | 0.756 | 0.942 | 2.044 | FAIL |
| Zoom最低1.12＋每帧限0.15% | 0.741 | 0.786 | 0.955 | 2.261 | FAIL |

所有候选都在相同291帧raw、原19行RS处理后生成最终1280×720 H264。FFmpeg完整解码无坏帧，全部mesh顶点UV在范围内。几何PASS≠画质PASS。

- Zoom最低1.08＋Outlier-Only仅大跃迁限速0.15%：Motion≈0.741、中心≈0.763、边缘反向p90≈0.977、极值≈5.587。FAIL。
- Zoom最低1.10＋Outlier-Only：Motion≈0.728、中心≈0.757、边缘反向p90≈0.943、极值≈3.924。FAIL。
- 统一重跑全部七个V2候选的最终H264官方track_pair和左右中心FB-LK；**新黄金PASS共0个**。日常边缘与中心恢复仍需要新控制语义，不能调低Gate门槛装作完成。
- **96组真实视频特征留出模型诊断**：以空间交错的背景特征点训练4DoF Similarity和自由8DoF Homography，再用未参与拟合的空间独立特征检验。原Gemini GOLDEN右边区域p90：4DoF约5.34px，自由8DoF约7.98px。自由8DoF对边缘更差。这个实验不支持直接解锁8DoF/网格，也不能把现有差异笼统归因为唯一的非刚性畸变。证据 reports/gemini_tripod_v2/024630/spatial_4dof_vs_8dof_model_heldout.json。
- **专业HTML报告已实际生成并用Chrome验证**：reports/gemini_tripod_v2/index.html、reports/gemini_tripod_v2/024630/index.html。包含输入→识别→控制→几何→输出→性能六环节真实SVG、7个H264候选切换、帧67定位、逐帧三区域LK、Scale/mesh span/UV余量/最终ROI曲线、Gate显示。截图 024630/v2_dashboard_chrome.png。
- 实验CausalScaleGovernor.h 纯C++ Host平均约0.14–0.21微秒/帧；C++单元与Python四个case的16配置前缀不依赖未来帧测试全部PASS。这只证明Scale子模块因果，不证明输入的oracle和二次视觉轨迹因果，更不证明Android零前瞻。

现阶段建议先设计唯一Target owner里的自适应**Zoom–Scale预算联合控制器**并明确Camera稳定目标与完整RS19几何，尽量不要只扫固定Zoom值；将上游offline oracle换成同帧低分辨率预稳定LK后，才能评价实时Android替代架构。

## 四段真实原始录制的 V2 交叉验收（已完整跑完，2026-10-24）

**专业总入口：reports/gemini_tripod_v2/index.html**。四段每段有7张真数据SVG，完整SourceID4～294对应的V1/V2 H264并排，源raw2560×1440未旋转、最后一次ONE-WARP后180°归正。024630另有七候选视频切换和96对空间留出Homography诊断。
参考版本均为原始rendered_preview_fixed的 V1 GOLDEN。当前唯一跨四视频候选是 Zoom下限1.08 ＋ Stage2 logScale每帧0.15%限速。全部四段真实RS19网格0硬性UV越界，最终H264完整解码无坏帧。下面数值为1280×720输出像素域：

| 录制 | 全图Motion p90 (V1→V2) | 中心dx p90 (V1→V2) | 左右反向伸缩日常p90 (V1→V2) | 单帧极值max (V1→V2) | 本段Gate |
|---|---:|---:|---:|---:|---|
| 024605 低速往复 | 0.122→0.163 | 0.253→0.255 | 0.108→0.107 | 0.224→0.265 | FAIL |
| 024618 中速往复 | 0.163→0.177 | 0.295→0.331 | 0.195→0.514 | 7.213→0.965 | FAIL |
| 024630 快速往复 | 0.747→0.752 | 0.718→0.724 | 0.868→0.914 | 12.545→6.015 | FAIL |
| 024647 单侧Pan | 0.190→0.200 | 0.361→0.397 | 0.295→0.553 | 6.016→4.670 | FAIL |

**跨场景真实效果Gate：0/4 PASS**。原因不是RS19黑边，实际是静态Zoom下限与Scale慢变化消除局部极值时，把平时稳到极致的V1画面又推回轻微起伏。024618的日常边缘伸缩p90从0.195变0.514最能证明“固定1.08 + 0.15% limit”不是最终统一控制算法；024605整体Motion也增加约33%。禁止因024630最坏帧下降52%就升级产品。

**下一刀算法架构，不再盲目调固定Zoom/Scale参数：**在唯一Target owner里面引入有真实RS19 Margin约束的**场景和时域自适应Zoom–Scale联合预算优化器**；重点使锁定阶段的正常帧保留V1原样，仅在有可靠几何剩余空间、预测到边界风险和临近Yaw反向窗口时连续分配Zoom与局部Scale自由度，统一处理qTarget和视觉变换，防止两项控制打架。任何多帧优化只能使用当前/历史传感器和已经产生的可信视觉状态（前帧路径/因果关键帧），不能预读后续视频帧。自由8DoF未通过空间留出验证，本阶段禁止放开自由Homography。
先在PC通过四视频全部指标，再单独做Android真实Camera→320×180预稳定→Keyframe多区LK→C++ Target→19RS/20×33 mesh→全分辨率单Warp→encoder同帧耗时、30fps和功耗验证。



## 完整复现文件

- experiments/tripod_v2/zoom_scale_causal_governor.py：逐帧缩放候选＋真实C++ Replay RS19 Gate。
- experiments/tripod_v2/render_and_evaluate.py：源帧逐张解码、单次20×33网格重采样、180度归正、H.264与FFmpeg零错误验收、实际官方track_pair＋三区域LK。
- experiments/tripod_v2/CausalScaleGovernor.h 和 test_causal_governor.cpp：C++因果子模块、乱序/非有限值/前缀因果测试。
- reports/gemini_tripod_v2/024630/<variant>/：visual_governed.f64、mesh_governed.f32、manifest.json、gemini_v2_output.mp4、render_gate.json。
- reports/gemini_tripod_v2/024630/real_pixel_comparison.json：各轮真实像素结果（会被最后一次运行覆盖，之后须另存汇总）。
- reports/gemini_scale_ab/024630/index.html：上轮四因素A/B，能打开原版真实输出。

## 下一阶段：真正Android零前瞻（尚未实施）

- 不再输入离线oracle_matrices.f64，不得使用未来anchor[20:100]、全片Gaussian。
- 在同一Sensor帧已经可用时先低分辨率Gyro预稳定→当前帧与已缓存Keyframe多区域LK，获得真正因果Visual（不需先输出整幅视频再重新分析）。
- 在唯一的C++ Target owner中联合Zoom–Scale预算、真实RS19全网格、平滑虚拟三脚架和自然PAN，再只做一次高清输出Warp。
- 区分低分辨率预展平代价和全分辨率ONE-WARP，验收真正Y1 MT6768上的Camera→Visual→Mesh→GPU→Encoder耗时、29.6～30fps、资源退出、功耗、质量。
- 不能从PC avg0.8ms mesh或Scale avg0.21us推导Android实时PASS。可行性必须以真机日志证明。

本轮原Gemini repo master已有无关dirty状态，不操作任何git reset/clean/push/commit，冻结V1视频和src核心文件。
