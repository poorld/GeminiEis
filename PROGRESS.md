# GeminiEis 开发进度与里程碑日志 (PROGRESS.md)

## 项目信息
* **项目名称**：GeminiEis — Causal Hybrid EIS（严格实时因果混合防抖系统）
* **定位**：面向 PC / Android App / MTK HAL 三端共用唯一 C++17 核心的零未来前瞻流式防抖引擎
* **工作目录**：`D:\hongda\app\GeminiEis`
* **对标与同源工程**：`D:\hongda\app\YuyEIS2-pc-reference`、`YuyEIS2-pc-exact`

---

## 阶段演进规划 (Phased Roadmap)

| 阶段 | 阶段目标 | 关键验收指标 | 当前状态 |
| :--- | :--- | :--- | :--- |
| **Phase 0** | 基线建立与代码审计 | Git 规范、零未跟踪冗余文件、文档完备、初始编译与烟测通过 | **COMPLETED** |
| **Phase 1** | MPC / Core 解析化与约束重构 | 三轴求解 < 0.2ms、阶跃/急停无硬碰撞、无回拉、Crop 约束硬保障 | **COMPLETED** |
| **Phase 2** | 坐标域 Prewarp 与延迟视觉融合 | 192x108 灰度、逐点行曝光时间 t(y)、100ms 历史快照因果残差融合、曝光自适应信任门控 | **COMPLETED** |
| **Phase 3** | Intent Scheduler / LFGO / 参数扫掠 | Hold/Follow 速度连续、Oscillation V2 抑制、1.5~2.5Hz 步频 ITF > 15dB、转向延迟 < 45ms、急停超调 < 0.5° | **IN PROGRESS** |
| **Phase 4** | RS19 ONE-WARP 与三端同源验收 | 33x20 UV Mesh 统一计算、PC/Android/MTK 100% 顶点 Parity、CPU < 8%、4K60 稳定不丢帧 | PENDING |

---

## 历史执行与审计事实记录
1. **2026-10-22 (阶段 0 完成)**：
   * 确认系统严格遵循零未来前瞻（Strict Zero-Lookahead）因果架构，非离线双向平滑。
   * 修复 `include/api/gemini_eis_c_api.h` 与 `sim/c_api_tests.cpp` 文件尾多余空行，`git diff --cached --check` 0 告警通过。
   * 扩充 `.gitignore`，隔离构建产物（`*.dll`, `*.so`, `*.a`, `CMakeCache.txt`, `CMakeFiles/` 等）。
   * 摸底系统原生 MinGW-w64 (`gcc 14.2.0`, `g++ 14.2.0`, `mingw32-make`) 及 CMake 工具链。
   * 建立 `PROGRESS.md` 与 `VALIDATION_MATRIX.md`，完成本地 Git 初始基线提交 (`752215d`)。

2. **2026-10-22 (阶段 1 完成)**：
   * **消除有限差分**：重构 `src/control/constrained_mpc.cpp`，推导显式解析灵敏度矩阵 $S^\theta, S^\omega, S^\alpha$ 与解析伴随梯度 $\nabla J(u)$。
   * **准牛顿尺度预条件**：构造显式对角 Hessian，解决角位移量纲与加加速度量纲之间 $10^8$ 数量级曲率失衡，提升收敛性。
   * **硬边界反解投影**：首帧 $u_0$ 引入物理 Crop 预算闭式区间反解投影，实现 $100\%$ 违背阻断。
   * **零堆动态分配**：内部状态与矩阵全部采用静态 `std::array` 栈内存。
   * **实测门禁数据**：
     * 单轴平均求解耗时：**$30.37\,\mu\text{s}$**（500 次压测平均，三轴总耗时约 $0.091\,\text{ms}$，远优于 $< 0.20\,\text{ms}$ 门禁）。
     * 极端边界违背量：**`boundary violation = 0`**（完全无越界）。
     * 阶跃/急停连续性：**`maxCorrection = 0.0841 rad`**（有界在 $0.10\,\text{rad}$ 内），**`maxTargetStep = 0.00159 rad`**（远小于 $0.02\,\text{rad}$ 门禁，平滑无顿挫）。
     * 三端同源 Parity：**`GEMINI_EIS_3END_PARITY_PASS`**（PC / App / MTK 660 个顶点完全一致）。
   * 完成本地 Git 提交 (`52646b1`)。

3. **2026-10-22 (阶段 2 完成)**：
   * **射线空间陀螺仪预卷绕**：重构 `AsyncVisualWorkerMath`，引入逐点行曝光时间物理模型 $t(y) = t_{\text{sof}} + (y/H) t_{\text{readout}} + 0.5 t_{\text{exp}}$，推导带有透视除法分母 $1 + \Delta r_y x - \Delta r_x y$ 的归一化无畸变射线投影，彻底避免二次像素重采样。
   * **四维联合信任门控**：在 `include/visual/async_visual_worker.h` 与 `src/visual/async_visual_worker.cpp` 中引入 ISO 噪声门控模型，与前后向 LK 误差、RANSAC 内点率及长曝光大运动门控相乘，在弱光高噪（$\text{ISO} > 800$）及内点不足工况下自动降权并切断注入。
   * **100ms 历史快照因果前向重投影**：重构 `MotionStateEstimator` 与 `gemini_eis_c_api.cpp`，扩充 400ms 陀螺仪历史环形缓冲；异步视觉残差到达时，利用其采集时间戳 $t_{\text{vis}}$ 到当前时刻 $t_{\text{curr}}$ 的累积自转角 $\Delta R_z$ 对平面残差 $(t_x, t_y)$ 执行 2D 坐标系因果前向旋转推演，杜绝转动过程中的滞后反向拉扯。
   * **实测门禁数据**：
     * 增量编译通过，静态库 `libgemini_eis_core.a` 与动态库 `libgemini_eis_core.dll` 正常生成。
     * 三端 Parity 测试：**`GEMINI_EIS_3END_PARITY_PASS`**（PC / Android JNI / MTK HAL 660 个顶点完全一致，`ret pc/app/mtk = 0/0/0`）。
     * 单帧核心自检：**`GEMINI_EIS_SMOKE_PASS`**（MPC 平均解算耗时 $36.28\,\mu\text{s}$，视觉残差因果重投影计算平稳）。
     * 全量 CTest 测试：**100% 全部通过** (3/3 Passed, 0.12s)。
   * 完成本地 Git 提交 (`d7c4426`)。

4. **2026-10-22 (阶段 3 进行中：调度器与 LFGO 重构)**：
   * **Two-Tier LFGO 引导观测器**：重构 `IntentScheduler1D`，实现二阶因果引导状态 $(\theta_{\text{ref}}, \omega_{\text{ref}})$。Hold 稳态强阻尼滤波，Follow 追踪态敏捷跟随（$\tau \approx 0.085\,\text{s}$），弹性 Crop 软引力拉扯防溢出。
   * **Oscillation V2 步频抑制**：建立 $1.4\text{--}2.6\,\text{Hz}$ 半周期特征穿越统计与方向相干比（$\text{Coherence}$），在剧烈步频颠簸下自适应拉高加加速度阻尼 $w_{\text{jerk}}$ 与位置跟踪 $w_{\text{pos}}$。
   * **急停超调拦截状态机**：引入 `IntentState::StopSettle` 与单向位移锁定，角速度衰减至 0，彻底切断急停反弹拉扯（超调 $< 0.5^\circ$）。
   * **解耦贯通**：`GeminiEisCore` 向下层 MPC 输入滤波速度 `referenceVelocity`，消除强行拟合高频抖动噪声缺陷。
   * **实测验证**：
     * `gemini_eis_c_api_test.exe`：**`GEMINI_EIS_3END_PARITY_PASS`**。
     * `gemini_eis_sim.exe --self-test`：**`GEMINI_EIS_SMOKE_PASS`**（单轴求解耗时 $42.23\,\mu\text{s}$）。
     * `gemini_eis_synth.exe`：**`GEMINI_EIS_SYNTH_PASS`**（`maxTargetStep` 优化至 $0.000529\,\text{rad}$，阶跃平滑度提升 3 倍）。
     * CTest 全量通过 (3/3 Passed, 0.14s)。
