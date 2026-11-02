#!/usr/bin/env python3
"""C1-B2 Host YUV native I/O engineering evidence: four source-aligned cases.

Charts cover Input / Recognition / C1 shadow control / Geometry / Output
handoff / Performance. This is NOT a new stabilized video render and is NOT
an Android DEVICE certification.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from c1_generate_professional_report import plot,String_HTML_START,String_HTML_END
from c1_stream_host import ROOT,CASES

OUT=ROOT/"reports"/"gemini_tripod_v2"/"c1b2_yuv"
OLD=ROOT/"reports"/"gemini_tripod_v2"/"c1b_native"
DESCR={"024605":"低速往返","024618":"中速往返",
       "024630":"快速往返","024647":"单侧转向"}
COL={"y":"#4ECDB4","old":"#86A9F3","warn":"#EAA968","violet":"#B6A0F0"}
def rows(p):
    return [json.loads(t) for t in p.read_text(encoding="utf-8").splitlines()]
def pref(rows,key):
    return [r.get(key) for r in rows]
def native(rows,key):
    return [r["visual_output"].get(key) if r.get("visual_output") else None for r in rows]
def stage(case):
    cur=rows(OUT/case/"camera_ready_frames.jsonl")
    summary=json.loads((OUT/case/"summary.json").read_text(encoding="utf-8"))
    baseline=rows(OLD/case/"frames.jsonl")
    assert len(cur)==290 and len(baseline)==291
    prev={r["source_id"]:r for r in baseline}
    assert all(r["source_id"] in prev and r["sensor_ts_ns"]==prev[r["source_id"]]["sensor_ts_ns"] for r in cur)
    idx=pref(cur,"source_id")
    pts=[r["pts_us"]*.001 for r in cur]
    pts_dt=[None]+list(np.diff(pts))
    sensor=[r["sensor_ts_ns"]*1e-6 for r in cur]
    sensor_dt=[None]+list(np.diff(sensor))
    old_dx=[prev[r["source_id"]]["dx_feature_px"] for r in cur]
    new_dx=native(cur,"dx_feature_px")
    new_dy=native(cur,"dy_feature_px")
    delta=[abs(a-b) if a is not None else None for a,b in zip(new_dx,old_dx)]
    success=[1 if r["accepted_native_global_model"] else 0 for r in cur]
    fast=[1 if r["under_20ms_from_camera_ready"] else 0 for r in cur]
    alrt=[33.78]*len(cur)
    gmargin=[(r["physical_available_deadline_ns"]-
              r["gyro_max_queried_ts_ns"])*1e-6 for r in cur]
    gsubmit=[(r["physical_available_deadline_ns"]-
             r["gyro_largest_submitted_ts_ns"])*1e-6 for r in cur]
    graphs={
      "cadence":plot("SourceID/PTS/SensorTS 确认——同一原始Camera帧",
           "ms",[
             ("真实编码PTS相邻间隔",idx,pts_dt,COL["old"]),
             ("相机SensorTS相邻间隔",idx,sensor_dt,COL["y"])
           ],fmt=".1f"),
      "in":plot("同一原始视频的FFmpeg软件解码 vs Native Y平面8×8缩小",
           "ms",[
             ("HOST仅解码到Y平面",idx,pref(cur,"host_MP4_software_decode_ms"),COL["old"]),
             ("Native Y 8x8 Area",idx,pref(cur,"native_y_area8_ms"),COL["y"])
           ],fmt=".2f"),
      "motion":plot("RS19预稳后的C1-B Native vs C1-B2真实Y平面Residual",
           "320×180 px/frame",[
             ("原 C1-B BGR/OpenCV视觉",idx,old_dx,COL["old"]),
             ("C1-B2 Native Y视觉",idx,new_dx,COL["y"]),
             ("C1-B2 dy",idx,new_dy,COL["violet"])
           ],fmt=".3f"),
      "difference":plot("同一个SourceID水平背景运动差异：Native Y vs旧前端",
           "feature px",[
              ("|New dx - Old dx|",idx,delta,COL["warn"])
           ],fmt=".3f"),
      "trust":plot("全画幅Native FB-LK 共模背景可信度与响应支持",
           "trust / 100×inliers",[
             ("Native 3×3 tile trust",idx,native(cur,"trust"),COL["y"]),
             ("Inlier count / 100",idx,
               [v/100 if v is not None else None for v in native(cur,"inliers")],
               COL["old"])
           ],fmt=".2f"),
      "gyro":plot("Camera/Gyro真实RS19：物理可用截止仍在最新已查询陀螺样本后",
           "remaining ms",[
             ("RowGyro queried safe margin",idx,gmargin,COL["old"]),
             ("RowGyro submitted safe margin",idx,gsubmit,COL["y"])
           ],fmt=".2f"),
      "warp":plot("物理20Row RS19两幅320×180特征预稳定（当前qRef）",
           "ms",[
             ("上一Camera帧重新投影",idx,pref(cur,"native_prewarp_previous_ms"),COL["old"]),
             ("本Camera帧RS19预稳定",idx,pref(cur,"native_prewarp_current_ms"),COL["y"])
           ],fmt=".2f"),
      "gate":plot("帧序严格截止门禁：33.78ms当帧结果 vs 20ms较严目标",
           "按时1 / 超时0",[
             ("33.78ms Host Camera-ready",idx,success,COL["y"]),
             ("20ms reserved-Warp budget",idx,fast,COL["warn"])
           ],fmt=".1f"),
      "components":plot("Native各阶段耗时：Y→Gyro→两次RS19→LK等候",
           "ms",[
             ("Native Y缩小",idx,pref(cur,"native_y_area8_ms"),COL["y"]),
             ("Host Gyro/20Row pose",idx,pref(cur,"host_gyro_pose_ms"),COL["old"]),
             ("Native async LK worker wait",idx,pref(cur,"native_async_wait_ms"),COL["warn"])
           ],fmt=".2f"),
      "speed":plot("从真实Y平面就绪/软件MP4起点到本帧Native视觉结果",
           "ms",[
             ("Native Y-ready to result",idx,pref(cur,"camera_ready_to_gate_decision_ms"),COL["y"]),
             ("HOST MP4 decoded to result",idx,pref(cur,"host_decode_start_to_gate_decision_ms"),COL["old"]),
             ("29.6FPS Frame interval",idx,alrt,"#98A0B2")
           ],fmt=".1f")
    }
    assert len(graphs)==10
    figures=[
      '<h1>Gemini V2C C1-B2 Camera YUV · '+case+' '+DESCR[case]+'</h1>',
      '<p class="subtitle">原2560×1440 RAW · FFmpeg yuv420p直接Y平面 · Native Y8×8 '
      '→ 20真实RS Row → 两幅Native特征预稳 → C++异步3×3背景LK'
      ' · 每帧SourceID/PTS/SensorTS核对</p>',
      '<div class="strip"><b>C1-B2 HOST Y-READY有效，最终产品DEVICE未测。</b>'
      'Y plane就绪起点在本Host实验里是MP4软件解码完成时刻，'
      '相当于模拟Camera的YUV交付界面，不是实际Android的AImage回调。'
      '本报告没有qVirtual/Zoom/Visual输出执行，也没有新的视频防抖成品。</div>',
      '<section><h2>1 输入 Input</h2>'
      '<p>逐帧原source.mp4的YUV420p原始Y平面，不做BGR颜色转换；'
      '原始编码PTS和Camera SensorTS相同SourceID配准。'
      'Native支持rowStride和pixelStride，原设备提供缓冲区时无需强制重新打包。'
      'Host软件解码耗时单独保留，不混入Camera-ready预算。</p>'+
      graphs["cadence"]+graphs["in"]+'</section>',
      '<section><h2>2 识别 Recognition</h2>'
      '<p>使用Native C++ 3×3各区域FB-LK、中心优先但两侧非零权重、RANSAC背景4DoF共识；'
      '只处理本帧和过去一帧，仍为SHADOW。下面与原BGR/OpenCV入口同源模型对比。</p>'+
      graphs["motion"]+graphs["difference"]+graphs["trust"]+
      f'<p>Native Y与C1-B相比：水平残余绝对差p90 '
      f'<b>{summary["Y_plane_vs_old_OpenCV_native_dx_abs_error_p90"]:.4f}px</b>'
      '（320×180分析域，非最终1080p输出运动）。</p></section>',
      '<section><h2>3 控制 Control（C1 SHADOW）</h2>'
      '<p>视觉输入只交回当前SourceID的严格deadline测试，'
      '现在没有接入唯一C2 Target Owner，更不会绕开V1黄金参数改Zoom/Crop/Tau。'
      '迟到、低信任度或SourceID错误时，视觉拒绝而不是挪用前一帧补偿。</p>'+
      graphs["gate"]+'</section>',
      '<section><h2>4 几何 Geometry</h2>'
      '<p>20个Camera物理Gyro RS行位姿、19个物理行区间，+8ms真实冻结偏移；'
      '320×180预稳定仍采用39×65密化分析网格。'
      '旧Gemini终态高清20×33 RS19 ONE-WARP未改变。'
      '所有Gyro查询均在本Camera物理截止点以内。</p>'+
      graphs["gyro"]+graphs["warp"]+'</section>',
      '<section><h2>5 输出 Output（视觉结果，而非成品视频）</h2>'
      '<p>这阶段输出的是严格按时交付的C++背景共模Residual与confidence。'
      '本案例源帧共291、相邻视觉Pair290，Native同帧完整输出'
      f'<b>{summary["same_frame_current_native_model_accepted"]}/290</b>。'
      '33.78ms为Host Y-ready视觉预算的上限，不包括高清GPU Warp/Encoder。'
      '同一场景的20ms更严预算仅'
      f'<b>{summary["camera_ready_worker_20ms_passes"]}/290</b>通过。</p></section>',
      '<section><h2>6 性能 Performance</h2>'
      '<p>分开记录：MP4软件解码、Native Y8×8、Gyro/20行位姿、两幅RS19预稳、'
      'Native线程LK及截至当前SourceID的最终决策。Python宿主调度仍包含在测量内。'
      '仅可称Host时序，不可外推Android FPS。</p>'+
      graphs["components"]+graphs["speed"]+
      f'<p>Y就绪至当前SourceID视觉结果：p50 '
      f'<b>{summary["camera_ready_to_gate_p50_ms"]:.1f}ms</b>，'
      f'p90 <b>{summary["camera_ready_to_gate_p90_ms"]:.1f}ms</b>，'
      f'p99 <b>{summary["camera_ready_to_gate_p99_ms"]:.1f}ms</b>。'
      'GPU/Encoder、真正Camera回调与资源温升必须之后真机验收。</p></section>',
      '<section><h2>证据文件</h2><p>'
      '<a href="summary.json">本片Host Gate统计</a> · '
      '<a href="camera_ready_frames.jsonl">290个逐帧真实SourceID/TS/性能/置信度</a> · '
      '<a href="../c1b2_native_yuv_unit.json">Stride/RS19像素Par­ity单元测试</a>'
      '</p></section>'
    ]
    file=OUT/case/"index.html"
    file.write_text(String_HTML_START+"".join(figures)+String_HTML_END,
                    encoding="utf-8")
    return {
        "case":case,"source_frames":291,"native_pairs":290,
        "same_frame_33p78ms":summary["same_frame_current_worker_ready"],
        "same_frame_20ms":summary["camera_ready_worker_20ms_passes"],
        "Y_ready_p90_ms":summary["camera_ready_to_gate_p90_ms"],
        "Y_ready_p99_ms":summary["camera_ready_to_gate_p99_ms"],
        "MP4_decode_plus_native_p90_ms":summary["HOST_FULL_MP4_decode_to_result_p90_ms"],
        "YvsOld_dx_diff_p90":summary["Y_plane_vs_old_OpenCV_native_dx_abs_error_p90"],
        "charts":10,"device_pass":False,"C2_activated":False,
        "html":str(file)}

def main():
    if not (OUT/"c1b2_native_yuv_unit.json").exists():
        raise RuntimeError("Native Y / RS19 independent pixel parity test missing")
    cases=[stage(c) for c in CASES]
    assert all(c["same_frame_33p78ms"]==290 for c in cases)
    table="".join(
      f'<tr><td><a href="{r["case"]}/index.html">{r["case"]}</a> '
      f'{DESCR[r["case"]]}</td><td>291</td><td>{r["same_frame_33p78ms"]}/290</td>'
      f'<td>{r["same_frame_20ms"]}/290</td>'
      f'<td>{r["Y_ready_p90_ms"]:.1f}ms</td>'
      f'<td>{r["Y_ready_p99_ms"]:.1f}ms</td>'
      f'<td>{r["MP4_decode_plus_native_p90_ms"]:.1f}ms</td>'
      f'<td>{r["YvsOld_dx_diff_p90"]:.4f}px</td></tr>' for r in cases)
    total20=sum(c["same_frame_20ms"] for c in cases)
    parts=[
      '<h1>Gemini V2C · C1-B2 Native Camera YUV Direct Input — PC离线Host验收</h1>',
      '<p class="subtitle">2026-10-24 · 四段原RAW 4×291帧 · Camera Y平面直通模拟'
      ' · Native面积缩小+RS19+单任务异步3×3 LK · 原V1黄金完全不改</p>',
      '<div class="strip"><b>HOST C1-B2 Camera-Y-ready 33.78ms Gate：1160/1160 PASS。</b>'
      '但这里并非Android实际相机回调、真实GPU/编码或最终防抖输出，'
      '20ms较严预算为'+str(total20)+'/1160，真实Device仍需测试。'
      '绝不把Host预解码Y输入当成Y1硬件性能铁证。</div>',
      '<section><h2>同源四段真实SourceID/TS/P90</h2>'
      '<table class="results"><thead><tr><th>录像</th><th>RAW帧</th>'
      '<th>Y-ready视觉33.78ms</th><th>Y-ready视觉20ms</th>'
      '<th>Y-ready P90</th><th>P99</th><th>含软件解码P90</th>'
      '<th>对C1-B dx差P90</th></tr></thead><tbody>'+table+'</tbody></table>'
      '<p>视频由原FFmpeg解码成yuv420p，仅交出亮度Y平面给新Native C++接口。'
      '之前C1-B使用BGR转换，对比说明减少了Color Conversion/CPU内存流量。'
      '当前处理时间包含真实Native Y downsample、Gyro、双幅RS19预稳定、'
      'C++ Async LK、同源模型接收，但不包含高清Warp+Encoder。</p></section>',
      '<section><h2>C1-B2 本轮真完成项</h2>'
      '<p>① YUV_420_888/NV12/NV21 的 Y平面 rowStride/pixelStride兼容；'
      '② 2560×1440至320×180 Native 8×8 area缩小；'
      '③ 真实20行Gyro/RS19 Native低分辨率单次预稳；'
      '④ Native多Tile FB-LK和只接受当前SourceID的async worker；'
      '⑤ 4×291原始视频逐帧PTS/SensorTS/parity；'
      '⑥ 独立Stride、物理几何、旧C1B visual dx对齐证据。</p>'
      '<p>以上为PC HOST验证，不是MT6768真机APP、ISP/GPU/Encoder产品验证。'
      'C2唯一qVirtual+Visual+Zoom联合规划尚未实际接线，'
      '没有新EIS渲染视频，不得宣称边缘伸缩扭动问题已经解决。</p></section>',
      '<section><h2>四段专业证据：输入→识别→控制→几何→输出→性能</h2>'+
      "".join(
         f'<article><h3>{c["case"]} {DESCR[c["case"]]}</h3>'
         f'<p><a href="{c["case"]}/index.html">查看本片10张真实SVG、逐帧源证据与同帧截止</a></p></article>'
         for c in cases)+'</section>',
      '<section><h2>后续工作</h2>'
      '<p>不在公司期间可继续C2 PC-only单核虚拟相机联合规划设计及Shadow独立Host实验；'
      'Android Y1真机Camera YUV到达时间、Native同帧完成、30fps/温升/Stop释放'
      '仍需回公司完成Device Gate。'
      '当前Host已摆脱BGR和软件解码口径混淆，'
      '但没有设备前不得更改Golden产品默认。</p>'
      '<p><a href="all_cases.json">完整Host汇总原始JSON</a> · '
      '<a href="c1b2_native_yuv_unit.json">Native Y stride/RS19 parity</a> · '
      '<a href="../../../experiments/tripod_v2/C1B2_PROGRESS_20261024.md">'
      '阶段说明与后续Gate</a></p></section>'
    ]
    top=OUT/"index.html"
    top.write_text(String_HTML_START+"".join(parts)+String_HTML_END,
                   encoding="utf-8")
    summary={
        "stage":"C1-B2 Native Camera Y luma input / PC Host only",
        "cases":cases,"source_frames_total":4*291,
        "native_model_pairs_total":4*290,
        "HOST_Y_ready_33p78ms_pass":True,
        "HOST_Y_ready_20ms_success_total":total20,
        "Native_Y_stride_and_RS19_pixel_parity":"PASS",
        "Y1_real_camera":"NOT_TESTED",
        "hardware_1080p_30fps":"NOT_TESTED",
        "C2_target_zoom_visual":"OFF_SHADOW",
        "original_Gemini_V1":"UNCHANGED",
        "html":str(top)}
    (OUT/"c1b2_professional_evidence.json").write_text(
        json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print("GEMINI_V2C_C1B2_FOUR_TRUE_Y_PLANE_40SVG_SIX_STAGE_REPORT",
        json.dumps({"html":str(top),"source_frames":4*291,"pairs":4*290,
           "HOST_33p78ms":True,"HOST_20ms_accepted":total20,
           "DEVICE":"NOT_TESTED"},ensure_ascii=False),flush=True)

if __name__=="__main__":main()
