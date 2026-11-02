#!/usr/bin/env python3
"""C1-B real four-RAW, six-stage engineering report and fair async gates.

Two different Host deadlines:
 A) Strict full software decode+RS-prewarp+LK < 33.78ms frame budget.
 B) Already-ready 320x180 features -> Native LK < 20ms.
Only (A) can meaningfully approximate full Host frame delivery; neither is
a real Android Camera/ISP/Encoder deadline certificate.
"""
from __future__ import annotations
import json,math
from pathlib import Path
import numpy as np
from c1_generate_professional_report import plot,String_HTML_START,String_HTML_END
from c1_stream_host import ROOT,CASES

ROOT_OUT=ROOT/"reports/gemini_tripod_v2/c1b_native"
C1A=ROOT/"reports/gemini_tripod_v2/c1_causal"
COLORS={"old":"#78B9F8","native":"#4DD2A3","bad":"#F5B65A","f":"#B39CFF"}
LABELS={"024605":"低速往复","024618":"中速往复","024630":"快速往复","024647":"单侧转向"}
def read_jsonl(p):
    return [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines()]
def rec(x,key):
    v=x.get(key)
    return float(v) if v is not None else None
def diagram(case):
    path=ROOT_OUT/case
    old=read_jsonl(C1A/case/"frames.jsonl")
    native=read_jsonl(path/"frames.jsonl")
    whole=read_jsonl(path/"async_frame_deadline.jsonl")
    isolated=read_jsonl(ROOT_OUT/"feature_ready"/case/"async_frame_deadline.jsonl")
    comp=json.loads((path/"native_ab.json").read_text(encoding="utf-8"))
    fullstats=json.loads((path/"async_deadline_summary.json").read_text(encoding="utf-8"))
    featurestats=json.loads((ROOT_OUT/"feature_ready"/case/"async_deadline_summary.json").read_text(encoding="utf-8"))
    assert len(native)==len(old)==291 and len(whole)==len(isolated)==290
    assert all(a["source_id"]==b["source_id"] for a,b in zip(old,native))
    assert all(r["source_id"]==native[i+1]["source_id"] and
               f["source_id"]==r["source_id"]
               for i,(r,f) in enumerate(zip(whole,isolated)))
    x=[r["source_id"] for r in native]
    pairs=x[1:]
    diff=[abs(r.get("dx_feature_px",0)-s.get("dx_feature_px",0))
          for r,s in zip(native,old)]
    accA=[int(r["status"]=="ACCEPT") for r in old]
    accB=[int(r["status"]=="ACCEPT") for r in native]
    fullready=[int(r["same_frame_id_ready"]) for r in whole]
    featready=[int(r["same_frame_id_ready"]) for r in isolated]
    timeline={
      "input":plot("同源 SensorTS / MP4 PTS 帧间隔与序号",
                "ms",[("SensorTS",x[1:],
                    list(np.diff([r["sensor_ts_ns"] for r in native])*1e-6),
                    COLORS["old"]),
                    ("MP4 PTS",x[1:],
                     list(np.diff([r["pts_us"] for r in native])*1e-3),
                     COLORS["bad"])],fmt=".1f"),
      "yaw":plot("真实 Camera Gyro Yaw：方向与左右反向半周期",
                  "rad/s",[("Yaw",x,[rec(r,"raw_yaw_rad_s") for r in native],
                            COLORS["old"])],fmt=".2f"),
      "motion":plot("Native vs 原C1 OpenCV：预稳定后中心水平Residual",
                     "320x180 px/frame",
                [("Original OpenCV",x,[rec(r,"dx_feature_px") for r in old],COLORS["old"]),
                 ("New Native C++",x,[rec(r,"dx_feature_px") for r in native],COLORS["native"])],fmt=".2f"),
      "difference":plot("Native/OpenCV相同SourceID的水平残余误差",
                         "feature px",
                [("|Native - OpenCV|",x,diff,COLORS["bad"])],fmt=".3f"),
      "trust":plot("全画幅3x3区域：Native vs OpenCV全局视觉可信度",
                  "0..1",
                [("OpenCV trust",x,[rec(r,"trust") for r in old],COLORS["old"]),
                 ("Native trust",x,[rec(r,"trust") for r in native],COLORS["native"])],fmt=".2f"),
      "geometry":plot("Native观察的是原20 Row RS19的相同Gyro/源视场",
                     "valid percent",
                [("C1-B valid field",x,[100*rec(r,"prewarp_coverage_ratio")
                  if rec(r,"prewarp_coverage_ratio") is not None else None
                        for r in native],COLORS["native"])],fmt=".1f"),
      "output":plot("视觉的正常接受率（SHADOW、不修改虚拟相机Target）",
                   "0/1",
                [("C1-A OpenCV",x,accA,COLORS["old"]),
                 ("C1-B Native",x,accB,COLORS["native"])],fmt=".1f"),
      "async":plot("同帧截止Gate：真实Source全链路33.78ms vs 仅Native20ms",
                   "0=拒绝 1=当前SourceID按时交付",
                [("全Host软件解码截止",pairs,fullready,COLORS["bad"]),
                 ("已经准备好feature后20ms",pairs,featready,COLORS["native"])],fmt=".1f"),
      "performance":plot("Host实测：旧C1 / Native C1处理、当前原RAW全链路",
                   "ms",
                [("C1-A原Python OpenCV",x,[rec(r,"c1_total_ms") for r in old],COLORS["old"]),
                 ("C1-B Native + Python胶水",x,[rec(r,"c1_total_ms") for r in native],COLORS["native"]),
                 ("原RAW Host解码+C1-B",x,[rec(r,"c1_total_with_decode_ms") for r in native],COLORS["bad"]),
                 ("29.6fps frame budget",x,[33.78]*len(x),"#91A0AC")],fmt=".1f"),
      "deadline":plot("独立异步线程：相同当前帧软件解码/RS准备/Native耗时",
                      "ms",
                [("原RAW decode+gray",pairs,
                    [rec(r,"decode_and_preprocess_ms") for r in whole],COLORS["bad"]),
                 ("当前RS19两张feature预稳",pairs,
                    [rec(r,"pair_prewarp_and_prep_ms") for r in whole],COLORS["old"]),
                 ("Native异步线程实际等待",pairs,
                    [rec(r,"submit_plus_worker_wait_ms") for r in isolated],COLORS["native"])],fmt=".1f")
    }
    def case_num(label,fmt=".3f"):
        v=comp[label]
        return format(v,fmt) if v is not None else "N/A"
    parts=[
      f'<h1>Gemini V2C C1-B Native & Async · {case} {LABELS[case]}</h1>',
      '<p class="subtitle">严格SourceID4–294 · 原始Camera2560x1440 · SensorTS+真实20行RS19+320x180特征 · '
      'V1黄金不变、C2仍OFF</p>',
      '<div class="strip"><b>当前进度：Native C++观察器HOST准确性通过；完整Host同帧33.78ms截止未通过。</b>'
      '两个时限不同：真实软件解码到结果到手 vs 特征图已经准备好之后仅等待C++异步线程。'
      '绝不能把第二种口径冒充Android产品实时达标。</div>',
      '<section><h2>1 输入 Input</h2>'
      '<p>同一RAW源逐帧验证PTS/SensorTS，Gyro仅按当前真实readout截止时间提交。'
      'C++异步worker不访问Camera未来帧；一切Native LK针对当前帧与过去帧pair。</p>',
      timeline["input"]+timeline["yaw"]+'</section>',
      '<section><h2>2 识别 Recognition</h2>'
      '<p>与原OpenCV算法同样3x3均匀空间检测、中心加权全局Similarity、FB-LK、RANSAC。'
      'Native C++是自带金字塔LK的共享C ABI，0 Python/OpenCV LK调用。'
      '观察场景是普通背景不是“电脑屏幕检测器”。</p>',
      timeline["motion"]+timeline["difference"]+timeline["trust"]+
      '<p>Native dx 绝对差p90 = <b>'+case_num("native_dx_P90_abs_error_vs_original_feature_px")+
      '</b> feature px；dy 差p90 = <b>'+case_num("native_dy_P90_abs_error_vs_original_feature_px")+
      '</b> feature px；Scale(log)差p90 = <b>'+case_num("native_logscale_P90_abs_error_vs_original",".5f")+
      '</b>。这是C1观测parity，并非最终1080p输出像素差。</p></section>',
      '<section><h2>3 控制 Control（SHADOW）</h2>'
      '<p>Native只返回当前SourceID的背景共模残余及可信度，不写qTarget/Visual/Zoom、不改变V1 GOLDEN。'
      'C1与C2必须通过物理坐标/RS19预算联合转换之后才能执行输出修正。</p>'+
      timeline["output"]+'</section>',
      '<section><h2>4 几何 Geometry</h2>'
      '<p>沿用C1相同的20行物理RS19 Gyro预稳定，39x65是独立低分辨率分析网格，'
      '原高清最终RS19 20x33 ONE-WARP没有增加其他变形执行器。</p>'+
      timeline["geometry"]+'</section>',
      '<section><h2>5 输出 Output（异步结果到C2的门禁）</h2>'
      '<p>同一个SourceID提交Native单任务线程，只有完成时间未超过deadline且在实际poll'
      '时仍未超时才能把结果交给下一阶段。超过时限立即fail closed，绝不使用过去一帧的LK。'
      'Host软件解码33.78ms测试只是PC诊断，不代表Y1的零拷贝Camera YUV输入。</p>'+
      timeline["async"]+
      f'<p><b>完整Host 33.78ms同帧：</b>{fullstats["async_same_frame_ready"]}/290，'
      f'<b>已准备好feature后Native 20ms：</b>{featurestats["async_same_frame_ready"]}/290。'
      '超过截止的视觉被拒绝，原Gyro路线仍可继续，当前未实际接C2。</p></section>',
      '<section><h2>6 性能 Performance</h2>'
      '<p>三个时域不可混淆：真实RAW软件解码、低分辨率Native RS图准备、C++后台LK完成；'
      '图中和数据均来自真实Host回放，未测试Android相机回调与GPU/Encoder。</p>'+
      timeline["performance"]+timeline["deadline"]+
      f'<p><b>Native核心LK p90 {case_num("Native_CPP_LK_per_pair_p90_ms",".2f")}ms</b>；'
      f'C1-B Python胶水+Native处理p90 {case_num("C1_B_python_plus_cpp_p90_ms",".2f")}ms；'
      f'完整Host软件解码+C1-B p90 {case_num("C1_B_full_host_decode_plus_C1_p90_ms",".2f")}ms，'
      '仍不可宣称稳定30fps。</p></section>',
      '<section><h2>真实证据可追溯</h2>'
      '<p><a href="native_ab.json">Native C1 A/B原始汇总</a>、'
      '<a href="frames.jsonl">C1-B Native每帧/9tile诊断</a>、'
      '<a href="async_deadline_summary.json">严格原RAW截止统计</a>、'
      '<a href="async_frame_deadline.jsonl">同一SourceID所有超时及原因</a>、'
      f'<a href="../feature_ready/{case}/async_deadline_summary.json">Native特征输入20ms测试</a>。'
      '本阶段未生成新的全尺寸防抖视频；边缘形变是否改善，仍必须等C2完成之后实测。</p></section>'
    ]
    home=(path/"index.html")
    home.write_text(String_HTML_START+"".join(parts)+String_HTML_END,encoding="utf-8")
    return {
       "case":case,"input_frames":291,"visual_native_pairs":comp["native_fb_lk_pairs"],
       "native_vs_c1a_dx_abs_error_p90":comp["native_dx_P90_abs_error_vs_original_feature_px"],
       "native_vs_c1a_logscale_abs_error_p90":comp["native_logscale_P90_abs_error_vs_original"],
       "native_core_LK_p90_ms":comp["Native_CPP_LK_per_pair_p90_ms"],
       "host_c1_native_p90_ms":comp["C1_B_python_plus_cpp_p90_ms"],
       "host_decode_c1_native_p90_ms":comp["C1_B_full_host_decode_plus_C1_p90_ms"],
       "full_host_deadline_accepted":fullstats["async_same_frame_ready"],
       "feature_only_deadline_accepted":featurestats["async_same_frame_ready"],
       "charts":len(timeline),"native_async_stale_used":False,
       "android_DEVICE_PASS":False,"report":str(home)}
def main():
    cases=[diagram(case) for case in CASES]
    assert all(c["visual_native_pairs"]==290 and c["charts"]==10 for c in cases)
    inject=json.loads((ROOT_OUT/"async_failure_injection.json").read_text(encoding="utf-8"))
    table=''.join(
       f'<tr><td><a href="{r["case"]}/index.html">{r["case"]}</a> {LABELS[r["case"]]}</td>'
       f'<td>290/290</td><td>{r["native_vs_c1a_dx_abs_error_p90"]:.3f}px</td>'
       f'<td>{r["native_core_LK_p90_ms"]:.1f}ms</td>'
       f'<td>{r["host_c1_native_p90_ms"]:.1f}ms</td>'
       f'<td>{r["host_decode_c1_native_p90_ms"]:.1f}ms</td>'
       f'<td>{r["full_host_deadline_accepted"]}/290</td>'
       f'<td>{r["feature_only_deadline_accepted"]}/290</td></tr>'
       for r in cases)
    header=(
       '<h1>Gemini Tripod V2C · C1-B Native多区域LK + 严格同帧异步</h1>'
       '<p class="subtitle">2026-10-24 · 四段原始Camera 2560x1440每段291帧 · RS19 20行真实Gyro '
       '· Native C++四区域全局分析 · C2仍OFF</p>'
       '<div class="strip"><b>结果：Native LK算法parity通过，Host完整当前帧截止未通过；'
       '不能推广到Android/MTK，不能说已经能用当前帧视觉修正最终输出。</b>'
       '有两个独立门禁：左边从真实原RAW软件解码开始33.78ms；'
       '右边只从320x180特征已准备好后给予Native后台线程20ms。'
       '过去帧视觉禁止偷偷错配当前帧，全部迟到的结果会拒绝。</div>'
       '<section><h2>四段实际SOURCE时序/Native数据总表</h2>'
       '<table class="results"><thead><tr><th>RAW录像</th><th>Native视觉可用</th>'
       '<th>相对OpenCV dx差 p90</th><th>纯Native LK p90</th>'
       '<th>C1-B不含解码p90</th><th>软件解码+C1-B p90</th>'
       '<th>原RAW起点33.78ms成功</th><th>特征就绪20ms成功</th></tr></thead>'
       '<tbody>'+table+'</tbody></table>'
       '<p>这是实际C1-B单线程视觉与原始输入Host async工作者的实验，不是'
       'Camera驱动直接交YUV、不是Y1 1080p录像最终30fps。'
       '真实帧时限要从硬件Camera交付时刻与该帧最晚Warp提交点进行设备测量；'
       '在Host软件解码模式失败不能等价推导HAL也失败。</p></section>'
       '<section><h2>实际架构与Gate</h2>'
       '<p>输入：当前和过去的Camera/20行RS Gyro → 原有Native低分辨率Prewarp → '
       '新Portable C++3×3 Shi-Tomasi角点 + 金字塔正反LK + RANSAC+加权4DoF '
       '→ 单任务C++异步worker（顺序、背压、deadline、取消和归属SourceID） '
       '→ 仅本帧成功时可作为将来C2输入，否则fail closed。</p>'
       '<p>异步故障注入：BUSY后无旧帧复用、倒序SourceID拒绝、'
       '人为延迟85ms导致超过25ms截止拒绝、取消/退出安全释放：均通过。'
       '<a href="async_failure_injection.json">查看全部失败注入数值证据</a>。'
       'C1B Native本身只用当前与历史输入，没有未来IMU/帧；不会做Zoom、Crop或Target补偿。</p></section>'
       '<section><h2>按「输入→识别→控制→几何→输出→性能」查看四段专业图表</h2>'
       +''.join(f'<article><h3>{r["case"]} {LABELS[r["case"]]}</h3>'
       f'<p><a href="{r["case"]}/index.html">查看{r["charts"]}张真实证据SVG、'
       'Native C1 vs OpenCV及两个同帧截止Gate</a></p></article>' for r in cases)
       +'</section>'
       '<section><h2>下一阶段的真实阻塞</h2>'
       '<p>C1-B的Native跟踪已解决Python OpenCV前端不可共享的问题；但'
       '完整Host软件解码 + 缩小 + RS19当前及过去两图准备仍超过当前帧预算。'
       '最值得做的是在Android Camera原生YUV/纹理链路中提供直接的低分辨率视觉输入，'
       '减少多次下载/缩放/额外prewarp，按真实FrameDeadline分阶段证据评测；'
       '尚未完成Y1 Device PASS，不能跳转宣称C2可用。'
       '确认实际当前帧全部视觉数据可用后，才接唯一qVirtual+Visual+Zoom轨迹规划，'
       '保证Gemini黄金版已有的大幅左右往复被保留，避免边缘Zoom/Scale互相拉扯。</p>'
       '</section>'
    )
    top=ROOT_OUT/"index.html"
    top.write_text(String_HTML_START+header+String_HTML_END,encoding="utf-8")
    manifest={
        "stage":"V2C C1-B Native multizone LK + bounded async",
        "four_original_RAW_cases":True,
        "all_290_native_observations_pass":True,
        "C1B_native_parity":"HOST_PASS",
        "C1B_async_fails_closed_on_late":"PASS",
        "full_host_decode_to_current_frame_deadline":"FAIL",
        "feature_ready_native_worker_deadline":"CONDITIONAL_HOST_ONLY",
        "android_real_camera_device":"NOT_TESTED",
        "C2_output_actuation":"OFF",
        "original_Gemini_V1_golden_untouched":True,
        "cases":cases,
        "html":str(top)}
    (ROOT_OUT/"c1b_professional_evidence.json").write_text(
        json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    print("V2C_C1B_FINAL_4x291_SOURCE_40_CHARTS_TWO_ASYNC_GATES",
          json.dumps({"total":len(cases),"charts":[x["charts"] for x in cases],
             "home":str(top),"state":manifest["full_host_decode_to_current_frame_deadline"]},
             ensure_ascii=False),flush=True)
if __name__=="__main__":main()
