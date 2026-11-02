#!/usr/bin/env python3
"""C2-C3 professional evidence from FOUR real RAW sources with output H264 FB-LK.

Charts are from real frames and strictly post-hoc V1 comparisons; never feeds
Golden or future output pixels into the native C3 causal target owner.
"""
from __future__ import annotations
import html,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from c1_generate_professional_report import plot,String_HTML_START,String_HTML_END
from c1_stream_host import ROOT,CASES

RP=ROOT/"reports/gemini_tripod_v2"
OUT=RP/"c3_global_path"
COL={"v1":"#8AB1F4","c2":"#B4A2DE","c3":"#45CFB4","yaw":"#D7B45A","gyro":"#E6A46C"}
DESCR={"024605":"低速往返","024618":"中速往返","024630":"快速往返","024647":"单侧转向"}
def readl(path):
    return [json.loads(x) for x in path.read_text(encoding="utf-8").splitlines()]
def getpix(case,variant):
    return readl(OUT/case/(variant+"_output_4dof_frames.jsonl"))
def graph(title,unit,fields,fmt=".2f"):
    return plot(title,unit,fields,fmt=fmt)
def havevideo(case,variant):
    return (RP/variant/case/"c2_joint_visual_output.mp4").exists()
def phase(states):
    # Reconstruct the SAME causal yaw sign and halfcycle from saved physical
    # C3 yaw_rad_s, not from V1 future or offline video.
    fast=0.;prevts=None;sign=0;count=0;reversals=[];counts=[]
    for f in states:
        t=f["sensor_ts_ns"]
        dt=1/29.605 if prevts is None else min(.12,max(.002,(t-prevts)*1e-9))
        fast+=(1-math.exp(-dt/.055))*(f["yaw_rad_s"]-fast)
        now=1 if fast>.075 else -1 if fast<-.075 else 0
        rev=now!=0 and now!=sign and sign!=0 and count>=3
        if now:
            count=count+1 if now==sign else 1
            sign=now
        else:count=0
        reversals.append(1 if rev else 0)
        counts.append(count*sign)
        prevts=t
    return reversals,counts
def aligned(v,key):
    lookup={int(x["frame"]):x.get(key) for x in v if x["status"]=="ACCEPT"}
    return [lookup.get(i) for i in range(291)]
def case_report(case):
    dir=OUT/case
    trace=readl(RP/"c3_path_4dof"/case/"c2_joint_visual_frames.jsonl")
    c1=readl(RP/"c1b2_yuv"/case/"camera_ready_frames.jsonl")
    v1=getpix(case,"V1");c2=getpix(case,"C2_EDGE");c3=getpix(case,"C3_GLOBAL")
    yaw=getpix(case,"C3_YAW_FOLLOW")
    assert len(trace)==291 and len(c1)==290
    assert all(z["source_id"]==i+4 for i,z in enumerate(trace))
    for i,x in enumerate(c1):
        assert x["source_id"]==i+5 and x["sensor_ts_ns"]==trace[i+1]["sensor_ts_ns"]
    ref=json.loads((RP/"c2_shadow_edge"/case/"pixel_effect_vs_V1.json").read_text(encoding="utf-8"))
    v1p=ref["V1_GOLDEN"]
    c2p=ref["candidates"]["c2_joint_visual"]
    c3p=json.loads((RP/"c3_path_4dof"/case/"pixel_effect_vs_V1.json").read_text(encoding="utf-8"))["candidates"]["c2_joint_visual"]
    yawp=json.loads((RP/"c3_path_yawfollow"/case/"pixel_effect_vs_V1.json").read_text(encoding="utf-8"))["candidates"]["c2_joint_visual"]
    refd={v:json.loads((dir/(v+"_output_4dof_summary.json")).read_text(encoding="utf-8"))
          for v in ("V1","C2_EDGE","C3_GLOBAL","C3_YAW_FOLLOW")}
    yaw_trace=readl(RP/"c3_path_yawfollow"/case/"c2_joint_visual_frames.jsonl")
    assert len(yaw_trace)==291
    frame=list(range(291))
    q=np.array([r["q_camera_xyzw"] for r in trace])
    qt=np.array([r["q_virtual_xyzw"] for r in trace])
    yraw=Rotation.from_quat(q).as_rotvec()[:,1]
    ytar=Rotation.from_quat(qt).as_rotvec()[:,1]
    z=np.array([r["zoom"] for r in trace])
    vr=np.array([r["visual_rotation_rad"] for r in trace])
    halfrev,halfcount=phase(trace)
    getout=lambda data,key:aligned(data,key)
    cr={
      "gyro":graph("Input: Camera yaw与C3因果qVirtual Yaw","radians",
        [("Camera Gyro yaw",frame,yraw.tolist(),COL["gyro"]),
         ("C3 Virtual yaw",frame,ytar.tolist(),COL["c3"])],".3f"),
      "cadence":graph("Input: 真实SensorTS相邻Camera帧间隔","ms",
        [("Camera SensorTS",frame,[None]+list(np.diff([x["sensor_ts_ns"]*1.e-6
              for x in trace])),COL["gyro"])],".2f"),
      "trust":graph("Recognition: C1-B2 Native背景Global LK可信度与采纳","trust",
        [("C1B2 accept/trust",frame,[None]+[float(x["visual_output"]["trust"])
          if x["accepted_native_global_model"] else 0. for x in c1],COL["gyro"])],".2f"),
      "native_roll":graph("Recognition: 320×180预稳定全图Rotation（中心加权非ROI）","deg / frame",
        [("C1 native rotation",frame,[None]+[
          math.degrees(x["visual_output"]["rotation_rad"]) for x in c1],COL["gyro"]),
         ("C3已采纳Visual Δrotation",frame,[None]+[
          math.degrees(v) for v in np.diff(vr)],COL["c3"])],".3f"),
      "halfcycle":graph("Control: 物理Yaw半周期与因果反向事件","signed frames",
        [("Yaw halfcycle signed",frame,halfcount,COL["c3"])],".0f"),
      "yaw":graph("Control: 物理Yaw速度与C3目标姿态每帧步长","raw rad/s + target step rad",
        [("Gyro yaw",frame,[r["yaw_rad_s"] for r in trace],COL["gyro"]),
         ("Virtual step rad",frame,[r["qVirtual_step_rad"] for r in trace],COL["c3"])],".3f"),
      "zoom":graph("Control: 单一C3 qVirtual/Visual/Zoom原子提交 Zoom","zoom ratio",
        [("C3 Global zoom",frame,z.tolist(),COL["c3"]),
         ("C3 YawFollow zoom",frame,[r["zoom"] for r in yaw_trace],COL["yaw"])],".3f"),
      "margin":graph("Geometry: 20×33 RS19全网格物理边界最小余量","source px",
        [("C3 original physical RS19",frame,[r["margin_source_px"] for r in trace],COL["c3"])],".2f"),
      "center_dx":graph("Output TRUE H264: Global水平中心运动（已扣除Roll/Scale）","output px/frame",
        [("V1 GOLDEN",frame,getout(v1,"center_global_dx_px"),COL["v1"]),
         ("C2 Edge",frame,getout(c2,"center_global_dx_px"),COL["c2"]),
         ("C3 Global4DoF",frame,getout(c3,"center_global_dx_px"),COL["c3"]),
         ("C3 YawFollow",frame,getout(yaw,"center_global_dx_px"),COL["yaw"])],".2f"),
      "center_dy":graph("Output TRUE H264: Global垂直中心运动","output px/frame",
        [("V1 GOLDEN",frame,getout(v1,"center_global_dy_px"),COL["v1"]),
         ("C2 Edge",frame,getout(c2,"center_global_dy_px"),COL["c2"]),
         ("C3 Global4DoF",frame,getout(c3,"center_global_dy_px"),COL["c3"]),
         ("C3 YawFollow",frame,getout(yaw,"center_global_dy_px"),COL["yaw"])],".2f"),
      "output_roll":graph("Output TRUE H264: 全图4DoF旋转（这轮主要根因）","degrees/frame",
        [("V1 GOLDEN",frame,[None if x is None else math.degrees(x)
          for x in getout(v1,"rotation_rad")],COL["v1"]),
         ("C2 Edge",frame,[None if x is None else math.degrees(x)
          for x in getout(c2,"rotation_rad")],COL["c2"]),
         ("C3 Global4DoF",frame,[None if x is None else math.degrees(x)
          for x in getout(c3,"rotation_rad")],COL["c3"]),
         ("C3 YawFollow",frame,[None if x is None else math.degrees(x)
          for x in getout(yaw,"rotation_rad")],COL["yaw"])],".3f"),
      "output_scale":graph("Output TRUE H264: Global logScale（区分Zoom/非刚性）","logScale ×1000",
        [("V1 GOLDEN",frame,[None if x is None else 1000*x
          for x in getout(v1,"logscale")],COL["v1"]),
         ("C2 Edge",frame,[None if x is None else 1000*x
          for x in getout(c2,"logscale")],COL["c2"]),
         ("C3 Global4DoF",frame,[None if x is None else 1000*x
          for x in getout(c3,"logscale")],COL["c3"]),
         ("C3 YawFollow",frame,[None if x is None else 1000*x
          for x in getout(yaw,"logscale")],COL["yaw"])],".2f"),
      "parallax":graph("Output TRUE H264: 左右非Global 4DoF解释的独立外围残余",
        "left/right antisymmetric output px/frame",
        [("V1 GOLDEN",frame,getout(v1,"edge_antisymmetric_unmodeled_dx_px"),COL["v1"]),
         ("C2 Edge",frame,getout(c2,"edge_antisymmetric_unmodeled_dx_px"),COL["c2"]),
         ("C3 Global4DoF",frame,getout(c3,"edge_antisymmetric_unmodeled_dx_px"),COL["c3"]),
         ("C3 YawFollow",frame,getout(yaw,"edge_antisymmetric_unmodeled_dx_px"),COL["yaw"])],".3f"),
      "speed":graph("Performance: C1-B2 Native视觉交付+C3 C++联合求解分开记录","Host ms",
        [("C1B2 Y-plane ready to result",frame,[None]+[r["camera_ready_to_gate_decision_ms"] for r in c1],COL["gyro"]),
         ("C3 only 20×33 joint",frame,[r["C2_native_solve_ms"] for r in trace],COL["c3"])],".1f")
    }
    assert len(cr)==14
    def statsrow(label,st,dec,status):
        return (f'<tr><td>{label}</td>'
          f'<td>{st["official_motion_p90_px"]:.3f}</td>'
          f'<td>{st["center_dx_abs_p90"]:.3f}</td>'
          f'<td>{dec["global_rotation_p90_deg"]:.3f}°</td>'
          f'<td>{dec["global_logscale_p90_milli"]:.3f}</td>'
          f'<td>{st["edge_antisymmetric_abs_p90"]:.3f}</td>'
          f'<td>{st["edge_antisymmetric_max_abs"]:.3f}</td>'
          f'<td>{status}</td></tr>')
    table=('<table class="results"><thead><tr><th>版本</th><th>全图p90</th>'
      '<th>中心水平p90</th><th>Global Roll p90</th><th>Global logScale×1000</th>'
      '<th>边缘p90</th><th>边缘最大</th><th>严格对V1画质</th></tr></thead><tbody>'+
      statsrow("V1 GOLDEN",v1p,refd["V1"],"基线")+
      statsrow("C2 Edge",c2p,refd["C2_EDGE"],"FAIL")+
      statsrow("C3 Global4DoF",c3p,refd["C3_GLOBAL"],
               "PASS" if c3p["V1_anti_sway_and_edge_pass"] else "FAIL")+
      statsrow("C3 YawFollow-only",yawp,refd["C3_YAW_FOLLOW"],
               "PASS" if yawp["V1_anti_sway_and_edge_pass"] else "FAIL")+
      '</tbody></table>')
    videos=(
      '<div class="videos">'+
      f'<div><h4>V1 GOLDEN（离线画质参考）</h4><video id="v1" muted controls preload="metadata" src="../../../../rendered_preview_fixed/{case}_GEMINI_STABLE.mp4"></video></div>'+
      f'<div><h4>C2 Edge（旧中心+边缘）</h4><video id="c2" muted controls preload="metadata" src="../../c2_shadow_edge/{case}/c2_joint_visual_output.mp4"></video></div>'+
      f'<div><h4>C3 Global4DoF（全局Rotation路径）</h4><video id="c3" muted controls preload="metadata" src="../../c3_path_4dof/{case}/c2_joint_visual_output.mp4"></video></div>'+
      f'<div><h4>C3 YawFollow（预测只改变Target）</h4><video id="yaw" muted controls preload="metadata" src="../../c3_path_yawfollow/{case}/c2_joint_visual_output.mp4"></video></div>'+
      '</div><p><button onclick="sync()">同帧同步</button> <button onclick="play()">同步播放</button> <button onclick="pause()">全部暂停</button></p>'+
      '<script>function vs(){return [...document.querySelectorAll("video")]};'
      'function sync(){let t=vs()[0].currentTime;vs().forEach(v=>v.currentTime=t)}'
      'function play(){sync();vs().forEach(v=>v.play())}'
      'function pause(){vs().forEach(v=>v.pause())}</script>'
    )
    blocks=[
      f'<h1>V2C C2-C3 · Global Path Motion · {case} {DESCR[case]}</h1>',
      '<p class="subtitle">真实2560×1440 source.mp4，严格291 SourceIDs 4～294，Native C1-B2与C3同一Camera/Gyro SensorTS，最终1280×720 H264单次RS19 Warp</p>',
      '<div class="strip"><b>评价准则仍为冻结V1 GOLDEN。</b>C3是Host-PC SHADOW，不能以局部Roll降低替代全局Motion等全部指标PASS。原V1含离线未来数据，只用于画质参考。</div>',
      '<section><h2>1 输入 Input</h2>'+cr["cadence"]+cr["gyro"]+'</section>',
      '<section><h2>2 识别 Recognition</h2><p>C1-B2中心加权3×3真实背景几何，输出dx/dy/rotation/logScale/trust并与同一SourceID相连。</p>'+
        cr["trust"]+cr["native_roll"]+'</section>',
      '<section><h2>3 控制 Control</h2><p>C3独立C++唯一qVirtual/Visual/Zoom全量联合控制，历史状态只在完整RS19通过后原子提交。'
        'Causal Yaw半周期只从当前/过去Gyro推导；完整可增Zoom的预测版被否决；新YawFollow只对qVirtual作用，为独立Shadow候选，仍需全幅画质Gate。</p>'+
        cr["halfcycle"]+cr["yaw"]+cr["zoom"]+'</section>',
      '<section><h2>4 几何 Geometry</h2><p>实际20行Gyro、19个物理RS区间、660个Mesh顶点硬边界≥2源像素。C3只在一个最终Warp里执行。</p>'+
        cr["margin"]+'</section>',
      '<section><h2>5 输出 Output — 真实视频像素独立4DoF拆解</h2>'+table+
        cr["center_dx"]+cr["center_dy"]+cr["output_roll"]+cr["output_scale"]+cr["parallax"]+videos+'</section>',
      '<section><h2>6 性能 Performance</h2><p>Host独立测量C1 YUV视觉与C3 20×33多候选求解，并未计设备GPU/EGL/Encoder；没有Android Device PASS。</p>'+
        cr["speed"]+'</section>',
      '<section><h2>数据与Gate</h2><p>本片独立Global 4DoF解析：'
        '<a href="V1_output_4dof_frames.jsonl">V1逐帧输出</a>、'
        '<a href="C2_EDGE_output_4dof_frames.jsonl">C2旧输出</a>、'
        '<a href="C3_GLOBAL_output_4dof_frames.jsonl">C3新输出</a>；'
        f'<a href="../../c3_path_4dof/{case}/pixel_effect_vs_V1.json">实际Center/Edge全图H264门禁</a>；'
        f'<a href="../../c3_path_4dof/{case}/c2_joint_visual_frames.jsonl">qVirtual/RS19/Visual/Zoom单核逐帧记录</a>。</p></section>'
    ]
    path=dir/"index.html"
    path.write_text(String_HTML_START+"".join(blocks)+String_HTML_END,encoding="utf-8")
    return {
     "case":case,"html":str(path),
     "raw_camera_frames":291,"native_c1_pairs":290,
     "real_video_variants":["V1","C2_EDGE","C3_GLOBAL","C3_YAW_FOLLOW"],
     "plots":len(cr),
     "GOLDEN_output_motion_p90":v1p["official_motion_p90_px"],
     "C2_output_motion_p90":c2p["official_motion_p90_px"],
     "C3_output_motion_p90":c3p["official_motion_p90_px"],
     "C3_yawfollow_output_motion_p90":yawp["official_motion_p90_px"],
     "GOLDEN_rotation_deg_p90":refd["V1"]["global_rotation_p90_deg"],
     "C2_rotation_deg_p90":refd["C2_EDGE"]["global_rotation_p90_deg"],
     "C3_rotation_deg_p90":refd["C3_GLOBAL"]["global_rotation_p90_deg"],
     "GOLDEN_center_dx_p90":v1p["center_dx_abs_p90"],
     "C3_center_dx_p90":c3p["center_dx_abs_p90"],
     "GOLDEN_edge_max":v1p["edge_antisymmetric_max_abs"],
     "C3_edge_max":c3p["edge_antisymmetric_max_abs"],
     "C3_yawfollow_edge_max":yawp["edge_antisymmetric_max_abs"],
     "C3_yawfollow_quality_gate_pass":bool(yawp["V1_anti_sway_and_edge_pass"]),
     "full_quality_gate_pass":bool(c3p["V1_anti_sway_and_edge_pass"]),
     "C3_input_future_or_golden":False,
     "DEVICE":"NOT_TESTED"
    }
def main():
    cases=[case_report(case) for case in CASES]
    table="".join(
      f'<tr><td><a href="{c["case"]}/index.html">{c["case"]}</a> {DESCR[c["case"]]}</td>'
      f'<td>{c["GOLDEN_output_motion_p90"]:.3f}</td>'
      f'<td>{c["C2_output_motion_p90"]:.3f}</td>'
      f'<td>{c["C3_output_motion_p90"]:.3f}</td>'
       f'<td>{c["C3_yawfollow_output_motion_p90"]:.3f}</td>'
      f'<td>{c["GOLDEN_rotation_deg_p90"]:.3f}</td>'
      f'<td>{c["C3_rotation_deg_p90"]:.3f}</td>'
      f'<td>{c["GOLDEN_edge_max"]:.2f}</td>'
      f'<td>{c["C3_edge_max"]:.2f}</td>'
       f'<td>{c["C3_yawfollow_edge_max"]:.2f}</td>'
      f'<td>{"PASS" if c["full_quality_gate_pass"] else "FAIL"}</td></tr>'
      for c in cases)
    home=(
     '<h1>Gemini Tripod V2C · C2-C3 Global Path 4DoF 实验</h1>'
     '<p class="subtitle">2026-10-24 · 四段真实2560×1440 RAW，400Hz因果Gyro、Native C1 LK、唯一C++ qVirtual/Visual/Zoom状态、20×33 RS19</p>'
     '<div class="strip"><b>中心优先，全幅旋转同时消除；不锁单个电脑屏幕，不依赖未来帧或V1 oracle。</b>'
     '下表数值是真正H264输出独立光流，不是320×180特征域模拟。'
     '四段严格对V1全图+中心+边缘综合Gate未通过之前继续SHADOW，不替换黄金版；'
     'C3候选Host处理开销不含终态GPU/Encoder，DEVICE未测。</div>'
     '<section><h2>四段真实像素与全局Rotation分解</h2>'
     '<table class="results"><thead><tr><th>录像</th><th>V1全图p90</th><th>C2全图p90</th><th>C3全图p90</th><th>YawFollow全图p90</th><th>V1 Roll p90°</th><th>C3 Roll p90°</th><th>V1 Edge Max</th><th>C3 Edge Max</th><th>YawFollow Edge Max</th><th>全Gate</th></tr></thead><tbody>'+
     table+'</tbody></table>'
     '<p>每片14张SVG分为输入、识别、控制、几何、输出、性能6层；'
     '每片V1、C2、C3 Global、C3 YawFollow四路1280×720真H264可同步播放。'
     '输出结果同时分离全局dx/dy/rotation/Scale及左右两侧非Global残余。'
     '完整可增Zoom的Yaw半周期预测已因增加边缘motion且Zoom达到1.14而否决；四路中YawFollow版本仅按因果Yaw变化提前调整qVirtual、以旧Zoom进行RS19预算对照，但同样0/4全画幅质量Gate，产品仍然OFF。</p></section>'
     '<section><h2>四片视频与逐帧证据</h2>'+
     "".join(f'<article><h3>{c["case"]} {DESCR[c["case"]]}</h3>'
             f'<p><a href="{c["case"]}/index.html">打开 V1 / C2 / C3 Global / YawFollow 四路对比，14张SVG和详细门禁</a></p></article>'
             for c in cases)+'</section>'
     '<section><h2>阶段状态</h2><p>当前代码验证通过native C++因果50/60帧前缀与无未来样本、RS19真实20×33所有源边界安全；'
     'C2-C3主方案仍仅为PC SHADOW。严禁在未通过四段画质/Device FPS之前开启产品默认。</p>'
     '<p><a href="c3_global_report_all.json">整体验收原始数据</a></p></section>'
    )
    path=OUT/"index.html"
    path.write_text(String_HTML_START+home+String_HTML_END,encoding="utf-8")
    summ={
      "stage":"V2C C2-C3 native causal Global4DoF Path Observer",
      "real_4_videos_291_source_frames_each":True,
      "C3_actually_rendered_H264":True,
      "four_video_full_pixel_quality_gate":all(c["full_quality_gate_pass"] for c in cases),
      "yawfollow_four_quality_pass_count":sum(c["C3_yawfollow_quality_gate_pass"] for c in cases),
      "four_video_full_pixel_quality_pass_count":sum(c["full_quality_gate_pass"] for c in cases),
      "C3_no_future_frames_input":True,
      "V1_golden_offline_only_quality_reference":True,
      "Android_30fps":"NOT_TESTED",
      "html":str(path),"cases":cases
    }
    (OUT/"c3_global_report_all.json").write_text(json.dumps(summ,ensure_ascii=False,indent=2),encoding="utf-8")
    print("GEMINI_C3_TRUE4_RAW_OUTPUT_SIX_STAGES_56_REAL_SVG",json.dumps(
         {"home":str(path),"cases":len(cases),"plots":len(cases)*14,
          "quality_gate_passed":summ["four_video_full_pixel_quality_pass_count"]},
         ensure_ascii=False),flush=True)
if __name__=="__main__":main()
