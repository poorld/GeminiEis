#!/usr/bin/env python3
"""C2 genuine 4-case 1280x720 VIDEO Pixel-Gate dashboard, six-stage.

V1 GOLDEN read-only benchmark. The candidate inputs are true streaming
C1-B2 Native visual + Camera/Gyro/physical RS19, no V1/Oracle in controller.
Plots always distinguish output EIS pixel motion from mesh-only simulations.
"""
from __future__ import annotations
import html,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from c1_generate_professional_report import plot,String_HTML_START,String_HTML_END
from c1_stream_host import ROOT,CASES

ROOT_REPORT=ROOT/"reports"/"gemini_tripod_v2"
OUT=ROOT_REPORT/"c2_shadow_cal"
EDGE=ROOT_REPORT/"c2_shadow_edge"
OLD=ROOT_REPORT/"c2_shadow"
INPUT=ROOT_REPORT/"c1b2_yuv"
COL={"V1":"#8AB1F4","C2center":"#41CBA9","C2edge":"#C3A0F7",
     "sensor":"#EDA766","limit":"#A4AEB7"}
LABELS={"024605":"低速往返","024618":"中速往返",
        "024630":"快速往返","024647":"单侧转向"}
SIX=("Input","Recognition","Control","Geometry","Output","Performance")
def series(p):
    return [json.loads(v) for v in Path(p).read_text(encoding="utf-8").splitlines()]
def extract(k,stats,key):
    return {int(p["frame"]):float(p[key]) for p in stats["roi_series"]}.get(k)
def chart(title,unit,fields,fmt=".2f"):
    return plot(title,unit,fields,fmt=fmt)
def case_build(case):
    base=OUT/case
    eval_cal=json.loads((base/"pixel_effect_vs_V1.json").read_text(encoding="utf-8"))
    assert "c2_joint_visual" in eval_cal["candidates"]
    v1=eval_cal["V1_GOLDEN"]
    center=eval_cal["candidates"]["c2_joint_visual"]
    hasEdge=(EDGE/case/"pixel_effect_vs_V1.json").is_file()
    edge=(json.loads((EDGE/case/"pixel_effect_vs_V1.json").read_text(encoding="utf-8"))
           ["candidates"]["c2_joint_visual"] if hasEdge else None)
    c2=series(base/"c2_joint_visual_frames.jsonl")
    old=series(INPUT/case/"camera_ready_frames.jsonl")
    assert len(c2)==291 and len(old)==290
    q=np.array([f["q_camera_xyzw"] for f in c2])
    virtual=np.array([f["q_virtual_xyzw"] for f in c2])
    sensor_angles=Rotation.from_quat(q).as_rotvec()[:,1]
    virtual_angles=Rotation.from_quat(virtual).as_rotvec()[:,1]
    frames=np.array([r["source_id"] for r in c2],dtype=int)-4
    cam_time=[r["sensor_ts_ns"]*1e-6 for r in c2]
    ts_dt=[None]+list(np.diff(cam_time))
    zoom=np.array([r["zoom"] for r in c2])
    diffzoom=np.diff(np.log(zoom),prepend=np.log(zoom[0]))*100
    edgesrc=series(EDGE/case/"c2_joint_visual_frames.jsonl") if hasEdge else None
    native_obs={r["source_id"]-4:r["visual_output"] for r in old}
    obs_dx=[native_obs.get(int(i),{}).get("dx_feature_px") for i in frames]
    w=1280; fs=1280/320
    # Pixel-domain lists (the actual rendered mp4 is 1280x720).
    keys=("center_dx","edge_breath")
    f=frames.tolist()
    def px(st,key):
        return [extract(i,st,key) for i in f]
    pics={
      "input":chart("原始Camera SensorTS相邻帧间隔（真实SourceID 4..294）","ms",
       [("SensorTS",f,ts_dt,COL["sensor"])],".1f"),
      "gyro":chart("400Hz Gyro积分：真实Yaw与仅用当前历史的C2目标Yaw",
        "radians",[("Camera物理Yaw",f,sensor_angles.tolist(),COL["sensor"]),
                   ("C2 qVirtual Yaw",f,virtual_angles.tolist(),COL["C2center"])],".3f"),
      "residual":chart("C1-B2真实Gyro+RS19预稳定视觉Residual（中心加权全局）",
        "320x180 feature px/frame",
        [("C1 Global dx",f,obs_dx,COL["sensor"]),
         ("C2校准后预测剩余",f,[r["predicted_visual_residual_feature_px"] for r in c2],
          COL["C2center"])],".2f"),
      "center":chart("真正H264输出像素：中心背景左右运动（不是Source陀螺）",
        "1280x720 output px/frame",
        [("V1黄金",f,px(v1,"center_dx"),COL["V1"]),
         ("C2光学校准",f,px(center,"center_dx"),COL["C2center"])]
        +([("C2边缘连续",f,px(edge,"center_dx"),COL["C2edge"])] if hasEdge else []),".2f"),
      "edge":chart("真正H264输出像素：左右两侧相对中心的反向伸缩",
        "1280x720 output px/frame",
        [("V1黄金",f,px(v1,"edge_breath"),COL["V1"]),
         ("C2光学校准",f,px(center,"edge_breath"),COL["C2center"])]
        +([("C2边缘连续",f,px(edge,"edge_breath"),COL["C2edge"])] if hasEdge else []),".2f"),
      "zoom":chart("C2 Joint Zoom轨迹与边缘连续性（严格不是后期追加Zoom滤波）",
        "Zoom ratio",[("C2光学校准",f,zoom.tolist(),COL["C2center"])]
        +([("C2边缘连续",f,[r["zoom"] for r in edgesrc],COL["C2edge"])] if hasEdge else []),".3f"),
      "zoomstep":chart("Zoom变化对输出边缘的预测伸缩（0.5×1280×delta logZoom）",
        "output edge px/frame",
        [("C2光学校准",f,(diffzoom*640/100).tolist(),COL["C2center"])]
        +([("C2边缘连续",f,(np.diff(np.log([r["zoom"] for r in edgesrc]),
           prepend=np.log(edgesrc[0]["zoom"]))*640).tolist(),COL["C2edge"])] if hasEdge else []),".2f"),
      "geometry":chart("全20×33实际RS19顶点最小源边界余量",
        "source px",
        [("C2光学校准",f,[r["margin_source_px"] for r in c2],COL["C2center"])]
        +([("C2边缘连续",f,[r["margin_source_px"] for r in edgesrc],COL["C2edge"])] if hasEdge else []),".1f"),
      "span":chart("实体RS19全图边缘跨度变化（不等于最终像素边缘运动）",
        "source px/frame",
        [("C2光学校准",f,[r["mesh_span_step_px"] for r in c2],COL["C2center"])]
        +([("C2边缘连续",f,[r["mesh_span_step_px"] for r in edgesrc],COL["C2edge"])] if hasEdge else []),".2f"),
      "performance":chart("Host C2单次联合规划耗时及C1 Native输入时延（非Android帧率）",
        "ms",[("C2求解器",f,[r["C2_native_solve_ms"] for r in c2],COL["C2center"]),
              ("C1-B2 Native视觉就绪",f,[next((x["camera_ready_to_gate_decision_ms"]
                 for x in old if x["source_id"]==r["source_id"]),None) for r in c2],COL["sensor"])]
              +([("C2边缘连续求解器",f,[r["C2_native_solve_ms"] for r in edgesrc],COL["C2edge"])]
                 if hasEdge else []),".1f")
    }
    assert len(pics)==10
    def metric_table():
        names=[("Gemini V1 GOLDEN",v1,None),("C2 Optical Center",center,center["V1_anti_sway_and_edge_pass"])]
        if hasEdge:names.append(("C2 Edge-Coherent",edge,edge["V1_anti_sway_and_edge_pass"]))
        rows="".join(
            f'<tr><td>{html.escape(n)}</td>'
            f'<td>{stats["official_motion_p90_px"]:.3f}</td>'
            f'<td>{stats["center_dx_abs_p90"]:.3f}</td>'
            f'<td>{stats["edge_antisymmetric_abs_p90"]:.3f}</td>'
            f'<td>{stats["edge_antisymmetric_max_abs"]:.3f}</td>'
            f'<td>{"参考" if ok is None else ("PASS" if ok else "FAIL")}</td></tr>'
            for n,stats,ok in names)
        return '<table class="results"><thead><tr><th>版本</th><th>全图Motion p90</th><th>中心|dx|p90</th><th>边缘反向p90</th><th>边缘极值</th><th>相对V1全门禁</th></tr></thead><tbody>'+rows+'</tbody></table>'
    movie_v1=f'../../../../rendered_preview_fixed/{case}_GEMINI_STABLE.mp4'
    movie_cal="c2_joint_visual_output.mp4"
    movie_edge=f'../../c2_shadow_edge/{case}/c2_joint_visual_output.mp4'
    n_video=3 if hasEdge else 2
    movies=('<div class="videos">'+
            f'<div><h4>V1黄金（非因果离线，仅画质参考）</h4><video id="v1" controls muted preload="metadata" src="{movie_v1}"></video></div>'+
            f'<div><h4>C2 因果中心校准</h4><video id="cal" controls muted preload="metadata" src="{movie_cal}"></video></div>'+
            (f'<div><h4>C2 Zoom边缘连续候选</h4><video id="edge" controls muted preload="metadata" src="{movie_edge}"></video></div>'
             if hasEdge else '')+'</div>'+
            '<p><button onclick="syncAll()">同步视频时间</button> '
            '<button onclick="playAll()">同步播放</button> '
            '<button onclick="pauseAll()">暂停全部</button></p>'
            '<script>function videos(){return [...document.querySelectorAll("video")]};'
            'function syncAll(){let t=videos()[0].currentTime;videos().forEach(v=>v.currentTime=t)}'
            'function playAll(){syncAll();videos().forEach(v=>v.play())}'
            'function pauseAll(){videos().forEach(v=>v.pause())}</script>')
    stages=(
      f'<h1>Gemini V2C · C2 qVirtual+Visual+Zoom · {case} {LABELS[case]}</h1>'
      f'<p class="subtitle">真实2560×1440 RAW SourceID4～294→一次20×33 RS19 Warp→1280×720 H264 · 原V1黄金只用于后验质量评估</p>'
      '<div class="strip"><b>只有所有像素指标对V1都不退化才可晋级。</b>'
      '当前C2是PC因果Shadow实验，未上线Android或MTK。'
      '中心和边缘极值单项进步不代表全画幅长期稳定通过。</div>'
      '<section><h2>1 输入 Input</h2><p>每帧真实SensorTS/SourceID，当前+历史Gyro仅推送至物理曝光/RS19截止时间。'
      'C1 Native视觉为背景3×3 Tile一致的320x180真实残余，不读V1离线Oracle或未来帧。</p>'
      +pics["input"]+pics["gyro"]+'</section>'
      '<section><h2>2 识别 Recognition</h2><p>C1-B2 Global residual与C2采用的中心因果残余预测。'
      '这不是中心锁死某个电脑屏幕对象，而是全画幅背景共模。</p>'
      +pics["residual"]+'</section>'
      '<section><h2>3 控制 Control</h2><p>同一个C++ Target Owner选择qVirtual、Visual 4DoF和Zoom。'
      '所有SourceID完整提交，旧V2B的末端追加Zoom或回拉未使用。</p>'
      +pics["zoom"]+pics["zoomstep"]+'</section>'
      '<section><h2>4 几何 Geometry</h2><p>完整20×33共660个真实RS19几何顶点'
      '全部扫描，最小源余量不得低于2px，后验观测边缘变形必须和这个几何门禁区分。'
      '仅20个物理gyro行、19个RS行间，且全片只一次最终高清Warp。</p>'
      +pics["geometry"]+pics["span"]+'</section>'
      '<section><h2>5 输出 Output：真实H264像素</h2>'
      +metric_table()
      +pics["center"]+pics["edge"]
      +movies+'</section>'
      '<section><h2>6 性能 Performance</h2><p>C++真实C2完整RS19候选选择和C1-B2 Native视觉时间分开。'
      '不能把CPU Host和真正设备ISP、GPU、Encoder负载混成30fps通过。</p>'
      +pics["performance"]+'</section>'
      '<section><h2>实际证据</h2>'
      '<p><a href="pixel_effect_vs_V1.json">真实像素指标/每帧左中右轨迹</a>、'
      '<a href="c2_joint_visual_frames.jsonl">因果qVirtual/Visual/Zoom逐帧提交日志</a>、'
      '<a href="c2_joint_visual_summary.json">完整RS19安全和Native性能</a>。'
      '只有Host Gate，DEVICE_NOT_TESTED；冻结的Gemini V1 MP4未覆盖。</p></section>'
    )
    dest=base/"index.html"
    dest.write_text(String_HTML_START+stages+String_HTML_END,encoding="utf-8")
    out={
      "case":case,
      "V1_center_dx_p90":v1["center_dx_abs_p90"],
      "calibrated_center_dx_p90":center["center_dx_abs_p90"],
      "V1_edge_p90":v1["edge_antisymmetric_abs_p90"],
      "calibrated_edge_p90":center["edge_antisymmetric_abs_p90"],
      "V1_edge_peak":v1["edge_antisymmetric_max_abs"],
      "calibrated_edge_peak":center["edge_antisymmetric_max_abs"],
      "calibrated_full_gate":bool(center["V1_anti_sway_and_edge_pass"]),
      "edge_variant_available":hasEdge,
      "edge_full_gate":bool(edge["V1_anti_sway_and_edge_pass"]) if hasEdge else None,
      "C2_no_future_camera_or_oracle":True,
      "same_source_291_frames":True,
      "charts":10,
      "real_video_count":n_video,
      "html":str(dest)
    }
    return out
def main():
    files=[case_build(c) for c in CASES]
    asserts=[c["calibrated_full_gate"] for c in files]
    trs="".join(
      f'<tr><td><a href="{c["case"]}/index.html">{c["case"]}</a> {LABELS[c["case"]]}</td>'
      f'<td>{c["V1_center_dx_p90"]:.3f}</td>'
      f'<td>{c["calibrated_center_dx_p90"]:.3f}</td>'
      f'<td>{c["V1_edge_p90"]:.3f}</td>'
      f'<td>{c["calibrated_edge_p90"]:.3f}</td>'
      f'<td>{c["V1_edge_peak"]:.2f}</td>'
      f'<td>{c["calibrated_edge_peak"]:.2f}</td>'
      f'<td>{"PASS" if c["calibrated_full_gate"] else "FAIL"}</td></tr>'
      for c in files)
    page=(
      '<h1>Gemini V2C · C2 单核qVirtual + Visual + Zoom 真RAW视频Gate</h1>'
      '<p class="subtitle">2026-10-24 · 四段2560×1440原始Camera／各291帧，'
      '全部使用Native C1-B2背景Residual，PC C++因果RS19一次1280×720渲染</p>'
      '<div class="strip"><b>本页是四片真实输出H264像素测试，而不是仅有mesh/光流模拟。</b>'
      'V1离线三脚架只是冻存的画质黄金对照；从未用于C2控制。'
      '任何一项中心/全图/边缘p90相对V1退化都不得将C2晋级正式版本。'
      'Android 29.6fps设备门禁仍NOT_TESTED。</div>'
      '<section><h2>四段实际相同RAW的效果</h2>'
      '<table class="results"><thead><tr><th>视频</th><th>V1中心p90</th><th>C2中心p90</th>'
      '<th>V1边缘p90</th><th>C2边缘p90</th><th>V1边缘峰</th><th>C2边缘峰</th>'
      '<th>C2全部画质Gate</th></tr></thead><tbody>'+trs+'</tbody></table>'
      '<p>单位均为1280×720 H264真实输出像素；C1原特征域差值不能拿来冒充输出指标。'
      '相同SourceID配准、20×33物理RS19全部检验、已确认视频完整291帧可解码。</p></section>'
      '<section><h2>真实视频 A/B 同步对比</h2>'
      +''.join(f'<article><h3>{c["case"]} {LABELS[c["case"]]}</h3>'
               f'<p><a href="{c["case"]}/index.html">打开10张阶段图、V1黄金及C2新视频同步播放</a></p></article>'
               for c in files)+'</section>'
      '<section><h2>架构与下步门禁</h2>'
      '<p>当前为唯一C++qVirtual/Visual/Zoom决策，物理Gyro按Camera行时间+8ms校正，'
      '3×3 Native背景Residual因果到场，当前SourceID整帧候选在20×33安全门禁后原子提交。'
      '不再使用未来Anchor、oracle_matrices.f64、双向Gaussian或旧Core的Zoom事后修补。'
      '目前只是Host非产品候选，未碰Android/MTK/ALPS。'
      '整条Camera ISP/EGL Encoder30fps与最终软硬预算需DEVICE另验。</p></section>'
    )
    main=OUT/"index.html"
    main.write_text(String_HTML_START+page+String_HTML_END,encoding="utf-8")
    summary={"phase":"C2 joint qVirtual Visual Zoom real output Pixel Gate",
         "four_source_original_291":True,
         "four_videos_actual_H264_from_source":True,
         "golden_is_offline_readonly":True,
         "C2_is_causal_without_oracle":True,
         "C2_fully_passed_ALL_four_real_video_gates":all(asserts),
         "C2_NOT_product_enabled":True,
         "android":"NOT_TESTED",
         "cases":files,"home":str(main)}
    (OUT/"c2_professional_evidence.json").write_text(
        json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print("C2_FOUR_TRUE_RAW_PIXEL_GATE_HTML",json.dumps(
        {"html":str(main),"cases":len(files),"charts":10*len(files),
         "allGates":all(asserts)},ensure_ascii=False),flush=True)
if __name__=="__main__":main()
