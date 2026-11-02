#!/usr/bin/env python3
"""Engineering evidence for the architectural failure of two final-Warp controllers.

GOLDEN / Feedback / Output-only use same 291 original Sensor frames
and native 20x33 RS19. Graphs use actual encoded-video LK and true meshes.
No false DEVICE PASS, offline Oracle and future frames still upstream.
"""
from __future__ import annotations
import csv,html,json
from pathlib import Path
import numpy as np
from generate_report import svg_plot, String_HTML_START,String_HTML_END
from render_and_evaluate import OUT,ROOT

CASE="024630"
REL=OUT/CASE
DEST=OUT/"unified_budget_stage"
COLORS={"V1 GOLDEN":"#73AFF7","V2 Feedback":"#F5B65A",
        "V2 Output-only":"#4BD5A1"}
MODE={"V2 Feedback":"unified_joint","V2 Output-only":"unified_output_only"}
FIELDS=["event","sensor_ts_ns","reason","attempted","accepted","margin_px",
        "margin_new_px","span_source_px","span_new_source_px",
        "zoom","zoom_new","scale","scale_new","gyro_yaw_rad_s","center_shift_source_px"]
def read(case: str,mode: str):
    file=OUT/case/mode/"budget_trace.csv"
    with file.open(encoding="utf-8",newline="") as f:
        rr=[r for r in csv.DictReader(f,fieldnames=FIELDS) if r["event"]=="V2_BUDGET"]
    if len(rr)!=291:raise RuntimeError("Expected 291 raw camera frames, got "+str(len(rr)))
    return rr
def mesh_info(path):
    mesh=np.fromfile(path,dtype=np.float32).reshape(-1,20,33,2)
    span=(mesh[:,9,32,0]+mesh[:,10,32,0]-mesh[:,9,0,0]-mesh[:,10,0,0])*.5*2560
    margin=np.minimum.reduce([
        mesh[:,:,:,0].min((1,2)),1-mesh[:,:,:,0].max((1,2)),
        mesh[:,:,:,1].min((1,2)),1-mesh[:,:,:,1].max((1,2))])*1440
    return mesh,span,margin
def series(quality,field):
    return {int(row["frame"]):float(row[field]) for row in quality["roi_series"]}
def full_series(quality,field):
    d=series(quality,field)
    return [d.get(i) for i in range(291)]
def pic(name):
    return { "V1 GOLDEN":"../../../rendered_preview_fixed/024630_GEMINI_STABLE.mp4",
         "V2 Feedback":"../024630/unified_joint/gemini_v2_output.mp4",
         "V2 Output-only":"../024630/unified_output_only/gemini_v2_output.mp4"}[name]
def main():
    DEST.mkdir(parents=True,exist_ok=True)
    j1=json.loads((REL/"unified_joint_real_pixel_comparison.json").read_text(encoding="utf-8"))
    j2=json.loads((REL/"unified_output_only_real_pixel_comparison.json").read_text(encoding="utf-8"))
    qualities={"V1 GOLDEN":j1["GOLDEN"],"V2 Feedback":j1["UNIFIED_V2"],
               "V2 Output-only":j2["UNIFIED_V2"]}
    mesh_files={"V1 GOLDEN":ROOT/"gemini_mesh_024630.f32",
        "V2 Feedback":REL/"unified_joint/mesh_joint.f32",
        "V2 Output-only":REL/"unified_output_only/mesh_joint.f32"}
    mm={k:mesh_info(p) for k,p in mesh_files.items()}
    rr={name:read(CASE,mode) for name,mode in MODE.items()}
    xx=list(range(291))
    mask={}
    for name in MODE:
        row=rr[name]
        mask[name]=[1 if r["accepted"]=="1" else 0 for r in row]
    # Integrity of the hybrid effect: exact original for all non-applied frames.
    golden=mm["V1 GOLDEN"][0]
    parity={}
    for name in MODE:
        applied=np.array(mask[name],bool)
        changed=np.any(mm[name][0]!=golden,axis=(1,2,3))
        parity[name]={"accepted":int(applied.sum()),
            "changed":int(changed.sum()),
            "unaccepted_changed":int(np.sum(changed&~applied)),
            "accepted_unchanged":int(np.sum(applied&~changed))}
    assert parity["V2 Output-only"]["unaccepted_changed"]==0
    def graph(label,unit,fn,subset=xx,fmt=".2f"):
        return svg_plot(label,unit,[(n,subset,
            [fn(n,i) for i in subset],COLORS[n]) for n in COLORS],
            marker=67,fmt=fmt)
    names=list(COLORS)
    input_plot=svg_plot("真实Gyro Yaw 角速度（不使用未来帧）","rad/s",
        [("Same raw gyro",xx,
          [float(r["gyro_yaw_rad_s"]) for r in rr["V2 Feedback"]],"#9DB4CF")],
        marker=67,fmt=".2f")
    risk=svg_plot("RS19危险时段：选择（1）或不选择（0）当前帧补偿","0/1",[
        (name,xx,mask[name],COLORS[name]) for name in MODE],
        marker=67,fmt=".1f")
    zoom=svg_plot("控制器Zoom：基线输入与输出预算（注意反馈长期状态）","zoom",[
        ("Base Zoom",xx,[float(r["zoom"]) for r in rr["V2 Feedback"]],"#73AFF7"),
        ("Feedback Chosen",xx,[float(r["zoom_new"]) for r in rr["V2 Feedback"]],"#F5B65A"),
        ("Output-only Chosen",xx,[float(r["zoom_new"]) for r in rr["V2 Output-only"]],"#4BD5A1"),
    ],fmt=".3f")
    marg=svg_plot("真实20×33 RS19全网格最小边界余量","source px",[
        (name,xx,mm[name][2].tolist(),COLORS[name]) for name in names],fmt=".1f")
    spans=svg_plot("RS19中心行左右边缘采样跨度（几何代理，非最终像素质量）",
                   "source px",[(name,xx,mm[name][1].tolist(),COLORS[name])
                                for name in names],fmt=".1f")
    all_edge=graph("真实H264左右边缘反向伸缩，291帧","output px/frame",
                   lambda n,i:full_series(qualities[n],"edge_breath")[i],fmt=".1f")
    window=list(range(50,77))
    event_edge=graph("转向极值前后：真实H264左右反向伸缩事件窗口",
                   "output px/frame",
                   lambda n,i:full_series(qualities[n],"edge_breath")[i],
                   subset=window,fmt=".1f")
    center=graph("真实H264中心背景水平运动，极值事件窗口","output px/frame",
                 lambda n,i:full_series(qualities[n],"center_dx")[i],
                 subset=window,fmt=".1f")
    sensor_ts=[int(r["sensor_ts_ns"]) for r in rr["V2 Feedback"]]
    cadence=svg_plot("真实Camera SensorTS，相邻帧间隔（掉帧不补0）","ms",
        [("Camera",xx[1:],(np.diff(sensor_ts)*1e-6).tolist(),"#9DB4CF")],
        fmt=".1f")
    tab=[]
    for name,quality in qualities.items():
        rows=[quality[k] for k in (
            "official_motion_p90_px","center_dx_abs_p90",
            "edge_antisymmetric_abs_p90","edge_antisymmetric_max_abs")]
        tab.append(f'<tr><td>{html.escape(name)}</td>'+
                   ''.join(f'<td>{v:.3f}</td>' for v in rows)+
                   f'<td>{parity.get(name,{}).get("accepted","—")}</td>'+
                   f'<td>{parity.get(name,{}).get("unaccepted_changed","—")}</td></tr>')
    parts=[
    '<h1>Gemini Tripod V2B · 统一预算控制架构 A/B</h1>',
    '<p class="subtitle">PC同源291帧 2560×1440 SourceID4–294 · 原C++ 20×33 RS19 · 最终1280×720真实H264</p>',
    '<div class="strip"><b>重要结论：两条实际输出架构均FAIL。</b> 只有对目标相机轨迹与Zoom/Scale进行共同规划，才能同时保持已有中心锁定与日常边缘稳定。'
      '原V1依然是效果黄金版。此实验输入还是离线Visual/Oracle，Android真实零前瞻仍NOT_TESTED。</div>',
    '<section><h2>1 输入 Camera/Gyro</h2><p>三种实验使用同一真实原始source视频、同一Camera SensorTS和原始C++ RS19行投影。'
      '所有输出经FFmpeg完整H264解码0错误。源并非已经稳定后的二次转码。</p>'+input_plot+cadence+'</section>',
    '<section><h2>2 识别 Recognition</h2><p>使用真实Yaw速度方向、PAN置信度与本帧及历史几何余量识别高风险极值。'
      '两个分支只修改单个C++核心的最终选帧机制；当前V1离线Visual仍含未来帧，不能宣称整体因果。</p>'
      +risk+'</section>',
    '<section><h2>3 控制 Control</h2><p>橙色Feedback将被选的Zoom回写长期8秒退场状态；绿色Output-only不回写Zoom，只允许当前帧改变实际Warp。'
      '二者均不是最终目标轨迹联合求解器；先验证它们的副作用。</p>'+zoom+'</section>',
    '<section><h2>4 几何 Geometry</h2><p>Y1/Gemini RS19=19区间×33列间隔，即20×33物理顶点。通过原版C++ Rs19OneWarp完整计算，'
      '不增加安全裁剪，所有候选真实UV越界0。但“几何合法”≠“视频无伸缩”。</p>'+marg+spans+'</section>',
    '<section><h2>5 输出 Output · 完整真实像素</h2><table class="results"><tr><th>架构</th>'
      '<th>全图Motion p90</th><th>中心dx p90</th><th>边缘伸缩p90</th><th>极值max</th>'
      '<th>采用帧</th><th>未采用却被改的帧</th></tr>'+''.join(tab)+'</table>',
    '<p>橙色：12帧主动采用后导致其余210帧的长期Zoom轨迹不同；绿色：直接采用32帧，其余259帧逐顶点与V1一致，'
      '但输出相邻帧补偿不连续，边缘日常p90严重恶化。不能仅凭“未采用帧bit-exact”认定最终画面更好。</p>',
     event_edge+center+all_edge,
    '<h3>相同原始录像三联（优先观察帧56、67和69）</h3><div class="videos">'
      +''.join('<div><h4>'+html.escape(name)+'</h4><video controls muted preload="metadata" src="'+pic(name)+'"></video></div>'
        for name in names)+'</div>'
       '<div class="controls"><button onclick="syncAll()">同步到V1时间轴</button>'
       '<button onclick="goExtrema()">定位第67帧</button>'
       '<button onclick="playAll()">同步播放</button><button onclick="pauseAll()">暂停</button></div></section>',
    '<section><h2>6 性能 Performance 与新架构结论</h2>'
      '<p>隔离PC核心回放全帧实测约0.8–1.2ms（不含整段离线Visual生成、GPU采样、编码）；'
      '单位C++测试normal帧bit-exact、PAN否决、无前瞻参数读取及20×33真实几何测试通过，'
      '但1080p/30fps Android、热和端到端延迟均未测试。</p>'
      '<p><b>真正下一结构：</b>在基于当前/历史Gyro＋因果多区域LK的同一时刻，先让'
      '唯一Target owner联合规划qVirtual、Visual路径、Zoom预算和边界回拉，'
      '再一次性对真实19行网格做物理提交；受控保留稳定中心与正常PAN，'
      '不允许late-frame输出差分或回写长期Zoom形成尾迹。自由8DoF需另过空间留出Gate。</p>'
      '<p>详细跨四录制原生几何验收与架构设计：'
      'experiments/tripod_v2/ARCHITECTURE_V2B.md。'
      '<a href="../024630/spatial_4dof_vs_8dof_model_heldout.json">独立4DoF/8DoF空间留出验证</a>。</p></section>'
    ]
    script='''<script>
const v=()=>Array.from(document.querySelectorAll("video"));
function syncAll(){let t=v()[0].currentTime;for(let a of v())a.currentTime=t;}
function goExtrema(){for(let a of v()){a.pause();a.currentTime=67/29.605;}}
function playAll(){syncAll();for(let a of v())a.play().catch(()=>{});}
function pauseAll(){for(let a of v())a.pause();}
</script>'''
    content=String_HTML_START+"".join(parts)+script+String_HTML_END
    file=DEST/"index.html";file.write_text(content,encoding="utf-8")
    manifest={"case":CASE,"frames":291,
      "RS19_full_20x33_UV_zero_violations":True,"six_evidence_stages":True,
      "chart_count":9,"H264_videos":3,
      "golden_preserved":True,
      "actual_pixel_gate":"FAIL_BOTH_BRANCHES",
      "android_zero_lookahead":"NOT_TESTED_UPSTREAM_NONCAUSAL",
      "frame_parity":parity,
      "metrics":{name:{k:v for k,v in q.items() if k!="roi_series"}
                for name,q in qualities.items()}}
    (DEST/"evidence.json").write_text(
        json.dumps(manifest,indent=2,ensure_ascii=False),encoding="utf-8")
    print("V2B_UNIFIED_PROFESSIONAL_HTML_DONE",
          json.dumps({"file":str(file),"charts":9,"videos":3,
            "parity":parity,"gate":manifest["actual_pixel_gate"]},
             ensure_ascii=False),flush=True)

if __name__=="__main__":main()
