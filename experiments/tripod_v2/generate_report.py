#!/usr/bin/env python3
"""Self-contained offline HTML evidence report for Gemini Tripod V2.
Six stages: Input -> Recognition -> Control -> Geometry -> Output -> Performance.
Official p90 and pixel motion from already decoded identical H264 MP4s.
No external JS/network/CDN. NOT AN ANDROID REALTIME REPORT.
"""
from __future__ import annotations
import html,json,math,re
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/"reports"/"gemini_tripod_v2"
CASE="024630"
DEST=OUT/CASE
GOLDEN="#73AFF7";C108="#4BD5A1";C110="#F5B65A";WARN="#ED849D"
SHORTLIST=["GOLDEN","floor108_slew0p0015","floor110_slew0p0015"]
CC={"GOLDEN":GOLDEN,SHORTLIST[1]:C108,SHORTLIST[2]:C110}
BUNDLE=Path("D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849")/CASE
def svg_plot(title,unit,axes,marker=67,fmt=".2f",w=1040,h=250):
    """axes array: [(name, frame array, values array, color)]."""
    xs=[float(x) for name,ax,vals,color in axes for x,v in zip(ax,vals)
        if v is not None and math.isfinite(float(v))]
    ys=[float(v) for name,ax,vals,color in axes for v in vals
        if v is not None and math.isfinite(float(v))]
    if not xs or not ys:return "<p>没有可用数据</p>"
    xmin,xmax=min(xs),max(xs)
    ymin,ymax=min(ys),max(ys)
    if abs(ymax-ymin)<1e-9:ymax+=1;ymin-=1
    pad=(ymax-ymin)*.10
    ymin-=pad;ymax+=pad
    L,R,T,B=72,22,24,43
    X=lambda v:L+(v-xmin)/(xmax-xmin if xmax>xmin else 1)*(w-L-R)
    Y=lambda v:T+(ymax-v)/(ymax-ymin)*(h-T-B)
    s=[f'<div class="chart"><h3>{html.escape(title)}</h3><div class="svgscroll"><svg width="100%" viewBox="0 0 {w} {h}" role="img" aria-label="{html.escape(title)}" xmlns="http://www.w3.org/2000/svg">']
    for i in range(5):
        yv=ymin+(ymax-ymin)*i/4
        py=Y(yv)
        s.append(f'<line x1="{L}" y1="{py:.2f}" x2="{w-R}" y2="{py:.2f}" stroke="#344356" stroke-width=".8"/>')
        s.append(f'<text x="{L-9}" y="{py+4:.2f}" text-anchor="end" fill="#AFC0D4" font-size="12">{format(yv,fmt)}</text>')
    for i in range(6):
        xv=xmin+(xmax-xmin)*i/5
        px=X(xv)
        s.append(f'<text x="{px:.2f}" y="{h-18}" text-anchor="middle" fill="#9BABBE" font-size="11">{xv:.0f}</text>')
    if xmin<=marker<=xmax:
        sx=X(marker)
        s.append(f'<line x1="{sx:.2f}" y1="{T}" x2="{sx:.2f}" y2="{h-B}" stroke="{WARN}" stroke-dasharray="4,5" stroke-width="1.8"/>')
        s.append(f'<text x="{sx+6:.2f}" y="15" fill="{WARN}" font-size="12">关键帧 {marker}</text>')
    for name,xx,vv,color in axes:
        pts=[];segments=[]
        for x,y in zip(xx,vv):
            if y is None or not math.isfinite(float(y)):
                if pts:segments.append(pts);pts=[]
                continue
            pts.append(f"{X(float(x)):.2f},{Y(float(y)):.2f}")
        if pts:segments.append(pts)
        for points in segments:
            if len(points)>1:
                s.append(f'<polyline fill="none" stroke="{color}" stroke-width="2" stroke-linejoin="round" stroke-linecap="round" points="{" ".join(points)}"/>')
    s.append(f'<text x="{w//2}" y="{h-2}" text-anchor="middle" fill="#9BABBE" font-size="12">输出视频帧号（第0帧为原SOURCE source_index=4）</text>')
    s.append('</svg></div><div class="legend">')
    for name,xx,vv,color in axes:
        s.append(f'<span><em style="background:{color}"></em>{html.escape(name)}</span>')
    s.append(f'<span class="unit">{html.escape(unit)}</span></div></div>')
    return "".join(s)
def scale(arr):
    return np.hypot(arr[:,0,0],arr[:,1,0]*(1440/2560))
def mesh_stats(mesh):
    v=mesh
    width=(v[:,9,32,0]+v[:,10,32,0]-v[:,9,0,0]-v[:,10,0,0])*.5*2560
    margin=np.minimum.reduce([v[:,:,:,0].min((1,2)),
                             1-v[:,:,:,0].max((1,2)),
                             v[:,:,:,1].min((1,2)),
                             1-v[:,:,:,1].max((1,2))])*1440
    return width,margin
def cxx_core_us(meta):
    m=re.search(r"Average Frame Latency\s*:\s*([0-9.]+)\s*us",
                meta.get("CXX_RS19_replay_output",""))
    return float(m.group(1)) if m else None
def tabular(res):
    b=res["GOLDEN"]
    rows=[("GOLDEN",b,None)]+[(k,v,((v["edge_antisymmetric_max_abs"]/b["edge_antisymmetric_max_abs"])-1)*100)
            for k,v in res["variants"].items()]
    s=['<table class="results"><thead><tr><th>方案</th><th>全图Motion p90</th><th>中心|dx|p90</th><th>边缘反向伸缩p90</th><th>极值max</th><th>极值下降</th><th>通过全部画质Gate</th></tr></thead><tbody>']
    for k,v,pct in rows:
        status="参考" if k=="GOLDEN" else "PASS" if v["accept_as_golden"] else "FAIL"
        s.append(f'<tr><td>{html.escape(k)}</td><td>{v["official_motion_p90_px"]:.3f}</td>'
                 f'<td>{v["center_dx_abs_p90"]:.3f}</td><td>{v["edge_antisymmetric_abs_p90"]:.3f}</td>'
                 f'<td>{v["edge_antisymmetric_max_abs"]:.3f}</td><td>{pct:.1f}%</td>' if pct is not None else
                 f'<tr><td>{html.escape(k)}</td><td>{v["official_motion_p90_px"]:.3f}</td>'
                 f'<td>{v["center_dx_abs_p90"]:.3f}</td><td>{v["edge_antisymmetric_abs_p90"]:.3f}</td>'
                 f'<td>{v["edge_antisymmetric_max_abs"]:.3f}</td><td>—</td>')
        s.append(f'<td>{status}</td></tr>')
    s.append('</tbody></table>')
    return ''.join(s)
def main(case=CASE):
    global CASE,DEST,BUNDLE,SHORTLIST,CC
    CASE=case
    DEST=OUT/CASE
    BUNDLE=Path("D:/hongda/app/YuySuperEIS/artifacts/b38_restore_bundles_20261020_172849")/CASE
    res=json.loads((DEST/"real_pixel_comparison.json").read_text(encoding="utf-8"))
    assert res["case"]==CASE and len(res["variants"])>=1
    SHORTLIST=["GOLDEN"]+[v for v in ("floor108_slew0p0015","floor110_slew0p0015") if v in res["variants"]]
    CC={"GOLDEN":GOLDEN,"floor108_slew0p0015":C108,"floor110_slew0p0015":C110}
    original=np.fromfile(ROOT/f"visual_tripod_{CASE}.f64",dtype=np.float64).reshape(-1,3,3)
    oracle=np.fromfile(BUNDLE/"oracle_matrices.f64",dtype=np.float64).reshape(-1,3,3)
    rawscale=scale(original@np.linalg.inv(oracle))
    mesh0=np.fromfile(ROOT/f"gemini_mesh_{CASE}.f32",dtype=np.float32).reshape(-1,20,33,2)
    m0=mesh_stats(mesh0)
    series={};metrics=[]
    x=np.arange(len(rawscale)).tolist()
    for name in SHORTLIST:
        if name=="GOLDEN":series[name]={"scale":rawscale,"width":m0[0],
                        "margin":m0[1],"stats":res["GOLDEN"]};continue
        path=DEST/name
        vis=np.fromfile(path/"visual_governed.f64",dtype=np.float64).reshape(-1,3,3)
        et=vis@np.linalg.inv(oracle)
        mesh=np.fromfile(path/"mesh_governed.f32",dtype=np.float32).reshape(-1,20,33,2)
        wr,mar=mesh_stats(mesh)
        meta=json.loads((path/"manifest.json").read_text(encoding="utf-8"))
        encode=json.loads((path/"render_gate.json").read_text(encoding="utf-8"))
        assert len(vis)==len(mesh)==291 and encode["ffmpeg_decoder_errors"]==0
        series[name]={"scale":scale(et),"width":wr,
                      "margin":mar,"stats":res["variants"][name]}
        metrics.append((name,cxx_core_us(meta)))
    # Charts: same exact 291 identities; three lines only for clarity.
    bundle=json.loads((BUNDLE/"bundle.json").read_text(encoding="utf-8"))
    times=[int(frame["sensor_ts_ns"]) for frame in bundle["frames"]]
    assert len(times)==len(rawscale)==291
    stage_input=svg_plot("Camera真实输入SensorTS相邻帧时长","ms",
                 [("SensorTS",x[1:],(np.diff(times)*1e-6).tolist(),GOLDEN)],fmt=".2f")
    legend=lambda field:[(name,x,series[name][field].tolist(),CC[name]) for name in SHORTLIST]
    scale_chart=svg_plot("Oracle之外的Stage2视觉Scale（联合Zoom控制只调这条，勿误当Zoom实际值）",
                    "Scale ratio",legend("scale"),fmt=".3f")
    width_chart=svg_plot("最终RS19采样网格：中行左右跨度（有效Zoom和视觉Scale的共同结果）",
                    "raw-source px",legend("width"),fmt=".0f")
    margin_chart=svg_plot("20×33实际RS19网格最小源像素余量（越界风险）",
                    "source px",legend("margin"),fmt=".0f")
    originals=[series[name]["stats"] for name in SHORTLIST]
    # Use frame wise actual video pixels, not mesh proxies.
    def pixel_series(name,key):
        d={r["frame"]:r[key] for r in series[name]["stats"]["roi_series"]}
        return [d.get(i) for i in x]
    edge_abs=svg_plot("真实输出像素：左右反向伸缩事件区间（Source frame 50–75）",
                   "px per frame",
                   [(name,list(range(50,76)),
                     [pixel_series(name,"edge_breath")[i] for i in range(50,76)],
                     CC[name]) for name in SHORTLIST],
                   fmt=".1f")
    center=svg_plot("真实输出像素：中心背景水平移动（Source frame 50–75）",
                    "px per frame",
                    [(name,list(range(50,76)),
                      [pixel_series(name,"center_dx")[i] for i in range(50,76)],
                      CC[name]) for name in SHORTLIST],
                    fmt=".1f")
    edge_full=svg_plot("真实输出：左右反向伸缩完整291帧（观测偏差，非虚拟摄像机目标）",
                       "px per frame",[(name,x,pixel_series(name,"edge_breath"),CC[name])
                        for name in SHORTLIST],fmt=".1f")
    b=res["GOLDEN"];best=res["variants"]["floor108_slew0p0015"]
    best_peak=min(v["edge_antisymmetric_max_abs"] for v in res["variants"].values())
    spatial_file=DEST/"spatial_4dof_vs_8dof_model_heldout.json"
    spatial_evidence=('<a href="spatial_4dof_vs_8dof_model_heldout.json">4DoF/8DoF真实像素留出验证</a>'
                      if spatial_file.exists() else "本段尚未测试自由8DoF的独立留出特征")
    sources=["<option value='"+html.escape(k)+"'>"+html.escape(k)+"</option>"
             for k in res["variants"]]
    html_source=String_HTML_START+f"""
<h1>Gemini Tripod V2 · 全画幅几何稳定</h1>
<p class="subtitle">2026-10-24 · 原始{CASE} 2560×1440、291帧 SourceID4–294 · RS19 20×33 ONE-WARP · 1280×720最终H264</p>
<div class="strip"><b>阶段状态：PC效果实验已完成，尚无新黄金 PASS。</b> 原Gemini V1效果保留；
V2 Zoom–Scale受控候选最严重边缘伸缩从{b["edge_antisymmetric_max_abs"]:.2f}px降至{best_peak:.2f}px，当前仍需保留中心和日常边缘稳定的全部条件。
此实验仍使用离线未来帧的Oracle/Visual，不能算Android零前瞻实时。</div>
<section><h2>1 · 输入 Input</h2>
<p>输入固定为 b38_restore_bundles_20261020_172849/{CASE}/source.mp4，2560×1440原始未旋转；bundle原生SourceID4–294，一份同源SensorTS及RS19行采样；只在最后做180°显示旋转。禁止把已经0.8x裁剪/倒转的1280×720比较SOURCE当RAW。</p>{stage_input}</section>
<section><h2>2 · 识别 Recognition</h2>
<p>分解原V1 Oracle与二级视觉Similarity：缩放跳变被识别为在转向极值的Stage2附加Scale异常，且Scale控制不篡改输出光学中心映射。下图是前向Scale状态机的输出，不代表整个上游视觉Oracle也是因果的。</p>
{scale_chart}</section>
<section><h2>3 · 控制 Control</h2>
<p>主要候选：绿色Zoom输入下限1.08＋Scale对数每帧限速0.15%；如果有橙线则为下限1.10。C++ core仍负责实际Zoom增加/安全回拉，所以这里的“下限”<b>不是每帧真实currentZoom</b>。不允许为了指标放宽PAN/RS几何。</p>{width_chart}</section>
<section><h2>4 · 几何 Geometry</h2>
<p>Y1/Gemini定义RS19为19个区间、20×33＝660个顶点，不可与某些Y3的19行定义混用。所有候选在原C++ Rs19OneWarp 中实际重算，0个UV越界，所有H264完整解码；这只证明基础几何安全，不证明不变形。</p>{margin_chart}</section>
<section><h2>5 · 输出 Output（最终真实像素）</h2>
<h3>四级指标总表</h3>
{tabular(res)}
<p><b>Gate：</b>全图p90/中心p90/日常边缘反向伸缩p90均不得高于黄金5%，极值max需显著下降。各段表内有单段Gate，整个项目仍未满足四段统一Gate，严禁把某一局部指标变好宣称“已定住全画面”。</p>
{edge_abs}{center}{edge_full}
<h3>黄金与V2实视频并排</h3>
<div class="videos"><div><h4>原 Gemini V1 GOLDEN</h4><video id="gold" controls muted preload="metadata" src="../../../rendered_preview_fixed/{CASE}_GEMINI_STABLE.mp4"></video></div>
<div><h4>选择对比的V2候选</h4><select id="variant">{"".join(sources)}</select><video id="candidate" controls muted preload="metadata"></video></div></div>
<div class="controls"><button onclick="sync()">同步时间轴</button><button onclick="atEvent()">定位第67帧极值</button><button onclick="playBoth()">同时播放</button><button onclick="pauseBoth()">全部暂停</button></div>
<p class="note">对于新候选，同源291帧均通过 libx264 H264 + FFmpeg完整解码。上表源图与成品的SourceID/TS全相同；对比重点在67、68帧以及各次往复极值。H264的码率或视频大小不能代替防抖像素指标。</p>
</section>
<section><h2>6 · 性能 Performance / 最终裁决</h2>
<p>纯C++因果Scale状态机Host测试独立耗时约0.21μs/帧；原C++ RS19整体计算仅PC测量约0.7–1.0ms/帧，不含生成V1非因果视觉Oracle、图像预稳定、LK、全尺寸GPU Warp、EGL/编码与MT6768真机热功耗。<b>Android Zero Lookahead NOT_TESTED</b>。</p>
<p>约束范围已找到：最坏边缘伸缩可以大幅下降，但所测控制律对日常边缘稳定形成权衡。且在96对留出空间特征中自由8DoF Homography并未稳定胜过Similarity；不能立即放开8DoF网格。</p>
<p>证据：<a href="real_pixel_comparison.json">最终输出光流及逐帧ROI</a>、
{spatial_evidence}、
项目实验设计文件 experiments/tripod_v2/README.md；尚未推进DEVICE/Android。</p>
</section>""" + String_HTML_END
    # Avoid overriding imported html namespace.
    dest=DEST/"index.html"
    dest.write_text(html_source,encoding="utf-8")
    top=OUT/"index.html"
    top.write_text(String_HTML_START+f"""<h1>Gemini Tripod V2 全画幅几何稳定</h1>
<p>实验状态：HOST_OFFLINE，原Gemini V1保留，Android实时零前瞻未通过。</p>
<p><a href="{CASE}/index.html">打开024630七组Zoom–Scale同源视频实测与专业图表</a></p>
<p>输入→识别→控制→几何→输出→性能全链路，当前尚无黄金级别新候选。</p>"""+String_HTML_END,encoding="utf-8")
    outdata={
      "case":CASE,"status":"HOST_OFFLINE_NOT_ANDROID_REALTIME",
      "plots":7,"variants":len(res["variants"]),
      "golden_kept":True,"new_golden_pass":False,
      "peak_original":b["edge_antisymmetric_max_abs"],
      "best_center_preserved_floor108_p90":best["center_dx_abs_p90"],
      "html_report":str(dest),
      "offline_anchor_future_frames":True,
      "nonrigid_mesh_unlocked":False}
    (DEST/"report_manifest.json").write_text(
        json.dumps(outdata,ensure_ascii=False,indent=2),encoding="utf-8")
    print("V2_HTML_REPORT_READY",json.dumps(outdata,ensure_ascii=False),flush=True)

String_HTML_START="""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/><title>Gemini Tripod V2 全画幅几何证据</title>
<style>
:root{color-scheme:dark}body{background:#0d1420;color:#d7e2f0;max-width:1240px;margin:30px auto;padding:0 20px;font:15px/1.65 system-ui,"Microsoft YaHei",sans-serif}
h1{font-size:29px;line-height:1.25}h2{font-size:20px;color:#f4f7fb;margin:0 0 12px}h3{font-size:15px;color:#f4f7fb;margin:0 0 8px}
h4{margin:0 0 6px}.subtitle,.note{color:#aabacc}.strip{padding:15px;border:1px solid #56677e;border-radius:10px;background:#192637;margin:18px 0}
section{background:#141e2c;border:1px solid #2c394c;padding:19px;margin-top:17px;border-radius:12px}
.chart{background:#111b28;padding:15px;margin-top:13px;border-radius:9px}.chart .svgscroll{width:100%;overflow:auto}
.chart svg{min-width:700px}.legend{display:flex;flex-wrap:wrap;gap:12px;font-size:12px;color:#c9d7e9}
.legend span{display:inline-flex;gap:6px;align-items:center}.legend em{display:inline-block;width:15px;height:4px;border-radius:5px}
.legend .unit{color:#8fa2b8}table{border-collapse:collapse;width:100%;display:block;overflow:auto;white-space:nowrap;font-size:12px}
th,td{border-bottom:1px solid #2c394c;padding:8px 10px;text-align:right}th:first-child,td:first-child{text-align:left}
.results tr:nth-child(2n){background:#192637}
.videos{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:15px}
video{width:100%;background:#05090e;aspect-ratio:16/9;display:block;border-radius:9px}
button,select{background:#253b59;color:#e5f2ff;border:1px solid #56718d;padding:8px 12px;border-radius:7px;font:inherit;cursor:pointer}
.controls{display:flex;flex-wrap:wrap;gap:10px;margin-top:14px}
a{color:#82bdf7}@media(max-width:850px){.videos{grid-template-columns:1fr}}
</style></head><body>"""
String_HTML_END="""<script>
const paths = document.querySelectorAll("select#variant")[0];
const vid = document.querySelector("#candidate");
function changeV(){
 if(!paths)return;
 vid.src=paths.value+"/gemini_v2_output.mp4";
 vid.load();
}
if(paths){
 paths.value="floor108_slew0p0015";
 paths.addEventListener("change",changeV);
 changeV();
}
const a=document.querySelector("#gold"),b=vid;
function sync(){if(!a||!b)return;b.currentTime=a.currentTime;}
function atEvent(){if(!a||!b)return;a.pause();b.pause();a.currentTime=67/29.605;b.currentTime=67/29.605;}
function playBoth(){if(!a||!b)return;sync();a.play().catch(()=>{});b.play().catch(()=>{});}
function pauseBoth(){if(!a||!b)return;a.pause();b.pause();}
</script></body></html>"""
if __name__=="__main__":
    import argparse
    a=argparse.ArgumentParser()
    a.add_argument("--case",default="024630")
    o=a.parse_args()
    main(o.case)
