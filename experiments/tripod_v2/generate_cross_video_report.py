#!/usr/bin/env python3
"""Final all-four case browser report. Never promotes an unsafe V2."""
from __future__ import annotations
import html,json,sys
from pathlib import Path
from generate_report import main as render_case, String_HTML_START, String_HTML_END, OUT

CASES=("024605","024618","024630","024647")
VAR="floor108_slew0p0015"
def metrics(d):
    fields=("official_motion_p90_px","center_dx_abs_p90",
            "edge_antisymmetric_abs_p90","edge_antisymmetric_max_abs")
    return [d[k] for k in fields]
def percent(a,b):
    return 100*(a/b-1) if b else float("nan")
def main():
    data={}
    all_ok=True
    for case in CASES:
        source=OUT/case/"real_pixel_comparison.json"
        if not source.exists():raise FileNotFoundError("Full original + V2 actual H264 required: "+str(source))
        payload=json.loads(source.read_text(encoding="utf-8"))
        if VAR not in payload["variants"]:raise RuntimeError("Variant not video-tested: "+case)
        gold,cand=payload["GOLDEN"],payload["variants"][VAR]
        if gold["frames"]!=291 or cand["frames"]!=291:raise RuntimeError("Frame identity mismatch")
        if cand["real_LK_valid_3zones"]!=gold["real_LK_valid_3zones"]:
            raise RuntimeError("LK validity mismatch; re-evaluate under intersection")
        all_ok &= bool(cand["accept_as_golden"])
        render_case(case)
        data[case]={"GOLDEN":gold,"V2":cand,"same_input_frames":291,
                    "mesh_19_intervals_20x33":True,
                    "real_H264_pixel_measurement":True,
                    "android_realtime":False}
    rows=[]
    for case,v in data.items():
        g=v["GOLDEN"];c=v["V2"]
        vals=[case,*metrics(g),*metrics(c)]
        rows.append("<tr>"+f'<td><a href="{case}/index.html">{case}</a></td>'+
            "".join(f'<td>{n:.3f}</td>' for n in metrics(g)+metrics(c))+
            f'<td>{percent(c["official_motion_p90_px"],g["official_motion_p90_px"]):+.1f}%</td>'+
            f'<td>{percent(c["edge_antisymmetric_max_abs"],g["edge_antisymmetric_max_abs"]):+.1f}%</td>'+
            f'<td>{"PASS" if c["accept_as_golden"] else "FAIL"}</td></tr>')
    video_panels=[]
    for case in CASES:
        video_panels.append(f'''
<article><h3>{case} · <a href="{case}/index.html">专业图表和完整证据</a></h3>
<div class="videos"><div><h4>V1 GOLDEN</h4>
<video class="gold" muted controls preload="metadata" src="../../rendered_preview_fixed/{case}_GEMINI_STABLE.mp4"></video></div>
<div><h4>V2 1.08 Zoom floor + causal Stage2 Scale</h4>
<video class="candidate" muted controls preload="metadata" src="{case}/{VAR}/gemini_v2_output.mp4"></video></div></div>
<div class="controls"><button onclick="syncPair(this)">同步本组时间</button><button onclick="playPair(this)">播放本组</button><button onclick="pausePair(this)">暂停</button></div></article>
''')
    report=String_HTML_START+'''
<h1>Gemini Tripod V2 · 全画幅几何稳定跨四段报告</h1>
<p class="subtitle">2026-10-24 · V1黄金保留 · V2没有通过跨场景画质Gate · 真实同源RAW 2560×1440、20×33 RS19、H264最后像素</p>
<div class="strip"><b>现阶段验收结论：不得升级到 Android/MTK。</b> 试验控制器自身前向因果，但输入仍是离线Oracle与双遍未来帧视觉锚点。四段同源测试发现：在快摆024630可以降低极值伸缩，但在其它录制可能损失日常稳定。因此固定Zoom floor不是最终联合控制器。</div>
<h2>输入→识别→控制→几何→输出→性能</h2>
<p>每段链接的专业报告按六环节排列，画面与所有指标均来自相同SourceID/TS/传感器原图的两路编码输出；不能把PC离线像素PASS冒充Android零前瞻PASS。</p>
<section><h2>1–6 总表：四段 H264 真实像素</h2>
<p>指标均为1280×720输出像素或px/帧。全图Motion p90使用同一官方track_pair；中心/边缘来自独立左中右FB-LK。+代表该指标变差，负代表下降。</p>
<table class="results"><thead><tr><th>录制（详细图表）</th><th>V1全图p90</th><th>V1中心p90</th><th>V1边缘p90</th><th>V1极值max</th><th>V2全图p90</th><th>V2中心p90</th><th>V2边缘p90</th><th>V2极值max</th><th>Motion变化</th><th>极值变化</th><th>本段Gate</th></tr></thead>
<tbody>'''+''.join(rows)+'''</tbody></table>
<p>全部四段均真实解码291帧H264，RS19网格无UV越界，Camera真实PTS和180°显示归正一致。几何仅负责不能产生黑边，无法单独证明像素画质。</p></section>
<section><h2>四段视频：原V1 vs 同源V2</h2>
'''+''.join(video_panels)+'''
</section>
<section><h2>技术方向与实时限制</h2>
<p>快摆极值的主要对立是缩放预算和画面稳定：静态Zoom floor无法兼顾低速与快速转向。下一步研究应在同一个C++虚拟相机Target控制器内，将视觉Scale、RS19真实像素Margin、Zoom及历史Yaw/反向转态组合成可证明的动态预算优化器，而非继续死调全片固定Zoom。</p>
<p>自由8DoF尚未获得独立特征留出验证支持。真正Android零前瞻需要替换掉median(未来20–100帧)、完整Gaussian、离线Oracle，做当前帧低分辨率预稳定＋关键帧因果视觉，再单次全分辨率Warp；Camera/编码耗时和功耗仍为NOT_TESTED。</p>
<p><a href="024630/spatial_4dof_vs_8dof_model_heldout.json">8DoF留出测试</a>、实验研发文档位于GeminiEis/experiments/tripod_v2/README.md。</p>
</section>
<script>
function pair(btn){return btn.closest("article");}
function syncPair(btn){let p=pair(btn),a=p.querySelector(".gold"),b=p.querySelector(".candidate");b.currentTime=a.currentTime;}
function playPair(btn){let p=pair(btn),a=p.querySelector(".gold"),b=p.querySelector(".candidate");b.currentTime=a.currentTime;a.play().catch(()=>{});b.play().catch(()=>{});}
function pausePair(btn){let p=pair(btn);p.querySelector(".gold").pause();p.querySelector(".candidate").pause();}
</script>'''+String_HTML_END
    (OUT/"index.html").write_text(report,encoding="utf-8")
    serialized={}
    for case,d in data.items():
        serialized[case]={"golden":{k:v for k,v in d["GOLDEN"].items() if k!="roi_series"},
                          "candidate":{k:v for k,v in d["V2"].items() if k!="roi_series"}}
    manifest={"capture_count":4,"each_source_frames":291,
              "all_source_and_outputH264":True,
              "all_pixel_gates_PASS":bool(all_ok),
              "android_causal_implementation_PASS":False,
              "original_golden_preserved":True,
              "model_span_and_homography_is_still_research":True,
              "cases":serialized}
    (OUT/"four_video_cross_comparison.json").write_text(
        json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    print("V2_FOUR_VIDEO_PROFESSIONAL_GATE",json.dumps({"cases":4,
        "same_frames_each":291,"overall_effect":"FAIL_NOT_PRODUCT",
        "new_golden_pass":all_ok,
        "html":str(OUT/"index.html")},ensure_ascii=False),flush=True)

if __name__=="__main__":main()
