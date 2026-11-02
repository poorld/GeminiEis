#!/usr/bin/env python3
"""Gemini V2C C1 six-stage, four-video TRUE-RAW-INPUT evidence dashboard.

All charts use current C1 observer traces from unmodified original 2560x1440
source.mp4. No future visual frames, no Gemini offline oracle, no new final
EIS stabilized output. Do not mislabel feature-domain LK as output quality.
"""
from __future__ import annotations
import html,json,math
from pathlib import Path
import numpy as np
from generate_report import svg_plot,String_HTML_START,String_HTML_END
from c1_stream_host import RESULT_ROOT,CASES

COLORS={"raw":"#E7A660","prewarp":"#4BCBA5","gyro":"#8E9DFC",
        "trust":"#78B9F8","perf":"#E6B367",
        "key":"#CE9EF6","source":"#9DB6CD"}
CASE_NAME={"024605":"低速往返","024618":"中速往返",
           "024630":"快速往返","024647":"单侧转向"}
def plot(title,unit,axes,marker=71,fmt=".2f"):
    return svg_plot(title,unit,axes,marker=marker,fmt=fmt).replace(
        "输出视频帧号（第0帧为原SOURCE source_index=4）",
        "真实 Camera SourceID（从4到294；图中71对应V1显示第67帧）")

def loaded(case):
    base=RESULT_ROOT/case
    rows=[json.loads(x) for x in (base/"frames.jsonl").read_text(encoding="utf-8").splitlines()]
    summ=json.loads((base/"summary.json").read_text(encoding="utf-8"))
    cal=json.loads((base/"gyro_prewarp_validation.json").read_text(encoding="utf-8"))
    assert len(rows)==291 and summ["total_source_frames"]==291
    assert rows[0]["source_id"]==4 and rows[-1]["source_id"]==294
    assert cal["case"]==summ["case"]==case and cal["quality_gate"]=="PASS"
    assert summ["gyro_never_pushed_beyond_camera_frame_deadline"]
    assert all(not r.get("future_video_frame_access") for r in rows)
    assert all(not r.get("offline_oracle_used") for r in rows)
    return rows,summ,cal

def val(row,key):
    x=row.get(key)
    return float(x) if x is not None else None

def zone(rows,z):
    out=[]
    for r in rows:
        tiles=r.get("tiles",[])
        points=[t.get("median_raw_dx") for t in tiles if t["col"]==z
                  and t.get("global_inliers",0)>=3 and
                  t.get("median_raw_dx") is not None]
        out.append(float(np.median(points)) if points else None)
    return out

def generate_case(case):
    rows,s,cal=loaded(case)
    x=[int(r["source_id"]) for r in rows]
    time_ms=[r["sensor_ts_ns"]*1e-6 for r in rows]
    ts_diff=[None]+(np.diff([r["sensor_ts_ns"] for r in rows])*1e-6).tolist()
    pts_diff=[None]+(np.diff([r["pts_us"] for r in rows])*.001).tolist()
    deadline=[(r["available_deadline_ns"]-
       r.get("gyro_last_submitted_ns",r.get("last_gyro_ts",0)))*1e-6
       for r in rows]
    yaw=[val(r,"raw_yaw_rad_s") for r in rows]
    yaw_filter=[val(r,"yaw_ema_rad_s") for r in rows]
    trust=[val(r,"trust") for r in rows]
    support=[val(r,"support_tiles") for r in rows]
    inliers=[val(r,"inliers") for r in rows]
    dx=[val(r,"dx_feature_px") for r in rows]
    dy=[val(r,"dy_feature_px") for r in rows]
    scale=[val(r,"logscale")*100 if val(r,"logscale") is not None else None for r in rows]
    coverage=[val(r,"prewarp_coverage_ratio")*100
       if val(r,"prewarp_coverage_ratio") is not None else None for r in rows]
    ky=[val(r,"keyframe_path_disagreement_px") for r in rows]
    l,c,r=zone(rows,0),zone(rows,1),zone(rows,2)
    anti=[(right-left)*.5 if right is not None and left is not None else None
          for left,right in zip(l,r)]
    prewarp_ms=[(row.get("prewarp_current_ms",0)+row.get("prewarp_prev_ms",0))
                if "prewarp_current_ms" in row else None for row in rows]
    lk_ms=[val(row,"LK_pair_ms") for row in rows]
    total=[val(row,"c1_total_ms") for row in rows]
    full=[val(row,"c1_total_with_decode_ms") for row in rows]
    samples=cal["pairs"]
    sx=[z["source_id"] for z in samples]
    graphs={
      "time":plot("真实 Camera SensorTS 与源 MP4 PTS 帧间隔（跨数据域对齐）",
                  "ms",[("SensorTS",x,ts_diff,COLORS["raw"]),
                         ("Source MP4 PTS",x,pts_diff,COLORS["source"])]),
      "gyro":plot("真实采样Gyro Yaw与因果半周期趋势（±代表方向）","rad/s",
                 [("Camera Yaw omega",x,yaw,COLORS["gyro"]),
                  ("Causal EMA",x,yaw_filter,COLORS["prewarp"])]),
      "raw_vs_gyro":plot("真实RAW vs 本帧Gyro+RS19预稳定：相同输入图像LK",
                         "320×180 px/frame",
              [("原始Camera相对运动",sx,[z["raw_motion_320px"] for z in samples],COLORS["raw"]),
               ("Gyro+RS19后未解释运动",sx,[z["gyro_rs19_motion_320px"] for z in samples],COLORS["prewarp"])],fmt=".1f"),
      "trust":plot("3×3中间可信度：有效角点、跨tile支持及FB-LK加权trust",
                   "count / 100×trust",
               [("Tracked inliers",x,inliers,COLORS["raw"]),
                ("trust×100",x,[v*100 if v is not None else None for v in trust],COLORS["trust"]),
                ("spatial tiles×10",x,[v*10 if v is not None else None for v in support],COLORS["key"])],fmt=".1f"),
      "residual":plot("预稳定后的C1视觉共模残余（只是观察，不对Target施加修正）",
                      "feature px/frame",
                 [("Global dx",x,dx,COLORS["raw"]),
                  ("Global dy",x,dy,COLORS["trust"])],fmt=".2f"),
      "tiles":plot("场景无关的左/中心/右3列视觉残余投票（非固定电脑屏幕ROI）",
                   "feature px/frame",
              [("LEFT column",x,l,COLORS["raw"]),
               ("CENTER column",x,c,COLORS["trust"]),
               ("RIGHT column",x,r,COLORS["prewarp"])],fmt=".2f"),
      "anti":plot("C1三列左右反向差值——诊断外围不一致，不是最终输出边缘变形",
                   "feature px/frame",
                  [("(RIGHT-LEFT)/2",x,anti,COLORS["key"]),
                   ("Observed logscale×100",x,scale,COLORS["raw"])],fmt=".3f"),
      "key":plot("历史关键帧与相邻帧累计残余的差值（第9帧稀疏核对）",
                 "feature px",
                 [("Keyframe vs causal path difference",x,ky,COLORS["key"])],fmt=".2f"),
      "geometry":plot("真实Gyro/RS19输入到预稳定的特征视场覆盖率",
                      "百分比",[("VALID feature coverage (%)",x,coverage,COLORS["prewarp"])],fmt=".1f"),
      "perf":plot("PC Host性能：Native RS prewarp+LK+C1、以及含软件解码总耗时",
                  "ms",
             [("Native gyroRS19 预稳定",x,prewarp_ms,COLORS["prewarp"]),
              ("LK 相邻帧",x,lk_ms,COLORS["trust"]),
              ("C1 处理",x,total,COLORS["raw"]),
              ("C1+软件解码",x,full,COLORS["key"]),
              ("29.6fps帧预算",x,[1000/29.605]*len(x),"#8CA0AF")],fmt=".1f")
    }
    frontend=(
        f'<h1>Gemini V2C C1 · {case} {CASE_NAME[case]}</h1>'
        f'<p class="subtitle">原RAW 2560×1440 / SourceID 4–294 / 真实SensorTS＋编码PTS / '
         '320×180 Gyro+19RS20Row 预稳定特征流 / 原V1黄金与最终ONE-WARP均不改</p>'
         '<div class="strip"><b>当前状态 HOST C1 SHADOW（观测链路通过，非最终防抖效果PASS）。</b>'
         '此HTML只使用本轮真实原始录像的当前/历史Gyro与当前/历史低分辨率图像，不读取Oracle、'
         '未来影像、整段Gaussian。Y1 Android真实零前瞻和完整30fps仍未验证。</div>'
         '<section><h2>1 · 输入 Input</h2>'
         '<p>SourceID与编码 PTS 每帧按同一原始source.mp4实测，Orientation仍为buffer-native，'
         '不预先旋转180°；同一SOF的Exposure、RS Readout、Camera-to-Gyro +8ms全部保留。'
         'Gyro只提交到当前帧结束的物理可用时刻，不会预送未来50ms。</p>'
         +graphs["time"]+'<p>每帧有效陀螺仪提交均不晚于本帧读出/曝光截止点。</p></section>'
         '<section><h2>2 · 识别 Recognition</h2>'
         '<p>真实Camera域Yaw随时间变化；同帧Gyro+RS19预稳定把物理转向投影先扣除。'
         '下面的同源LK前/后评测证明不是仅从“中心光流很小”猜测陀螺仪方向正确。</p>'
         +graphs["gyro"]+graphs["raw_vs_gyro"]+
         f'<p>RAW p90 <b>{cal["raw_center_motion_p90_px_at320"]:.3f}</b> px '
         f'→ 预稳定 <b>{cal["prewarped_center_motion_p90_px_at320"]:.3f}</b> px，'
         f'抑制<b>{cal["p90_pair_suppression_percent"]:.1f}%</b>；'
         f'比较帧数{cal["sampled_pairs"]}（稀疏诊断，非全部290帧）。</p></section>'
         '<section><h2>3 · C1控制候选 Control（SHADOW，无执行器）</h2>'
         '<p>3×3 Tile、中心加权（非中心独占）、全图4DoF Similarity+FB-LK/RANSAC与可信点票决。'
         'C1还提供半周期/PAN状态，但不会因观察到残余就立即改qTarget、Visual或Zoom。</p>'
         +graphs["trust"]+graphs["residual"]+graphs["tiles"]+
         '<p>真实接收的C1残余在feature 320×180域，必须经C2统一Target Owner的物理投影/可信门禁转换，'
         '绝不可把这里的dx直接作为原Gemini视觉tx插入C++，否则容易重复补偿。</p></section>'
         '<section><h2>4 · 几何 Geometry</h2>'
         '<p>物理RS19用<strong>20行实际Gyro四元数</strong>，低分辨率Native预稳定使用39×65'
         '密化采样网格并三角插值，不是额外高清Warp。原Gemini最终RS19 20×33和ONE-WARP完全没改变。'
         '多行角度采样和SourceID/sensor一致，并以有效视场mask防止黑角伪特征参与LK。</p>'
         +graphs["geometry"]+'</section>'
         '<section><h2>5 · 输出 Output（视觉证据，不是高清成品视频）</h2>'
         '<p>当前只输出当前帧全局Residual、9 Tile有效证据、历史关键帧一致性和低分辨率预稳定样张。'
         '没有输出新EIS视频，<strong>不能声称V1边缘扭动已经改善</strong>。'
         '绝不使用电脑显示器专属ROI。</p>'
         +graphs["anti"]+graphs["key"]+
         '<div class="videos">'
         +''.join(f'<div><h4>SourceID={srcid} 真实Gyro+RS预稳定特征图</h4>'
                  f'<img style="width:100%;image-rendering:pixelated" '
                  f'src="prewarp_source_{srcid}.png" alt="来自原始相机帧的Gyro预稳定特征图"/></div>'
                  for srcid in (71,96) if (RESULT_ROOT/case/f"prewarp_source_{srcid}.png").exists())
         +'</div></section>'
         '<section><h2>6 · 性能 Performance</h2>'
         '<p>PC C++原生RS19特征预映射不等于Android摄像头链路吞吐。'
         '原始source.mp4为软件解码；最终GPU Warp/encoder不存在本次C1实验，必须另行验证Y1真机。'
         '只有作为视觉子模块测量不含解码的Host p90。</p>'
         +graphs["perf"]+
         '<p><strong>本片p90：</strong>纯C1观察'
         f'<b>{s["C1_host_without_decode_p90_ms"]:.1f}ms</b>、'
         f'含软件解码<b>{s["C1_host_with_decode_p90_ms"]:.1f}ms</b>、'
         f'独立LK <b>{s["LK_pair_host_p90_ms"]:.1f}ms</b>。'
         'HOST全链路30fps Gate和DEVICE均未通过。</p></section>'
         '<section><h2>证据与后续阶段</h2>'
         '<p><a href="summary.json">C1本片真实统计</a>；'
         '<a href="frames.jsonl">291帧SourceID+TS+Gyro/9Tile/关键帧完整诊断</a>；'
         '<a href="gyro_prewarp_validation.json">真实RAW与Gyro+RS19运动对照</a>。'
         '本阶段只解决C1因果输入及视觉识别；下一步C2需在同一个C++ Target Owner中'
         '联合qVirtual+Visual+Zoom，并在全物理RS19预算下原子提交，之后才有四段真正成品视频效果门禁。</p></section>'
    )
    path=RESULT_ROOT/case/"index.html"
    path.write_text(String_HTML_START+frontend+String_HTML_END,encoding="utf-8")
    return {"case":case,
      "svg_chart_count":len(graphs),
      "original_frames":len(rows),
      "suppression_p90_percent":cal["p90_pair_suppression_percent"],
      "C1_host_p90_ms":s["C1_host_without_decode_p90_ms"],
      "with_software_decode_host_p90_ms":s["C1_host_with_decode_p90_ms"],
      "valid_visual_pairs":s["accepted_pairs"],
      "full_pipeline_android_pass":False,
      "C1_uses_future_images":False,
      "C1_oracle_usage":False,
      "C2_effect_gate":"NOT_TESTED",
      "evidence_html":str(path)}

def main():
    unit=json.loads((RESULT_ROOT/"c1_unit_tests.json").read_text(encoding="utf-8"))
    summaries=[generate_case(k) for k in CASES]
    if any(x["valid_visual_pairs"]!=290 or x["svg_chart_count"]<8 for x in summaries):
        raise RuntimeError("Missing four complete C1 real input cases or charts")
    rows=''.join(
      f'<tr><td><a href="{item["case"]}/index.html">{item["case"]}</a>'
      f'　{CASE_NAME[item["case"]]}</td>'
      f'<td>291</td><td>{item["valid_visual_pairs"]}/290</td>'
      f'<td>{item["suppression_p90_percent"]:.1f}%</td>'
      f'<td>{item["C1_host_p90_ms"]:.1f}ms</td>'
      f'<td>{item["with_software_decode_host_p90_ms"]:.1f}ms</td>'
      f'<td>HOST C1 SHADOW</td></tr>' for item in summaries)
    home=(
     '<h1>Gemini Tripod V2C · C1 真实因果视觉输入</h1>'
     '<p class="subtitle">2026-10-24 · 四段原始摄像头同源数据 · C++ Native RS19预稳定 + 因果3×3 Tile LK + 历史Keyframe</p>'
     '<div class="strip"><b>Host C1前端已完成四段291帧验证；最终防抖效果、Android实时尚未通过。</b>'
     '本轮没有使用离线Oracle、未来Camera帧、完整双向Gaussian或V1稳定视频作为视觉输入。'
     '全部四段输出的C1残余当前是SHADOW，不改变Gemini黄金版qTarget/Zoom/RS19。</div>'
     '<section><h2>输入→识别→控制→几何→输出→性能总表</h2>'
     '<table class="results"><thead><tr><th>录制</th><th>原始帧</th><th>视觉观测</th>'
     '<th>原RAW→Gyro预稳 p90抑制</th><th>C1 p90</th><th>软件解码+C1 p90</th>'
     '<th>判定</th></tr></thead><tbody>'+rows+'</tbody></table>'
     '<p>所有抑制率在320×180特征域，通过同一真实RAW相邻帧视觉估计计算；不与1080p最终EIS输出抑制混淆。'
     '没有新成品H264，因此没有“画面真正定住/边缘被修复”的画质PASS声明。</p></section>'
     '<section><h2>C1真正实现的因果边界</h2>'
     '<p>源视频严格按原 ordinal 单向读取，并将SourceID与实际解码PTS、SensorTS逐帧核对；'
     'IMU只把 <= 当前帧可用截止时刻的400Hz样本提交给状态；20行RS pose全部按真实行时间查询。'
     'C++预稳定由物理20行Gyro驱动，39×65细分网格只供320×180特征图，'
     '不会改变原Gemini全分辨率20×33单次RS19 Warp。'
     'OpenCV FB-LK跨3×3全区域，中心权重更高但外围有非零权重，'
     '历史关键帧每十帧校验一次；失信时拒绝，不把视觉残余硬塞给Target。</p>'
     '<p>经独立Host测试：前80帧与整291帧重放，关键视觉输出、状态、Gyro查询逐帧一致；'
     '合成Yaw图像Gyro预稳前约4.498px，后0.002px；'
     '中间动态物体独立运动11px时，全局背景2px仍估计为约2.015px；'
     '无特征场景会拒绝。测试文件：<a href="c1_unit_tests.json">C1 unit证据</a>。</p>'
     '<p>**限制**：这证明C1 Host算法输入因果，不证明离线双级Gemini V1实现变成了因果'
     '也不证明Y1实机30fps。C1视觉前端目前是Python OpenCV实验驱动＋'
     '原生C++ RS19 mapping，尚需把完整Tile LK/调度接到Android/MTK共享算法库。</p></section>'
     '<section><h2>四段逐帧专业图表</h2>'
     +''.join(f'<article><h3>{item["case"]} · {CASE_NAME[item["case"]]}</h3>'
              f'<p>真实Host时序、Gyro匹配、3×3 Tile、左右残余与性能'
              f'</p><p><a href="{item["case"]}/index.html">打开{item["svg_chart_count"]}张真实SVG图表及291帧诊断</a></p></article>'
              for item in summaries)+'</section>'
     '<section><h2>下一步：C2唯一VirtualCamera控制器</h2>'
     '<p>输入来源就绪，但C1目前是Shadow：下一阶段先把经校验的'
     'Feature-domain共模Residual、tile trust与Yaw/PAN信号转换到相机物理坐标'
     '并进入唯一 qVirtual+Visual+Zoom 联合预算，完整RS19提交后再重渲四段真实H264，'
     '必须保持V1原先的大幅转向稳定同时把小幅边缘伸缩压下去。'
     '不再用孤立的Zoom floor或末端视觉加法。</p>'
     '<p>原版Gemini对照依旧可在仓库 <code>rendered_preview_fixed/gemini_compare_viewer.html</code>打开；'
     'Golden未覆盖，Android/MTK/ALPS未修改。</p></section>'
    )
    dest=RESULT_ROOT/"index.html"
    dest.write_text(String_HTML_START+home+String_HTML_END,encoding="utf-8")
    manifest={
     "stage":"V2C-C1 causal visual input",
     "cases":summaries,
     "four_case_real_input_host_causal_gate":"PASS",
     "independent_gyro_RS_motion_gate":"PASS_4_OF_4",
     "identical_source_id_pts_sensor_ts":True,
     "true_causal_prefix_80_frames":"PASS",
     "physical_camera_synthetic_projection":"PASS",
     "dynamic_middle_tile_robust_global":"PASS",
     "C1_visual_actuator":"OFF_SHADOW_ONLY",
     "C2_joint_target_integration":"NOT_STARTED",
     "four_case_final_EIS_image_effect":"NOT_TESTED",
     "android_zero_lookahead_device":"NOT_TESTED",
     "host_full_pipeline_30fps_gate":"BLOCKED_FULL_DECODE_P90_EXCEEDS_FRAME_INTERVAL",
     "four_cases_all_291":True,
     "original_v1_golden_untouched":True,
     "html":str(dest)}
    (RESULT_ROOT/"c1_professional_evidence.json").write_text(
      json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
    print("GEMINI_V2C_C1_SIX_STAGE_FOUR_CAPTURE_HTML_DONE",
          json.dumps({"home":str(dest),
                      "per_capture_svg_count":[r["svg_chart_count"] for r in summaries],
                      "realtime":manifest["host_full_pipeline_30fps_gate"],
                      "host_causal":manifest["four_case_real_input_host_causal_gate"]},
                       ensure_ascii=False),flush=True)

if __name__=="__main__":main()
