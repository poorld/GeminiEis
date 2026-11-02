#!/usr/bin/env python3
"""Build an isolated Unified Tripod V2 Core + C API from unchanged repo source.

No source files in src/ or include/ are rewritten. Header + cpp overrides are
GENERATED solely under reports/gemini_tripod_v2/bin/unified_sources/.
The adapter is PC-only. This does not change Android native binaries.
"""
import pathlib,subprocess,re,sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
BIN=ROOT/"reports"/"gemini_tripod_v2"/"bin"
GENERATED=BIN/"unified_sources"
INC=GENERATED/"include"/"core"
INC.mkdir(parents=True,exist_ok=True)
COMPILER=pathlib.Path("D:/Program Files/mingw64/bin/g++.exe")
HEADER=ROOT/"include"/"core"/"gemini_eis_core.h"
SOURCE=ROOT/"src"/"core"/"gemini_eis_core.cpp"
assert COMPILER.exists() and HEADER.exists() and SOURCE.exists()
h=HEADER.read_text(encoding="utf-8")
needle='#include "warp/rs19_one_warp.h"'
assert h.count(needle)==1
h=h.replace(needle,needle+'\n#include "UnifiedTripodBudget.h"')
needle2='    int frameCount_ = 0;'
assert h.count(needle2)==1
h=h.replace(needle2,needle2+'\n    tripod_v2::UnifiedBudgetController v2_budget_{};')
(INC/"gemini_eis_core.h").write_text(h,encoding="utf-8")
orig=SOURCE.read_text(encoding="utf-8")
assert orig.count('#include <cmath>')==1
orig=orig.replace('#include <cmath>','#include <cmath>\n#include <cstdio>\n#include <cstdlib>\n#include <utility>')
needle3='''    wr.qTarget = qTarget_;
    wr.visual = out.visual;
    wr.zoom = currentZoom_;
    out.mesh = warp_.buildMesh(wr);
    return out;'''
assert orig.count(needle3)==1, 'SOURCE CORE VERSION CHANGED'
new='''    wr.qTarget = qTarget_;
    wr.visual = out.visual;
    wr.zoom = currentZoom_;
    // Exclusive V2 target owner: the baseline V1 qTarget trajectory is still
    // produced inside this very same stateful core. No second hidden core.
    // V2 chooses ONLY output warp and budget after accurate RS19 scrutiny.
    const auto decision = v2_budget_.choose(
        wr, warp_, input.timestampNs, input.angularVelocityRadS.y,
        out.panConfidence);
    wr = decision.warp;
    out.visual = wr.visual;
    currentZoom_ = wr.zoom;
    out.mesh = decision.vertices;
    static const bool kLog = std::getenv("GEMINI_TRIPOD_V2_LOG") != nullptr;
    if (kLog) {
        const auto& d=decision.diag;
        std::fprintf(stderr,
            "V2_BUDGET,%lld,%d,%d,%d,%.9f,%.9f,%.9f,%.9f,"
            "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\\n",
            static_cast<long long>(d.timestamp_ns),d.reason,d.attempted,
            d.accepted,d.margin_px,d.result_margin_px,
            d.span_source_px,d.result_span_source_px,
            d.base_zoom,d.final_zoom,d.base_scale,d.final_scale,
            d.yaw_rad_s,d.center_shift_source_px);
    }
    return out;'''
core=orig.replace(needle3,new)
# Deliberately keep ctor, scheduler, original qAnchor/qTarget and gyro path;
# only one V2 selection at actual output warp after V1 frozen baseline state.
(GENERATED/"gemini_eis_core_v2.cpp").write_text(core,encoding="utf-8")
staticlib=ROOT/"build"/"libgemini_eis_core.a"
assert staticlib.exists()
opts=["-std=c++17","-O2","-DGEMINI_EIS_STATIC",
      "-I"+str(GENERATED/"include"),"-I"+str(ROOT/"include"),
      "-I"+str(ROOT/"experiments"/"tripod_v2")]
units={
    "core":GENERATED/"gemini_eis_core_v2.cpp",
    "api":ROOT/"src"/"api"/"gemini_eis_c_api.cpp",
    "runner":ROOT/"sim"/"replay_dataset.cpp"
}
objects=[]
for key,source in units.items():
    out=GENERATED/(key+".o")
    cmd=[str(COMPILER),*opts,"-c",str(source),"-o",str(out)]
    proc=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=100)
    if proc.returncode:
        print("BUILD_COMPILE_FAIL",key,proc.stdout,proc.stderr,flush=True)
        sys.exit(proc.returncode)
    objects.append(out)
exe=BIN/"replay_unified_tripod_v2.exe"
cmd=[str(COMPILER),"-std=c++17","-O2",
     *(str(o) for o in objects),str(staticlib),"-o",str(exe)]
proc=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,timeout=100)
if proc.returncode:
    print("BUILD_LINK_FAIL",proc.stdout,proc.stderr,flush=True)
    sys.exit(proc.returncode)
print("V2_ISOLATED_UNIFIED_CORE_BUILT",exe, "object_files",len(objects),
      "original_src_untouched",True,flush=True)
