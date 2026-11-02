#include "CausalScaleGovernor.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <vector>
using gemini::tripod_v2::CausalScaleGovernor;
using gemini::tripod_v2::ScaleGovernorConfig;
int main(){
    ScaleGovernorConfig cfg{};
    CausalScaleGovernor gov(cfg);
    const std::int64_t T=33000000;
    const std::vector<double> signal{1.,1.001,1.002,.94,.943,.98,1.01,1.001};
    std::vector<double> output;
    for(size_t i=0;i<signal.size();++i){
        auto s=gov.update(signal[i],T*(i+1));
        assert(std::isfinite(s));
        if(!output.empty()) assert(std::abs(std::log(s/output.back()))<=.00150000001);
        output.push_back(s);
    }
    // Future measurements cannot retroactively influence earlier outputs.
    for(size_t prefix=1;prefix<=signal.size();++prefix){
        CausalScaleGovernor replay(cfg);
        for(size_t i=0;i<prefix;++i){
            auto s=replay.update(signal[i],T*(i+1));
            assert(std::abs(s-output[i])<1e-13);
        }
    }
    // Fail closed on invalid, duplicated and backward timestamps.
    auto last=output.back();
    assert(std::abs(gov.update(-2.,T*9)-last)<1e-13);
    assert(std::abs(gov.update(1.1,T*8)-last)<1e-13);
    assert(std::abs(gov.update(NAN,T*9)-last)<1e-13);
    gov.reset();
    assert(!gov.hasValue());
    assert(std::abs(gov.update(1.0,T)-1)<1e-13);
    // Benchmark ONLY this state machine, not gyro, visual LK, RS or encoder.
    gov.reset();
    volatile double sum=0;
    const auto t0=std::chrono::steady_clock::now();
    for(int i=0;i<2000000;++i){
        sum+=gov.update(1.0+0.035*std::sin(i*.01),T*static_cast<std::int64_t>(i+1));
    }
    const auto t1=std::chrono::steady_clock::now();
    const double ns=std::chrono::duration<double,std::nano>(t1-t0).count()/2000000;
    std::printf("TRIPOD_V2_CAUSAL_CPP_TEST_PASS output_last=%0.9f isolated_scale_governor_avg_ns=%.2f sum=%0.2f\n",
          output.back(),ns,static_cast<double>(sum));
    return 0;
}
