#include "C1NativeRsPrewarp.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "common/types.hpp"

namespace {
gemini::Quat fromPtr(const double* p) {
    return gemini::Quat{p[0],p[1],p[2],p[3]}.normalized();
}
} // namespace

int gemini_v2c_rs19_prewarp_map(
    int width,int height,
    double fx,double fy,double cx,double cy,
    const double* q_rows20_xyzw,const double* q_ref_xyzw,
    float* map_x,float* map_y,std::uint8_t* valid) {
    if(width<=0||height<=0||width>4096||height>4096||
       !(std::isfinite(fx)&&std::isfinite(fy)&&fx>0&&fy>0)||
       !q_rows20_xyzw||!q_ref_xyzw||!map_x||!map_y||!valid)
        return -1;
    gemini::Quat qs[20];
    for(int i=0;i<20;++i) {
        const double* p=q_rows20_xyzw+i*4;
        for(int c=0;c<4;++c)if(!std::isfinite(p[c]))return -2;
        qs[i]=fromPtr(p);
    }
    for(int c=0;c<4;++c)if(!std::isfinite(q_ref_xyzw[c]))return -2;
    const auto qRef=fromPtr(q_ref_xyzw);
    for(int y=0;y<height;++y){
        const double yo=static_cast<double>(y)+.5;
        for(int x=0;x<width;++x) {
            const size_t idx=static_cast<size_t>(y)*width+x;
            const gemini::Vec3 outputRay{
                ((static_cast<double>(x)+.5)-cx)/fx,
                (yo-cy)/fy,1.0};
            double sourceRow=yo/static_cast<double>(height);
            double outX=0,outY=0,den=1;
            for(int iter=0;iter<3;++iter){
                const double rf=std::clamp(sourceRow,0.,1.)*19.;
                const int i0=std::min(18,static_cast<int>(rf));
                const double u=rf-i0;
                const auto row=gemini::Quat::slerp(qs[i0],qs[i0+1],u);
                const auto qrel=(row.inverse()*qRef).normalized();
                const auto p=qrel.apply(outputRay);
                den=p.z;
                const double safe=std::max(den,1e-6);
                outX=fx*p.x/safe+cx-.5;
                outY=fy*p.y/safe+cy-.5;
                sourceRow=std::clamp((outY+.5)/height,0.,1.);
            }
            map_x[idx]=static_cast<float>(outX);
            map_y[idx]=static_cast<float>(outY);
            valid[idx]=(den>1e-4 &&
                outX>=.5 && outY>=.5 &&
                outX<width-1.5 && outY<height-1.5)
                ? std::uint8_t{255}:std::uint8_t{0};
        }
    }
    return 0;
}
