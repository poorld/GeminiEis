#include "C1B2NativeYuv.h"
#include "C1NativeRsPrewarp.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {
constexpr int kSourceW=2560,kSourceH=1440,kW=320,kH=180,kArea=8;
inline std::uint8_t lerp8(const uint8_t* src,int stride,float x,float y) {
    // OpenCV remap INTER_LINEAR uses quantized fractions at 1/32 precision.
    // Use 5-bit fraction to closely preserve the previous C1-A/B image.
    const int ix=static_cast<int>(std::floor(x));
    const int iy=static_cast<int>(std::floor(y));
    const int fx=std::clamp(static_cast<int>(std::round((x-ix)*32.)),0,32);
    const int fy=std::clamp(static_cast<int>(std::round((y-iy)*32.)),0,32);
    const int a=32-fx,b=32-fy;
    const int iy0=iy*stride;
    const int top=a*int(src[iy0+ix])+fx*int(src[iy0+ix+1]);
    const int bot=a*int(src[iy0+stride+ix])+fx*int(src[iy0+stride+ix+1]);
    return static_cast<uint8_t>(std::clamp((top*b+bot*fy+512)/1024,0,255));
}
}
int gemini_c1b2_luma_area8(const uint8_t* src,int width,int height,
    int row_stride,int pixel_stride,uint8_t* dst,int out_stride) {
    if(!src||!dst||width!=kSourceW||height!=kSourceH||
       pixel_stride<1||pixel_stride>4||
       row_stride<=0||row_stride>32768||
       row_stride<((width-1)*pixel_stride+1)||
       out_stride<kW||out_stride>4096)return -1;
    for(int y=0;y<kH;++y) {
        std::uint8_t* dest=dst+static_cast<size_t>(y)*out_stride;
        for(int x=0;x<kW;++x) {
            int sum=0;
            const int xo=x*kArea*pixel_stride,yo=y*kArea;
            for(int dy=0;dy<kArea;++dy) {
                const uint8_t* row=src+
                    static_cast<size_t>(yo+dy)*row_stride+xo;
                for(int dx=0;dx<kArea;++dx)
                    sum+=row[dx*pixel_stride];
            }
            dest[x]=static_cast<uint8_t>((sum+32)>>6);
        }
    }
    return 0;
}
int gemini_c1b2_prewarp_gray(const uint8_t* src,int stride,
    double fx,double fy,double cx,double cy,
    const double* rows,const double* reference,
    uint8_t* dst,uint8_t* valid) {
    if(!src||stride<kW||stride>4096||!rows||!reference||!dst||!valid ||
       !std::isfinite(cx)||!std::isfinite(cy))return -1;
    std::array<float,kW*kH> mapX{},mapY{};
    const int ret=gemini_v2c_rs19_prewarp_map(
        kW,kH,fx,fy,cx,cy,rows,reference,
        mapX.data(),mapY.data(),valid);
    if(ret)return ret;
    for(size_t i=0;i<mapX.size();++i) {
        if(!valid[i]) {
            dst[i]=0;
            continue;
        }
        dst[i]=lerp8(src,stride,mapX[i],mapY[i]);
    }
    return 0;
}
