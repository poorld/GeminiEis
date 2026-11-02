#include "C1NativeRsPrewarp.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include "common/types.hpp"

// FAST C1 19RS / 20x33 actual geometry, matching Gemini's final mesh structure.
// Interpolation is the same diagonal-triangle bilinear partition as the
// original PC renderer; use this only for feature prewarp (320x180), not
// a second final full-resolution output interpolation.
namespace {
constexpr int kR=20,kC=33;
struct Coord {double x=0,y=0,z=0;};
gemini::Quat qread(const double* p) noexcept {
    return gemini::Quat{p[0],p[1],p[2],p[3]}.normalized();
}
}
int gemini_v2c_rs19_prewarp_map(int width,int height,
    double fx,double fy,double cx,double cy,
    const double* q_rows20_xyzw,const double* q_ref_xyzw,
    float* map_x,float* map_y,std::uint8_t* valid) {
    if(width<=0||height<=0||width>4096||height>4096||
       !(std::isfinite(fx)&&std::isfinite(fy)&&fx>0&&fy>0)||
       !q_rows20_xyzw||!q_ref_xyzw||!map_x||!map_y||!valid)
        return -1;
    std::array<gemini::Quat,kR> qs{};
    for(int i=0;i<kR;++i) {
        const double* p=q_rows20_xyzw+4*i;
        for(int j=0;j<4;++j)if(!std::isfinite(p[j]))return -2;
        qs[static_cast<size_t>(i)]=qread(p);
    }
    for(int j=0;j<4;++j)if(!std::isfinite(q_ref_xyzw[j]))return -2;
    const auto qRef=qread(q_ref_xyzw);
    std::array<Coord,kR*kC> vertex{};
    for(int row=0;row<kR;++row) {
        const double y=static_cast<double>(row)/(kR-1)*height;
        for(int col=0;col<kC;++col) {
            const double x=static_cast<double>(col)/(kC-1)*width;
            const gemini::Vec3 ray{(x-cx)/fx,(y-cy)/fy,1.};
            double fraction=static_cast<double>(row)/(kR-1);
            double sampleX=0,sampleY=0,den=0;
            for(int it=0;it<3;++it) {
                double rf=std::clamp(fraction,0.,1.)*19.;
                const int i0=std::min(18,static_cast<int>(rf));
                const auto qRow=gemini::Quat::slerp(
                    qs[static_cast<size_t>(i0)],
                    qs[static_cast<size_t>(i0+1)],rf-i0);
                const auto relative=(qRow.inverse()*qRef).normalized();
                const auto p=relative.apply(ray);
                den=p.z;
                double safe=std::max(den,1e-6);
                sampleX=fx*p.x/safe+cx-.5;
                sampleY=fy*p.y/safe+cy-.5;
                fraction=std::clamp((sampleY+.5)/height,0.,1.);
            }
            vertex[static_cast<size_t>(row*kC+col)]={sampleX,sampleY,den};
        }
    }
    for(int y=0;y<height;++y){
        const double cellY=(static_cast<double>(y)+.5)/height*19.;
        const int iy=std::min(18,static_cast<int>(cellY));
        const double sy=cellY-iy;
        for(int x=0;x<width;++x){
            const double cellX=(static_cast<double>(x)+.5)/width*32.;
            const int ix=std::min(31,static_cast<int>(cellX));
            const double sx=cellX-ix;
            const int tl=iy*kC+ix;
            const auto& A=vertex[static_cast<size_t>(tl)];
            const auto& B=vertex[static_cast<size_t>(tl+1)];
            const auto& C=vertex[static_cast<size_t>(tl+kC)];
            const auto& D=vertex[static_cast<size_t>(tl+kC+1)];
            double mx,my,den;
            if(sx+sy<=1.) {
                const double a=1.-sx-sy,b=sy,c=sx;
                mx=a*A.x+b*C.x+c*B.x;
                my=a*A.y+b*C.y+c*B.y;
                den=a*A.z+b*C.z+c*B.z;
            } else {
                const double a=1.-sy,b=1.-sx,c=sx+sy-1.;
                mx=a*B.x+b*C.x+c*D.x;
                my=a*B.y+b*C.y+c*D.y;
                den=a*B.z+b*C.z+c*D.z;
            }
            const auto k=static_cast<size_t>(y)*width+x;
            map_x[k]=static_cast<float>(mx);
            map_y[k]=static_cast<float>(my);
            valid[k]=(den>1e-4 &&
                mx>=.5&&my>=.5&&mx<width-1.5&&my<height-1.5)
                    ? std::uint8_t{255}:std::uint8_t{0};
        }
    }
    return 0;
}
