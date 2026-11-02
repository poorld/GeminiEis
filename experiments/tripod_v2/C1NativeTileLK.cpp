#include "C1NativeTileLK.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <vector>

namespace {
constexpr int W=320,H=180,TILES=9,MAX_TILE=18;
constexpr double PI=3.14159265358979323846;
struct P { double x=0,y=0; };
struct Candidate {P p; double quality=0;int tile=0;};
struct Track {P p,q; int tile=0;double fb=0,err=0;bool inlier=false;};
struct Image {
    int w=0,h=0;
    std::vector<uint8_t> a;
};
inline uint8_t at(const Image& im,int x,int y) {
    return im.a[static_cast<size_t>(y)*im.w+x];
}
Image half(const Image& src) {
    Image d;d.w=src.w/2;d.h=src.h/2;
    d.a.resize(static_cast<size_t>(d.w)*d.h);
    for(int y=0;y<d.h;++y)for(int x=0;x<d.w;++x){
        const int xx=x*2,yy=y*2;
        d.a[static_cast<size_t>(y)*d.w+x]=static_cast<uint8_t>(
          (int(at(src,xx,yy))+int(at(src,xx+1,yy))+
           int(at(src,xx,yy+1))+int(at(src,xx+1,yy+1))+2)/4);
    } return d;
}
bool sample(const Image& im,double x,double y,double& val) {
    if(x<1||y<1||x>=im.w-2||y>=im.h-2)return false;
    int ix=static_cast<int>(x),iy=static_cast<int>(y);
    const double dx=x-ix,dy=y-iy;
    const double a=at(im,ix,iy)*(1-dx)+at(im,ix+1,iy)*dx;
    const double b=at(im,ix,iy+1)*(1-dx)+at(im,ix+1,iy+1)*dx;
    val=a*(1-dy)+b*dy;
    return true;
}
double weight(int tile) {
    return tile==4?2.: (tile==1||tile==3||tile==5||tile==7)?1.30:.95;
}
std::vector<Candidate> corners(const Image& prev,const uint8_t* mask,int stride) {
    std::vector<Candidate> all;
    // Harris/Shi-Tomasi smallest second-moment eigenvalue within 5x5.
    // Find independently in 3x3 regions; center is preferred, not exclusive.
    for(int tile=0;tile<9;++tile) {
        int row=tile/3,col=tile%3;
        int x0=col*W/3+7,x1=(col+1)*W/3-7;
        int y0=row*H/3+7,y1=(row+1)*H/3-7;
        std::vector<Candidate> v;
        v.reserve(1300);
        double maxq=0.;
        for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x) {
            if(mask[static_cast<size_t>(y)*stride+x]==0)continue;
            double a=0,b=0,c=0;
            for(int dy=-2;dy<=2;dy+=2)for(int dx=-2;dx<=2;dx+=2){
                int sx=x+dx,sy=y+dy;
                const double gx=(int(at(prev,sx+1,sy))-int(at(prev,sx-1,sy)))*.5;
                const double gy=(int(at(prev,sx,sy+1))-int(at(prev,sx,sy-1)))*.5;
                a+=gx*gx;b+=gx*gy;c+=gy*gy;
            }
            const double eig=(a+c-std::sqrt((a-c)*(a-c)+4*b*b))*.5;
            if(eig>1.) { v.push_back({{double(x),double(y)},eig,tile});
                         maxq=std::max(maxq,eig); }
        }
        // Bound the sort while retaining the strongest spatial NMS candidates.
        const auto score_cmp=[](const Candidate& a,const Candidate& b){
            if(a.quality!=b.quality)return a.quality>b.quality;
            if(a.p.y!=b.p.y)return a.p.y<b.p.y;
            return a.p.x<b.p.x;
        };
        if(v.size()>480){
            std::nth_element(v.begin(),v.begin()+480,v.end(),score_cmp);
            v.resize(480);
        }
        std::sort(v.begin(),v.end(),[](const Candidate& a,const Candidate& b){
            if(a.quality!=b.quality)return a.quality>b.quality;
            if(a.p.y!=b.p.y)return a.p.y<b.p.y;
            return a.p.x<b.p.x;
        });
        int selected=0;
        const double threshold=maxq*.011;
        for(const Candidate& c:v) {
            if(selected>=MAX_TILE||c.quality<threshold)break;
            bool nearby=false;
            for(const Candidate& accepted:all)if(accepted.tile==tile) {
                double dx=accepted.p.x-c.p.x,dy=accepted.p.y-c.p.y;
                if(dx*dx+dy*dy<36.){nearby=true;break;}
            }
            if(!nearby){all.push_back(c);++selected;}
        }
    }
    return all;
}
bool trackPoint(const std::array<Image,3>& prev,const std::array<Image,3>& curr,
                const P& source,P& dst) {
    P q{};
    for(int lvl=2;lvl>=0;--lvl) {
        const Image& a=prev[static_cast<size_t>(lvl)];
        const Image& b=curr[static_cast<size_t>(lvl)];
        const double s=1.0/(1<<lvl);
        P p{source.x*s,source.y*s};
        if(lvl==2)q=p;
        else {q.x*=2;q.y*=2;}
        const int radius=3;
        // 16 iterations solve inverse compositional additive 2D LK.
        // Hessian uses gradients of current frame; *no future observations*.
        for(int iter=0;iter<9;++iter) {
            double Hxx=0,Hxy=0,Hyy=0,bx=0,by=0;
            for(int dy=-radius;dy<=radius;++dy)for(int dx=-radius;dx<=radius;++dx) {
                double x0,y0,gxp,gxm,gyp,gym;
                if(!sample(a,p.x+dx,p.y+dy,x0)||
                   !sample(b,q.x+dx,q.y+dy,y0)||
                   !sample(b,q.x+dx+1,q.y+dy,gxp)||
                   !sample(b,q.x+dx-1,q.y+dy,gxm)||
                   !sample(b,q.x+dx,q.y+dy+1,gyp)||
                   !sample(b,q.x+dx,q.y+dy-1,gym))return false;
                const double gx=(gxp-gxm)*.5,gy=(gyp-gym)*.5;
                const double delta=x0-y0;
                Hxx+=gx*gx;Hxy+=gx*gy;Hyy+=gy*gy;
                bx+=gx*delta;by+=gy*delta;
            }
            double det=Hxx*Hyy-Hxy*Hxy;
            if(det<8000.)return false;
            double ux=(bx*Hyy-by*Hxy)/det;
            double uy=(by*Hxx-bx*Hxy)/det;
            ux=std::clamp(ux,-2.,2.);
            uy=std::clamp(uy,-2.,2.);
            q.x+=ux;q.y+=uy;
            if(ux*ux+uy*uy<.00009)break;
        }
        if(q.x<4||q.y<4||q.x>=b.w-4||q.y>=b.h-4)return false;
    }
    dst=q;return true;
}
bool solve4(double m[4][5],double out[4]) {
    for(int k=0;k<4;++k) {
        int best=k;
        for(int row=k+1;row<4;++row)
            if(std::abs(m[row][k])>std::abs(m[best][k]))best=row;
        if(std::abs(m[best][k])<1e-8)return false;
        if(best!=k)for(int j=k;j<5;++j)std::swap(m[best][j],m[k][j]);
        double a=m[k][k];
        for(int j=k;j<5;++j)m[k][j]/=a;
        for(int i=0;i<4;++i)if(i!=k){
            double scale=m[i][k];
            for(int j=k;j<5;++j)m[i][j]-=scale*m[k][j];
        }
    }
    for(int i=0;i<4;++i)out[i]=m[i][4];
    return true;
}
struct Sim {double a=1,b=0,x=0,y=0;};
bool fitPair(const Track& l,const Track& r,Sim& out) {
    const double ux=r.p.x-l.p.x,uy=r.p.y-l.p.y;
    const double vx=r.q.x-l.q.x,vy=r.q.y-l.q.y;
    const double d=ux*ux+uy*uy;
    if(d<16)return false;
    out.a=(ux*vx+uy*vy)/d;
    out.b=(ux*vy-uy*vx)/d;
    out.x=l.q.x-out.a*l.p.x+out.b*l.p.y;
    out.y=l.q.y-out.b*l.p.x-out.a*l.p.y;
    return true;
}
double error(const Track& tr,const Sim& s) {
    const double ex=s.a*tr.p.x-s.b*tr.p.y+s.x-tr.q.x;
    const double ey=s.b*tr.p.x+s.a*tr.p.y+s.y-tr.q.y;
    return std::sqrt(ex*ex+ey*ey);
}
bool weightedRefit(const std::vector<Track>& tr,const std::vector<uint8_t>& inliers,
                   Sim& result) {
    double m[4][5]{};
    int n=0;
    for(size_t i=0;i<tr.size();++i)if(inliers[i]){
        const auto& t=tr[i];double x=t.p.x,y=t.p.y;
        const double rw=weight(t.tile);
        double rows[2][5]={{x,-y,1.,0.,t.q.x},
                            {y, x,0.,1.,t.q.y}};
        for(auto& row:rows){
            for(int a=0;a<4;++a){
                for(int b=0;b<4;++b)m[a][b]+=rw*row[a]*row[b];
                m[a][4]+=rw*row[a]*row[4];
            }
        }
        ++n;
    }
    if(n<4)return false;
    double out[4]{};
    if(!solve4(m,out))return false;
    result={out[0],out[1],out[2],out[3]};
    return true;
}
double pct(std::vector<double>& v,double q) {
    if(v.empty())return 0;
    std::sort(v.begin(),v.end());
    double k=(v.size()-1)*q;
    int a=static_cast<int>(k),b=std::min(int(v.size())-1,a+1);
    return v[static_cast<size_t>(a)]*(b-k)+v[static_cast<size_t>(b)]*(k-a);
}
struct RNG {
    uint32_t x=0x91273847u;
    uint32_t next() {x^=x<<13;x^=x>>17;x^=x<<5;return x;}
};
} // anonymous

int c1b_visual_pair(const uint8_t* prev_gray,const uint8_t* curr_gray,
                    const uint8_t* prev_valid,const uint8_t* curr_valid,
                    int width,int height,int stride,C1BVisual* out) {
    using Clock=std::chrono::steady_clock;
    const auto t0=Clock::now();
    if(!out)return -1;
    *out=C1BVisual{};
    out->status=C1B_INVALID_ARGUMENT;
    out->matrix[0]=out->matrix[4]=1.;
    for(int i=0;i<9;++i){
        out->tile_dx[i]=out->tile_dy[i]=out->tile_fit[i]=
            std::numeric_limits<double>::quiet_NaN();
    }
    if(!prev_gray||!curr_gray||!prev_valid||!curr_valid||
       width!=W||height!=H||stride<width)return -1;
    const auto finish=[&](int status) {
        out->status=status;
        out->elapsed_ms=std::chrono::duration<double,std::milli>(
            Clock::now()-t0).count();
        return status==C1B_INVALID_ARGUMENT?-1:0;
    };
    int gooda=0,goodb=0;
    Image old{W,H,std::vector<uint8_t>(static_cast<size_t>(W)*H)};
    Image now{W,H,std::vector<uint8_t>(static_cast<size_t>(W)*H)};
    for(int y=0;y<H;++y)for(int x=0;x<W;++x){
        size_t k=static_cast<size_t>(y)*stride+x;
        size_t r=static_cast<size_t>(y)*W+x;
        old.a[r]=prev_gray[k];now.a[r]=curr_gray[k];
        gooda+=prev_valid[k]>0;goodb+=curr_valid[k]>0;
    }
    if(gooda<W*H*.55||goodb<W*H*.55)
        return finish(C1B_COVERAGE_LOW);
    const auto selected=corners(old,prev_valid,stride);
    out->checked=static_cast<int>(selected.size());
    if(selected.empty())return finish(C1B_NO_FEATURES);
    const std::array<Image,3> a{old,half(old),half(half(old))};
    const std::array<Image,3> b{now,half(now),half(half(now))};
    std::vector<Track> tracks;tracks.reserve(selected.size());
    for(const Candidate& c:selected){
        P q{},back{};
        if(!trackPoint(a,b,c.p,q)||!trackPoint(b,a,q,back))continue;
        double dx=q.x-c.p.x,dy=q.y-c.p.y;
        double fb=std::hypot(back.x-c.p.x,back.y-c.p.y);
        if(fb>=.85||dx*dx+dy*dy>=900.)continue;
        int xi=std::clamp(int(std::round(q.x)),0,W-1);
        int yi=std::clamp(int(std::round(q.y)),0,H-1);
        if(curr_valid[static_cast<size_t>(yi)*stride+xi]==0)continue;
        tracks.push_back({c.p,q,c.tile,fb,0.,false});
        out->tile_checked[c.tile]++;
    }
    out->tracked=static_cast<int>(tracks.size());
    if(tracks.size()<25)return finish(C1B_TOO_FEW);
    RNG rng;
    std::vector<uint8_t> best(tracks.size()),sel(tracks.size());
    int bestN=-1;double bestErr=1e100;
    Sim bestSim{};
    constexpr int kIters=180;
    for(int iter=0;iter<kIters;++iter){
        int i=int(rng.next()%tracks.size()),j=int(rng.next()%tracks.size());
        if(i==j)j=(j+1)%int(tracks.size());
        Sim test{};
        if(!fitPair(tracks[size_t(i)],tracks[size_t(j)],test))continue;
        int n=0;double err=0;
        for(size_t k=0;k<tracks.size();++k){
            double e=error(tracks[k],test);
            sel[k]=e<=1.5;
            if(sel[k]){++n;err+=e;}
        }
        if(n>bestN||(n==bestN&&err<bestErr)){
            bestN=n;bestErr=err;best=sel;bestSim=test;
        }
    }
    if(bestN<18)return finish(C1B_NO_MODEL);
    if(!weightedRefit(tracks,best,bestSim))return finish(C1B_NO_MODEL);
    std::array<int,9> support{};
    std::vector<double> fb,errorValues;
    fb.reserve(tracks.size());errorValues.reserve(tracks.size());
    int n=0;
    for(auto& tr:tracks){
        tr.err=error(tr,bestSim);
        tr.inlier=(tr.err<=1.5);
        if(tr.inlier){
            ++n;++support[tr.tile];
            fb.push_back(tr.fb);errorValues.push_back(tr.err);
            out->tile_inliers[tr.tile]++;
        }
    }
    out->inliers=n;
    for(int z=0;z<9;++z){
        if(support[z])++out->support_tiles;
    }
    for(int col=0;col<3;++col)
        if(support[col]||support[3+col]||support[6+col])++out->support_cols;
    if(out->support_cols<3||out->support_tiles<4)
        return finish(C1B_SPATIAL_FAIL);
    for(int z=0;z<9;++z){
        std::vector<double> dx,dy,er;
        for(const auto& tr:tracks)if(tr.tile==z){
            dx.push_back(tr.q.x-tr.p.x);
            dy.push_back(tr.q.y-tr.p.y);
            er.push_back(tr.err);
        }
        if(!dx.empty()){
            out->tile_dx[z]=pct(dx,.5);
            out->tile_dy[z]=pct(dy,.5);
            out->tile_fit[z]=pct(er,.5);
        }
    }
    out->fb_p90=pct(fb,.90);
    out->fit_p90=pct(errorValues,.90);
    double scale=std::hypot(bestSim.a,bestSim.b);
    out->logscale=std::log(std::max(scale,1e-10));
    out->rotation_rad=std::atan2(bestSim.b,bestSim.a);
    out->center_dx=(bestSim.a-1.)*W*.5-bestSim.b*H*.5+bestSim.x;
    out->center_dy=bestSim.b*W*.5+(bestSim.a-1.)*H*.5+bestSim.y;
    out->matrix[0]=bestSim.a;out->matrix[1]=-bestSim.b;out->matrix[2]=bestSim.x;
    out->matrix[3]=bestSim.b;out->matrix[4]= bestSim.a;out->matrix[5]=bestSim.y;
    double fraction=double(n)/tracks.size();
    out->trust=std::clamp(std::min(1.,n/65.)*fraction*
         std::min(1.,out->support_tiles/6.)*std::exp(-.7*out->fit_p90),0.,1.);
    if(out->fb_p90>1.1)return finish(C1B_FB_UNRELIABLE);
    if(out->fit_p90>2.)return finish(C1B_NONRIGID);
    if(out->trust<.14)return finish(C1B_LOW_TRUST);
    if(std::abs(out->logscale)>.045||std::abs(out->rotation_rad)>.09)
        return finish(C1B_MOTION_OUTLIER);
    if(std::hypot(out->center_dx,out->center_dy)>22)
        return finish(C1B_MOTION_OUTLIER);
    return finish(C1B_ACCEPT);
}
