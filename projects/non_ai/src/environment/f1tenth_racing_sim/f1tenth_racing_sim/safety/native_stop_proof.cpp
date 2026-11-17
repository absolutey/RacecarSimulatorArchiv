// Candidate native stop rollout. NEVER a safety authority until exact parity
// and deployment qualification; the Python StopEnvelope remains the default.
// Copyright (c) F1TENTH research prototype. C++17, no ROS dependencies.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <limits>
#include <memory>
#include <vector>

namespace {
constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double TAU = 2.0 * PI;
constexpr int COUNT = 36;
constexpr int SX=0,SY=1,SYAW=2,SSPEED=3,SVX=4,SVY=5,SYAW_RATE=6,
    SACCEL=7,SAX=8,SAY=9,SSLIP=10,SREQUEST=11,SJERK=12,SSTEER=13,SSTEER_RATE=14;
inline double clamp(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
inline double wrap(double x) {double a=std::fmod(x+PI,TAU); if(a<0.0)a+=TAU;return a-PI;}
inline double mix(double x){double t=clamp(x,0.,1.);return t*t*(3.-2.*t);}
inline bool finite(double x){return std::isfinite(x);}
inline std::int64_t clock_ns(){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return std::int64_t(t.tv_sec)*1000000000LL+t.tv_nsec;}
using State=std::array<double,15>;
struct Pose {double x,y,yaw;};
struct V2 {double x,y;};
struct Engine {
  std::array<double,COUNT> c;
  int width,height,threshold; bool flip;
  double resolution, origin_x,origin_y,cos0,sin0;
  std::vector<uint8_t> cells;
  std::vector<uint32_t> integral;
  double fxmin,fxmax,fymin,fymax,radius;
  Engine(const double* params,const uint8_t* pix,int w,int h):width(w),height(h){
    std::copy(params,params+COUNT,c.begin());
    resolution=c[30];origin_x=c[31];origin_y=c[32];cos0=std::cos(c[33]);sin0=std::sin(c[33]);
    threshold=(int)c[34];flip=c[35]!=0.0;
    cells.assign(pix,pix+size_t(w)*h);
    integral.assign(size_t(w+1)*(h+1),0);
    for(int y=0;y<h;++y){uint32_t row=0;for(int x=0;x<w;++x){
      row+=(cells[size_t(y)*w+x]<threshold);
      integral[size_t(y+1)*(w+1)+x+1]=integral[size_t(y)*(w+1)+x+1]+row;
    }}
    fxmin=c[26]-c[23];fxmax=c[27]+c[23];fymin=c[28]-c[23];fymax=c[29]+c[23];
    radius=std::max({std::hypot(fxmin,fymin),std::hypot(fxmin,fymax),
                     std::hypot(fxmax,fymin),std::hypot(fxmax,fymax)});
  }
  V2 pixel(double x,double y)const {
    double dx=x-origin_x,dy=y-origin_y;
    double gx=(cos0*dx+sin0*dy)/resolution;
    double gy=(-sin0*dx+cos0*dy)/resolution;
    return {gx,flip?height-gy:gy};
  }
  bool rectangle_free(int x0,int y0,int x1,int y1)const {
    if(x0<0||y0<0||x1>=width||y1>=height||x0>x1||y0>y1)return false;
    size_t stride=width+1;
    std::int64_t ct=std::int64_t(integral[size_t(y1+1)*stride+x1+1])
                  -integral[size_t(y0)*stride+x1+1]
                  -integral[size_t(y1+1)*stride+x0]
                  +integral[size_t(y0)*stride+x0];
    return ct==0;
  }
  bool sat(const std::array<V2,4>&poly,int col,int row)const {
    std::array<V2,4> cell={V2{double(col),double(row)},V2{double(col+1),double(row)},
       V2{double(col+1),double(row+1)},V2{double(col),double(row+1)}};
    std::array<V2,4> axes{V2{1.,0.},V2{0.,1.},V2{},V2{}};
    for(int e=0;e<2;++e){double dx=poly[e].x-poly[e+1].x,dy=poly[e].y-poly[e+1].y;axes[e+2]={-dy,dx};}
    for(const auto&axis:axes){
      double pmin=INFINITY,pmax=-INFINITY,qmin=INFINITY,qmax=-INFINITY;
      for(int i=0;i<4;++i){double a=axis.x*poly[i].x+axis.y*poly[i].y;
        double b=axis.x*cell[i].x+axis.y*cell[i].y;
        pmin=std::min(pmin,a);pmax=std::max(pmax,a);qmin=std::min(qmin,b);qmax=std::max(qmax,b);
      }
      if(pmax<qmin-1e-9||qmax<pmin-1e-9)return false;
    }
    return true;
  }
  bool footprint_collision(const Pose& pose,double pad)const {
    double cs=std::cos(pose.yaw), sn=std::sin(pose.yaw);
    double xmin=fxmin-pad,xmax=fxmax+pad,ymin=fymin-pad,ymax=fymax+pad;
    std::array<V2,4> corners{{{xmin,ymin},{xmax,ymin},{xmax,ymax},{xmin,ymax}}};
    double lowx=INFINITY,highx=-INFINITY,lowy=INFINITY,highy=-INFINITY;
    for(auto& p:corners){
      double gx=pose.x+cs*p.x-sn*p.y,gy=pose.y+sn*p.x+cs*p.y;
      p=pixel(gx,gy);
      if(!finite(p.x)||!finite(p.y))return true;
      lowx=std::min(lowx,p.x);highx=std::max(highx,p.x);
      lowy=std::min(lowy,p.y);highy=std::max(highy,p.y);
    }
    if(lowx<0.||lowy<0.||highx>=width||highy>=height)return true;
    int x0=(int)std::floor(lowx),x1=(int)std::floor(highx),y0=(int)std::floor(lowy),y1=(int)std::floor(highy);
    if(rectangle_free(x0,y0,x1,y1))return false;
    for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x){
      if(cells[size_t(y)*width+x]<threshold && sat(corners,x,y))return true;
    }
    return false;
  }
  bool swept(const Pose& previous,const Pose& current)const {
    double dx=current.x-previous.x,dy=current.y-previous.y;
    double delta=wrap(current.yaw-previous.yaw);
    double motion=std::hypot(dx,dy)+radius*std::abs(delta);
    // An adversarially large finite yaw/displacement must never overflow
    // the integer subdivision counter or authorize a partial sweep.
    double segments=std::ceil(motion/(0.5*resolution));
    if(!finite(segments)||segments>100000.)return true;
    int n=std::max(1,int(segments));
    double pad=motion/(2*n)+1e-9;
    V2 p0=pixel(previous.x,previous.y),p1=pixel(current.x,current.y);
    double r=(radius+pad)/resolution;
    // Pixel conversion can overflow for finite but physically impossible
    // world coordinates; reject before any integer cast or memory lookup.
    if(!finite(p0.x)||!finite(p0.y)||!finite(p1.x)||!finite(p1.y)||
       !finite(r)||std::abs(p0.x)>1.0e8||std::abs(p1.x)>1.0e8||
       std::abs(p0.y)>1.0e8||std::abs(p1.y)>1.0e8||r>1.0e8)return true;
    int x0=(int)std::floor(std::min(p0.x,p1.x)-r-1e-8);
    int x1=(int)std::ceil(std::max(p0.x,p1.x)+r+1e-8);
    int y0=(int)std::floor(std::min(p0.y,p1.y)-r-1e-8);
    int y1=(int)std::ceil(std::max(p0.y,p1.y)+r+1e-8);
    if(rectangle_free(x0,y0,x1,y1))return false;
    for(int k=0;k<=n;++k){double t=double(k)/n;
      Pose pose{previous.x+t*dx,previous.y+t*dy,previous.yaw+t*delta};
      if(footprint_collision(pose,pad))return true;
    }
    return false;
  }
  double lateral(double b,double cc,double d,double angle)const {
    return d*std::sin(cc*std::atan(b*angle));
  }
  int step(State &s,double req_steer,double req_accel,double dt)const {
    double old=s[SREQUEST];
    double steer_delta=clamp(req_steer-s[SSTEER],-c[11]*dt,c[11]*dt);
    s[SSTEER_RATE]=steer_delta/dt;
    s[SSTEER]=clamp(s[SSTEER]+steer_delta,-c[10],c[10]);
    double accel_delta=clamp(req_accel-old,-c[15]*dt,c[15]*dt);
    s[SJERK]=accel_delta/dt;
    s[SREQUEST]=clamp(old+accel_delta,-c[14],c[13]);
    s[SACCEL]=s[SREQUEST];
    double v=s[SSPEED];
    if(std::abs(v)<1e-8)s[SYAW_RATE]=0.;
    double lr=c[3], lf=c[2], wheelbase=lr+lf;
    double beta_target=std::atan2(lr*std::tan(s[SSTEER]),wheelbase);
    beta_target*=mix(std::abs(v)/c[16]);
    double yaw_target=v*std::cos(beta_target)*std::tan(s[SSTEER])/wheelbase;
    double low_beta_dot=wrap(beta_target-s[SSLIP])/c[18];
    double low_yaw_dot=(yaw_target-s[SYAW_RATE])/c[18];
    double alpha=mix((std::abs(v)-c[16])/(c[17]-c[16]));
    double tire_beta_dot=0.,tire_yaw_dot=0.;
    if(alpha>0.){
      double front_slip=-std::atan2(s[SVY]+lf*s[SYAW_RATE],s[SVX])+s[SSTEER];
      double rear_slip=-std::atan2(s[SVY]-lr*s[SYAW_RATE],s[SVX]);
      double f=lateral(c[4],c[5],c[6],front_slip), r=lateral(c[7],c[8],c[9],rear_slip);
      tire_beta_dot=(f+r)/(c[0]*v)-s[SYAW_RATE];
      tire_yaw_dot=(lf*f*std::cos(s[SSTEER])-lr*r)/c[1];
    }
    double slip_dot=(1.-alpha)*low_beta_dot+alpha*tire_beta_dot;
    double yaw_dot=(1.-alpha)*low_yaw_dot+alpha*tire_yaw_dot;
    if(!finite(slip_dot)||!finite(yaw_dot))return 5;
    s[SX]+=s[SSPEED]*std::cos(s[SYAW]+s[SSLIP])*dt;
    s[SY]+=s[SSPEED]*std::sin(s[SYAW]+s[SSLIP])*dt;
    s[SYAW]=wrap(s[SYAW]+s[SYAW_RATE]*dt);
    s[SSLIP]=wrap(s[SSLIP]+slip_dot*dt);
    s[SSPEED]=clamp(s[SSPEED]+s[SACCEL]*dt,-c[12],c[12]);
    s[SVX]=s[SSPEED]*std::cos(s[SSLIP]);
    s[SVY]=s[SSPEED]*std::sin(s[SSLIP]);
    s[SYAW_RATE]+=yaw_dot*dt;
    s[SAX]=s[SACCEL]*std::cos(s[SSLIP])-s[SSPEED]*s[SYAW_RATE]*std::sin(s[SSLIP]);
    s[SAY]=s[SACCEL]*std::sin(s[SSLIP])+s[SSPEED]*s[SYAW_RATE]*std::cos(s[SSLIP]);
    for(double x:s)if(!finite(x))return 6;
    return 0;
  }
  // Result raw fields: [qualified,reason_code,steer,time,dist,maxslip,maxlat,steps].
  void run(State z,double steer,double accel,std::int64_t deadline,double* out)const {
    double originx=z[SX],originy=z[SY];
    double max_slip=std::abs(z[SSLIP]),max_lat=std::abs(z[SSPEED]*z[SYAW_RATE]);
    double elapsed=0.;int steps=0;
    auto done=[&](int qualified,int reason){
      out[0]=qualified;out[1]=reason;out[2]=steer;out[3]=elapsed;
      out[4]=std::hypot(z[SX]-originx,z[SY]-originy);
      out[5]=max_slip;out[6]=max_lat;out[7]=steps;
    };
    if(deadline>0 && clock_ns()>=deadline){done(0,1);return;}
    for(double v:z){if(!finite(v)){done(0,2);return;}}
    if(!finite(steer)||!finite(accel)){done(0,3);return;}
    if(std::abs(z[SSPEED])>c[12]){done(0,4);return;}
    int react=int(std::ceil(c[22]/c[19])), total=int(std::ceil(c[20]/c[19]));
    for(int k=0;k<total;++k){
      if(deadline>0 && ((k&3)==0) && clock_ns()>=deadline){done(0,1);return;}
      Pose before{z[SX],z[SY],z[SYAW]};double oldspeed=z[SSPEED];
      double commanded;
      if(k<react)commanded=accel;
      else commanded=(z[SSPEED]>c[21]*c[19])?-c[21]:
           ((z[SSPEED]<-c[21]*c[19])?c[21]:-z[SSPEED]/c[19]);
      int step_error=step(z,steer,commanded,c[19]);
      if(step_error){done(0,step_error);return;}
      elapsed+=c[19];++steps;
      max_slip=std::max(max_slip,std::abs(z[SSLIP]));
      max_lat=std::max(max_lat,std::abs(z[SSPEED]*z[SYAW_RATE]));
      if(std::abs(z[SSLIP])>c[24] || std::abs(z[SSPEED]*z[SYAW_RATE])>c[25]){done(0,7);return;}
      if(swept(before,Pose{z[SX],z[SY],z[SYAW]})){done(0,8);return;}
      if(k>=react && std::abs(z[SSPEED])<=0.025 && std::abs(z[SYAW_RATE])<=0.1 &&
         std::abs(z[SREQUEST])<=0.2){done(1,0);return;}
      if(oldspeed>0.025 && z[SSPEED]<-0.025){done(0,9);return;}
      if(oldspeed<-0.025 && z[SSPEED]>0.025){done(0,9);return;}
    }
    done(0,10);
  }
};
}
extern "C" {
void* f1_create_native_stop(const double* cfg,const unsigned char* pix,int w,int h){
  if(!cfg||!pix||w<=0||h<=0)return nullptr;
  for(int i=0;i<COUNT;++i)if(!std::isfinite(cfg[i]))return nullptr;
  if(cfg[30]<=0||cfg[19]<=0||cfg[20]<=0||cfg[21]<=0||cfg[17]<=cfg[16]||cfg[18]<=0)return nullptr;
  try {return new Engine(cfg,pix,w,h);}catch(...){return nullptr;}
}
void f1_destroy_native_stop(void* ptr){delete static_cast<Engine*>(ptr);}
// Two complete independent full-stop rollouts, from the observed state and
// the predicted next simulator state. A bounded deadline covers BOTH.
// A false first proof never authorizes or substitutes for the second.
// Output: [both_certified, current_certified, next_certified, ran_next,
//          current_steps, next_steps].
int f1_evaluate_native_dualtick(void* ptr,const double* observed,double steer,
    double accel,double applied_steer,double applied_accel,int64_t deadline,
    double* out){
  if(!ptr||!observed||!out||!std::isfinite(steer)||!std::isfinite(accel)||
     !std::isfinite(applied_steer)||!std::isfinite(applied_accel))return -1;
  std::fill(out,out+6,0.0);
  try{
    Engine*engine=static_cast<Engine*>(ptr);
    State current;std::copy(observed,observed+15,current.begin());
    for(double v:current)if(!std::isfinite(v))return -2;
    State next=current;
    if(engine->step(next,applied_steer,applied_accel,.025)!=0)return -3;
    if(engine->step(current,steer,accel,.025)!=0)return -4;
    double proof[8]{};engine->run(current,steer,accel,deadline,proof);
    out[1]=proof[0];out[4]=proof[7];
    if(proof[0]!=1.0|| (deadline>0&&clock_ns()>=deadline))return 0;
    if(engine->step(next,steer,accel,.025)!=0)return -5;
    out[3]=1.0;
    double second[8]{};engine->run(next,steer,accel,deadline,second);
    out[2]=second[0];out[5]=second[7];
    out[0]=(proof[0]==1.0&&second[0]==1.0&&
            (deadline<=0||clock_ns()<deadline))?1.0:0.0;
    return 0;
  }catch(...){return -6;}
}

// Produce the SAME commanded 25ms physics step as the Python Guard before
// the full stop envelope. Never mutate caller state on model failure.
int f1_step_native(void* ptr,double* state,double steer,double accel,double dt){
  if(!ptr||!state||!std::isfinite(steer)||!std::isfinite(accel)||
     !std::isfinite(dt)||dt<=0.0||dt>0.05)return -1;
  try{
    State z;std::copy(state,state+15,z.begin());
    for(double v:z)if(!std::isfinite(v))return -2;
    int error=static_cast<Engine*>(ptr)->step(z,steer,accel,dt);
    if(error)return error;
    std::copy(z.begin(),z.end(),state);return 0;
  }catch(...){return -3;}
}
int f1_evaluate_native_stop(void* ptr,const double* state,double steer,double accel,int64_t deadline,double*out){
  if(!ptr||!state||!out)return -1;
  try{State z;std::copy(state,state+15,z.begin());static_cast<Engine*>(ptr)->run(z,steer,accel,deadline,out);return 0;}
  catch(...){return -2;}
}
}
