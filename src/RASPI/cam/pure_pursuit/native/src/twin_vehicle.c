#include <math.h>
#include "fox_native.h"
/* State and parameters are doubles, with contraction disabled by native/build.py.
 * Parameter layout (constructed by native_vehicle.py):
 * 0..10: servo max/min/deadband/slew/backlash/lag/center/gain R/L/wheel max R/L
 * 11..16: pwm0, brake lock speed, slide decel, brake decel, wheel tau, coast decel
 * 17..22: reverse curve enabled, reverse k/pwm0, forward k, max wheel, scrub
 * 23..29: coast/drive tau, reverse/forward acceleration, reverse slip enabled,
 *         reverse spin tau, reverse steering scale
 * 30..34: wheelbase, lateral acceleration L/R, right/left table lengths
 * 35..: interleaved servo units / wheel degrees, right table then left table.
 */
static double (*volatile py_sin)(double)=sin;
static double (*volatile py_cos)(double)=cos;
static double table(double u, const double *t, int n) {
 double u0=t[0],w0=t[1];
 for(int i=1;i<n;i++){double u1=t[2*i],w1=t[2*i+1];if(u<=u1)return w0+(w1-w0)*(u-u0)/(u1-u0);u0=u1;w0=w1;}return w0;
}
FOX_API void fox_twin_vehicle(double *s,const double *p){
 int rn=(int)p[33],ln=(int)p[34]; const double *rt=p+35,*lt=rt+2*rn;
 double dt=s[10],cmd=s[11],pwm=s[12];int dir=(int)s[13];
 /* s: x y heading v servo effective wheel_v sliding lag last_wheel */
 volatile double square_exponent = 2.0;
 double target=fmax(p[1],fmin(cmd,p[0])),servo=s[4],err=target-servo;
 if(fabs(err)>p[2]){double step=p[3]*dt;servo=err>0?fmin(servo+step,target):fmax(servo-step,target);s[4]=servo;}
 double eff=s[5],b=p[4];if(servo-eff>b){eff=servo-b;s[5]=eff;}else if(eff-servo>b){eff=servo+b;s[5]=eff;}
 if(p[5]>0)eff=s[8]+(eff-s[8])*fmin(1.0,dt/p[5]);s[8]=eff;
 double delta=p[6]-eff,wheel;
 if(delta>=0)wheel=fmin(rn?table(delta,rt,rn):delta*p[7]/90.0,p[9]);
 else wheel=ln?-fmin(table(-delta,lt,ln),p[10]):fmax(delta*p[8]/90.0,-p[10]);
 s[9]=wheel;double vel;
 if(dir!=0 && pwm<=p[11]){
  double v0=s[3];if(!s[7]&&fabs(v0)>p[12])s[7]=1;
  double dec=(s[7]?p[13]:p[14])*dt;
  vel=v0>dec?v0-dec:(v0< -dec?v0+dec:0);s[3]=vel;
  if(s[7]){s[6]-=s[6]*(dt/p[15]);if(fabs(vel)<1e-6){s[7]=0;s[6]=0;}}else s[6]=vel;
 }else if(dir==0 && p[16]){
  double dec=p[16]*dt,v0=s[3];vel=v0>dec?v0-dec:(v0< -dec?v0+dec:0);s[3]=vel;s[6]=vel;s[7]=0;
 }else{
  double vt=0;
  if(dir!=0){double v=(dir<0&&p[17])?p[18]*(pwm-p[19]):p[20]*(pwm-p[11]);
   if(p[21]>1e-6)v*=1.0-p[22]*pow(fabs(wheel)/p[21],square_exponent);
   double ss=fmax(0,v);vt=dir>0?ss:-ss;}
  double tau=dir==0?p[23]:p[24],dv=(vt-s[3])*(dt/tau);
  double amax=(dir<0&&p[25])?p[25]:p[26];int spinning=0;
  if(dir!=0&&amax){double lim=amax*dt;if(dv>lim){dv=lim;spinning=dir<0&&p[27];}else if(dv< -lim){dv=-lim;spinning=dir<0&&p[27];}}
  vel=s[3]+dv;s[3]=vel;
  if(spinning){s[6]+=(vt-s[6])*fmin(1.0,dt/p[28]);if(fabs(s[6])<fabs(vel))s[6]=vel;}else s[6]=vel;s[7]=0;
 }
 if(vel<0)wheel*=p[29];
 double wh=wheel*(3.14159265358979323846/180.0);
 if(fabs(wh)>1e-9&&fabs(p[30])>1e-6){double yaw=vel/p[30]*tan(wh),alat=yaw>0?p[31]:p[32];
  if(alat&&fabs(vel)>1e-3&&fabs(yaw*vel)>alat)yaw=copysign(alat/fabs(vel),yaw);
  s[2]+=(yaw*dt)*(180.0/3.14159265358979323846);}
 double h=s[2]*(3.14159265358979323846/180.0);s[0]+=vel*dt*py_sin(h);s[1]+=vel*dt*py_cos(h);
}
