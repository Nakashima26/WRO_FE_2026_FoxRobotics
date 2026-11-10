#include "fox_native.h"
#include <math.h>
/* Batched ray/segment intersection. Strict < keeps the first segment on ties. */
FOX_API void fox_twin_cast(int nr,int ns,const double *orig,const double *dir,const double *a,const double *b,const double *norm,const int *mat,const int *obj,double range,double *dist,int *material,double *normal,int *object){
 for(int r=0;r<nr;r++){
  double best=INFINITY;int chosen=-1;
  double vx=-dir[2*r+1],vy=dir[2*r];
  for(int j=0;j<ns;j++){
   double dx=b[2*j]-a[2*j],dy=b[2*j+1]-a[2*j+1];
   double den=dx*vx+dy*vy;
   if(fabs(den)<=1e-12)continue;
   double x=orig[2*r]-a[2*j],y=orig[2*r+1]-a[2*j+1];
   double t=(dx*y-dy*x)/den,u=(x*vx+y*vy)/den;
   if(t>=0&&t<=range&&u>=0&&u<=1&&t<best){best=t;chosen=j;}
  }
  dist[r]=chosen<0?range:best;material[r]=chosen<0?-1:mat[chosen];object[r]=chosen<0?-1:obj[chosen];
  normal[2*r]=chosen<0?0:norm[2*chosen];normal[2*r+1]=chosen<0?0:norm[2*chosen+1];
 }
}
