#include "app/probe.h"
#include <cassert>
int main(){std::vector<float4> p={{0,0,0,1},{.1f,0,0,1},{.2f,0,0,1}},v={{1,0,0,0},{2,0,0,0},{3,0,0,0}};ProbeResult r;assert(probe_nearest(p,v,{{0,0,-10},{0,0,1}},100,&r,.25f));assert(r.index==0&&r.neighbors==2);}
