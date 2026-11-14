#include "tui/format.h"
#include <cmath>
#include <iomanip>
#include <sstream>
static std::string scaled(double v,double d,const char* s) {
  std::ostringstream o; double x=v/d; o<<std::fixed<<std::setprecision(std::abs(x)>=100?0:(std::abs(x)>=10?1:2))<<x<<s; return o.str();
}
std::string format_count(long long v){double a=std::abs((double)v);if(a>=1e9)return scaled(v,1e9,"B");if(a>=1e6)return scaled(v,1e6,"M");if(a>=1e3)return scaled(v,1e3,"k");return std::to_string(v);}
std::string format_ms(double v){std::ostringstream o;o<<std::fixed<<std::setprecision(1)<<v<<" ms";return o.str();}
std::string format_bytes(std::size_t v){return v%(1024*1024)==0?std::to_string(v/(1024*1024))+" MiB":scaled(v,1024*1024," MiB");}
std::string format_energy(float v){std::ostringstream o;o<<std::fixed<<std::setprecision(2)<<v;return o.str();}
std::string format_percent(double v){std::ostringstream o;o<<std::fixed<<std::setprecision(1)<<v<<"%";return o.str();}
