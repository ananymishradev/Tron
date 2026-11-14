#include "tui/field.h"
#include <algorithm>
#include <cmath>
ftxui::Element render_field(const DensityGrid& d,const ProbeResult&,bool probe,const Theme& t,int w,int h){
  // Use the observed maximum so a quiet scene still shows contrast.
  uint32_t max=d.max_count; for(auto v:d.particles) max=std::max(max,v);
  std::string shades=" .:-=+*#%@"; std::vector<ftxui::Element> rows;
  for(int y=0;y<std::max(1,h);++y){std::string row;for(int x=0;x<std::max(1,w);++x){
    int sx=d.width?x*d.width/std::max(1,w):0, sy=d.height?y*d.height/std::max(1,h):0;
    size_t i=(size_t)(sy*std::max(1,d.width)+sx); uint32_t v=i<d.particles.size()?d.particles[i]:0;
    // Log scaling matches the PPM export and keeps dense clusters readable.
    double n=max?std::log1p(v)/std::log1p(max):0; row+=shades[std::clamp((int)(n*9),0,9)];
  } rows.push_back(ftxui::text(row));}
  if(probe&&!rows.empty()) rows.front()=rows.front()|t.strong();
  return ftxui::vbox(std::move(rows))|ftxui::border;
}
