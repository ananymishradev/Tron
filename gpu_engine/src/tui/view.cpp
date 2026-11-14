#include "tui/view.h"
#include "tui/field.h"
#include "tui/format.h"
#include <string>
using namespace ftxui;
static Element lv(const char* l,std::string v,const Theme&t){return hbox({text(l)|t.muted(),text(" "),text(v)|t.strong()});}
// Pure view: no CUDA, no runner calls, so it is cheap to unit test.
Element render_dashboard(const Telemetry&t,const UiState&s,const Theme&th){
 // Below this size the panels overlap; show one clear message instead.
 if(s.field_width<80||s.field_height<24)return text("Terminal too small: "+std::to_string(s.field_width)+"x"+std::to_string(s.field_height)+" (need 80x24)")|center;
 std::string st=t.paused?(t.stepping?"STEP":"PAUSED"):"RUNNING";
 auto head=hbox({text("TRON")|th.strong(),filler(),text(st)|th.status(!t.paused),text("  PARTICLES ")|th.muted(),text(format_count(t.particles))|th.strong(),text("  FRAME "),text(std::to_string(t.frame))});
 auto perf=vbox({text("PERFORMANCE")|th.strong(),lv("FPS",format_ms(t.fps),th),lv("SIM",format_ms(t.sim_ms),th),lv("FRAME",format_ms(t.frame_ms),th),lv("P50",format_ms(t.sim_history.percentile(.5)),th)})|border;
 auto phys=vbox({text("PHYSICS")|th.strong(),lv("KE",format_energy(t.stats.kinetic),th),lv("PE",format_energy(t.stats.potential),th),lv("PAIRS",format_count(t.contacts.pairTests),th),lv("CONTACTS",format_count(t.contacts.contacts),th),lv("GRID",format_count(t.grid_cells),th),lv("DEVICE",format_bytes(t.device_used_bytes),th),text(t.stats.firstNonFinite>=0?"NaN @ "+std::to_string(t.stats.firstNonFinite):"STATE OK")|th.status(t.stats.firstNonFinite<0)})|border;
 auto field=vbox({text("FIELD · "+std::string(t.plane==ViewPlane::TopXZ?"TOP XZ":"SIDE XY"))|th.strong(),render_field(t.density,t.probe,t.probe_valid,th,s.field_width-35,s.field_height-10),text("TELEMETRY")|th.strong(),text("sim "+format_ms(t.sim_ms)+"  frame "+format_ms(t.frame_ms))|th.muted()})|border|flex;
 return vbox({head,separator(),hbox({field,vbox({perf,phys})}),separator(),text("Space pause · . step · R reset · P probe · L log · V view · ? help · Q quit")|th.muted()})|border;
}
Element render_help(const Theme&th){return vbox({text("TRON HELP")|th.strong(),separator(),text("Space  pause or resume the simulation"),text(".      advance one simulation step"),text("R      reset with the current seed"),text("P      probe the current field"),text("L      toggle CSV logging"),text("V      toggle top/side view"),text("?      toggle this help"),text("Q/Esc  quit")})|border|center;}
