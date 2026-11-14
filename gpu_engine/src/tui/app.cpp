#include "tui/app.h"
#include "app/runner.h"
#include "tui/terminal_guard.h"
#include "tui/theme.h"
#include "tui/view.h"
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <atomic>
#include <chrono>
#include <thread>
// The UI thread never touches CUDA; everything goes through the runner.
int TuiApp::run(const Options&o){
 TerminalGuard guard; Theme th=detect_theme(o.no_color,o.ascii);apply_terminal_quirks(th);
 RunnerConfig c{ o.scene_config,o.stats_hz,o.log_path }; SimulationRunner r;std::string e;if(!r.start(c,&e))return 1;
 UiState s;Telemetry t;auto screen=ftxui::ScreenInteractive::Fullscreen();auto quit=screen.ExitLoopClosure();
 // Poll the latest snapshot on each repaint; a missed frame is harmless.
 auto renderer=ftxui::Renderer([&]{r.poll(&t);s.field_width=100;s.field_height=30;if(o.frames>0&&t.frame>=static_cast<uint64_t>(o.frames))quit();return s.help_open?render_help(th):render_dashboard(t,s,th);});
 auto component=ftxui::CatchEvent(renderer,[&](ftxui::Event ev){
  if(ev==ftxui::Event::Character(" ")){r.set_paused(!t.paused);return true;}
  if(ev==ftxui::Event::Character(".")){r.step_once();return true;}
  if(ev==ftxui::Event::Character("r")||ev==ftxui::Event::Character("R")){r.reset(o.seed);return true;}
  if(ev==ftxui::Event::Character("l")||ev==ftxui::Event::Character("L")){r.set_logging(!t.logging);return true;}
  if(ev==ftxui::Event::Character("v")||ev==ftxui::Event::Character("V")){r.request_view(t.plane==ViewPlane::TopXZ?ViewPlane::SideXY:ViewPlane::TopXZ);return true;}
  if(ev==ftxui::Event::Character("?")){s.help_open=!s.help_open;return true;}
  if(ev==ftxui::Event::Escape||ev==ftxui::Event::Character("q")||ev==ftxui::Event::Character("Q")||ev==ftxui::Event::CtrlC){quit();return true;} return false;});
 // A 30 Hz ticker keeps the dashboard fresh even while paused.
 std::atomic<bool> live{true};std::thread ticker([&]{while(live){std::this_thread::sleep_for(std::chrono::milliseconds(33));screen.PostEvent(ftxui::Event::Custom);}});
 screen.Loop(component);live=false;ticker.join();if(!o.dump_path.empty())r.request_dump(o.dump_path);r.stop();return 0;
}
