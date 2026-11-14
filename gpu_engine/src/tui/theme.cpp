#include "tui/theme.h"
#include <cstdlib>
#include <string>
ftxui::Decorator Theme::strong() const { return color ? ftxui::color(accent)|ftxui::bold : ftxui::bold; }
ftxui::Decorator Theme::muted() const { return color ? ftxui::color(dim)|ftxui::dim : ftxui::dim; }
ftxui::Decorator Theme::status(bool good) const { return color ? ftxui::color(good?ok:error)|ftxui::bold : ftxui::bold; }
// Follow the terminal palette; no RGB values are constructed here.
Theme detect_theme(bool no_color, bool ascii) {
  Theme t; const char* term=std::getenv("TERM"); const char* nc=std::getenv("NO_COLOR");
  const char* lang=std::getenv("LC_ALL"); if(!lang) lang=std::getenv("LANG");
  t.color=!no_color && !nc && (!term || std::string(term)!="dumb");
  t.unicode=!ascii && lang && (std::string(lang).find("UTF-8")!=std::string::npos ||
                                std::string(lang).find("utf8")!=std::string::npos);
  return t;
}
void apply_terminal_quirks(const Theme&) {}
