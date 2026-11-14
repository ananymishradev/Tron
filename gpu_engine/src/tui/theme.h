#pragma once
#include <ftxui/dom/elements.hpp>
// Semantic colors, not a fixed palette; each field can fall back to the terminal default.
struct Theme {
  bool color = true; bool unicode = true;
  ftxui::Color text = ftxui::Color::Default, dim = ftxui::Color::Default,
                accent = ftxui::Color::Default, ok = ftxui::Color::Green,
                warn = ftxui::Color::Yellow, error = ftxui::Color::Red;
  ftxui::Decorator strong() const; ftxui::Decorator muted() const;
  ftxui::Decorator status(bool good) const;
};
Theme detect_theme(bool no_color, bool ascii);
void apply_terminal_quirks(const Theme&);
