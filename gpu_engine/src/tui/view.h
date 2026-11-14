#pragma once
#include "app/telemetry.h"
#include "tui/theme.h"
struct UiState { enum class Tab{Dashboard,Help} tab=Tab::Dashboard; bool help_open=false; int selected_particle=-1; int field_width=80; int field_height=24; };
ftxui::Element render_dashboard(const Telemetry&,const UiState&,const Theme&);
ftxui::Element render_help(const Theme&);
