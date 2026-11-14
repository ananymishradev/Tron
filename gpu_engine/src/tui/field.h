#pragma once
#include "app/telemetry.h"
#include "tui/theme.h"
bool field_pick(const DensityGrid& density, int cell_x, int cell_y, Ray* out);
ftxui::Element render_field(const DensityGrid&,const ProbeResult&,bool,const Theme&,int,int);
