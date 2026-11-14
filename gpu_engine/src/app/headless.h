#pragma once

#include "cli/options.h"

#include <iosfwd>

int run_headless(const Options& options, std::ostream& out, std::ostream& err);
