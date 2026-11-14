#include "tui/terminal_guard.h"
#include <cstdio>
// Leave the alternate screen, show the cursor, and clear attributes.
std::string terminal_restore_sequence(){return "\x1b[?1049l\x1b[?25h\x1b[0m";}
TerminalGuard::~TerminalGuard(){auto s=terminal_restore_sequence();std::fwrite(s.data(),1,s.size(),stderr);std::fflush(stderr);}
