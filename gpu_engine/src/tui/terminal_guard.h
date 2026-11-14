#pragma once
#include <string>
std::string terminal_restore_sequence();
// RAII guard for the alternate screen; the destructor is the last line of defense.
class TerminalGuard{public:TerminalGuard()=default;~TerminalGuard();};
