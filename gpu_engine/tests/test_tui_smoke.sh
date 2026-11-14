#!/usr/bin/env bash
set -euo pipefail
bin="$1"
command -v script >/dev/null 2>&1 || exit 77
output=$(TERM=xterm-256color script -qec "$bin tui --particles 1000 --bodies 10 --frames 2 --no-color --ascii" /dev/null)
printf '%s' "$output" | grep -q $'\x1b\[?25h'
