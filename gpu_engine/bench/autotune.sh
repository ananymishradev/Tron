#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$root/bench"
output="$root/bench/results.csv"
printf 'block_size,particles_per_thread,cell_size_factor,use_smem,scene,ms_per_step\n' > "$output"

for block in 128 256 512; do
  for particles in 1 2 4; do
    for factor in 2 3 4; do
      for smem in 0 1; do
        printf '%d,%d,%d,%d,sparse,not-measured\n' "$block" "$particles" \
          "$factor" "$smem" >> "$output"
      done
    done
  done
done
echo "wrote $output"
