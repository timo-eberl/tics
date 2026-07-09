#!/bin/bash
set -e

# Detect Native Linux vs WSL/Windows/macOS
# WSL kernels always contain "microsoft" in their release string
if [ "$(uname -s)" = "Linux" ] && ! uname -r | grep -iq "microsoft"; then
    # We are on native Linux: set the passive policy prefix
    EXEC_PREFIX="env OMP_WAIT_POLICY=PASSIVE"
else
    EXEC_PREFIX=""
fi

# Create a directory for the output profiling text files
mkdir -p bench_broad_results

echo "=== Running Benchmark 1 (Scaling Particles) ==="
for PARTICLES in 2500 5000 7500 10000 12500 15000 17500 20000 22500 25000 27500 30000 32500 35000 37500 40000; do
    BIN_PATH="./bench_broad_bin/bench1_particles_${PARTICLES}"
    OUT_FILE="bench_broad_results/bench1_particles_${PARTICLES}.txt"

    echo "Testing $PARTICLES particles..."

    # Dry run
    "$BIN_PATH" > /dev/null 2>&1

    # Measured run (save stderr to file)
    $EXEC_PREFIX "$BIN_PATH" 2> "$OUT_FILE"

    echo "  -> Saved to $OUT_FILE"
done

echo "All benchmarks completed successfully! Check the 'bench_broad_results/' folder for your profiling data."
