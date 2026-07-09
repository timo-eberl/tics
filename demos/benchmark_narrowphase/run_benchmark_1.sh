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

mkdir -p bench_narrow_results

echo "=== Running Narrow Phase Benchmark 1 ==="
    for STRATEGY in A B_NAIVE B_HALF_SHELL; do
    BIN_PATH="./bench_narrow_bin/bench1_${STRATEGY}"
    OUT_FILE="bench_narrow_results/bench1_${STRATEGY}.txt"

    echo "Testing strategy $STRATEGY..."

    # Dry run
    "$BIN_PATH" > /dev/null 2>&1

    # Measured run (save stderr to file)
    $EXEC_PREFIX "$BIN_PATH" 2> "$OUT_FILE"

    echo "  -> Saved to $OUT_FILE"
done

echo "All benchmarks completed successfully! Check the 'bench_narrow_results/' folder for your profiling data."
