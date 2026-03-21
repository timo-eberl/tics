#!/bin/bash
set -e

# Create a directory for the output profiling text files
mkdir -p results

echo ""
echo "=== Running Benchmark 2 (Scaling Grid Resolution) ==="
for RES in 100 80 60 40 20; do
    BIN_PATH="./bench_bin/bench2_res_${RES}"
    OUT_FILE="results/bench2_res_${RES}.txt"

    echo "Testing Grid Resolution ${RES}x${RES}x${RES}..."

    # Dry run
    $BIN_PATH > /dev/null 2>&1

    # Measured run (save stderr to file)
    $BIN_PATH 2> "$OUT_FILE" > /dev/null

    echo "  -> Saved to $OUT_FILE"
done

echo "All benchmarks completed successfully! Check the 'results/' folder for your profiling data."
