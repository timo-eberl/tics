#!/bin/bash
set -e

export OMP_WAIT_POLICY=PASSIVE
mkdir -p results

echo ""
echo "=== Running Benchmark 2 (Scaling Cell Size) ==="

for CELL in 1 5 10 15 20; do
    for STRATEGY in A B_NAIVE B_HALF_SHELL; do
        BIN_PATH="./bench_bin/bench2_cell_${CELL}_${STRATEGY}"
        OUT_FILE="results/bench2_cell_${CELL}_${STRATEGY}.txt"

        echo "Testing Cell Size ${CELL}.0 (Strategy: $STRATEGY)..."

        # Dry run
        $BIN_PATH > /dev/null 2>&1
        # Measured run (save stderr to file)
        $BIN_PATH 2> "$OUT_FILE" > /dev/null

        echo "  -> Saved to $OUT_FILE"
    done
done

echo "All benchmarks completed successfully! Check the 'results/' folder for your profiling data."
