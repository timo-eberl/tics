#!/bin/bash
set -e

# Create a directory for the output profiling text files
mkdir -p results

echo "=== Running Benchmark 1 (Scaling Particles) ==="
for PARTICLES in 2500 5000 7500 10000 12500 15000 17500 20000 22500 25000 27500 30000 32500 35000 37500 40000; do
    BIN_PATH="./bench_bin/bench1_particles_${PARTICLES}"
    OUT_FILE="results/bench1_particles_${PARTICLES}.txt"

    echo "Testing $PARTICLES particles..."

    # Dry run
    $BIN_PATH > /dev/null 2>&1

    # Measured run (save stderr to file)
    $BIN_PATH 2> "$OUT_FILE" > /dev/null
    
    echo "  -> Saved to $OUT_FILE"
done

echo "All benchmarks completed successfully! Check the 'results/' folder for your profiling data."
