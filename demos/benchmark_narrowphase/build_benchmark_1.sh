#!/bin/bash
set -e

mkdir -p bench_narrow_bin

echo "=== Building Narrow Phase Benchmark 1 ==="
STEPS=200
SPHERES=100000
CAPSULES=100000
BOXES=100000
CELL_SIZE=5

echo "Building..."

cmake -S . -B build_bench_narrow/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
    -DNARROW_BENCHMARK_STEPS=$STEPS \
    -DNARROW_BENCHMARK_SPHERE_COUNT=$SPHERES \
    -DNARROW_BENCHMARK_CAPSULE_COUNT=$CAPSULES \
    -DNARROW_BENCHMARK_BOX_COUNT=$BOXES \

cmake --build build_bench_narrow/ --config Release --parallel

cp build_bench_narrow/bin/benchmark_narrowphase bench_narrow_bin/bench1

echo "Benchmark binary has been built and saved in the 'bench_narrow_bin/' directory."
