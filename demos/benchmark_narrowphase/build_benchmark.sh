#!/bin/bash
set -e

mkdir -p bench_narrow_bin

echo "=== Building Narrow Phase Benchmark 1 ==="
STEPS=200
SPHERES=100000
CAPSULES=100000
BOXES=100000
CELL_SIZE=5

echo "Building bench1..."

cmake -S . -B build_bench_narrow/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
    -DNARROW_BENCHMARK_STEPS=$STEPS \
    -DNARROW_BENCHMARK_SPHERE_COUNT=$SPHERES \
    -DNARROW_BENCHMARK_CAPSULE_COUNT=$CAPSULES \
    -DNARROW_BENCHMARK_BOX_COUNT=$BOXES

cmake --build build_bench_narrow/ --config Release --parallel

cp build_bench_narrow/bin/benchmark_narrowphase bench_narrow_bin/bench1


echo "=== Building Narrow Phase Benchmark 2 ==="
TOTAL_BENCH2_OBJECTS=300000
BENCH2_CAPSULES=0

for BOX_PERCENT in 0 25 50 75 100; do
    BENCH2_BOXES=$(( TOTAL_BENCH2_OBJECTS * BOX_PERCENT / 100 ))
    BENCH2_SPHERES=$(( TOTAL_BENCH2_OBJECTS - BENCH2_BOXES ))

    echo "Building bench2_${BOX_PERCENT}_boxes (Spheres: $BENCH2_SPHERES, Boxes: $BENCH2_BOXES)..."

    cmake -S . -B build_bench_narrow/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
        -DNARROW_BENCHMARK_STEPS=$STEPS \
        -DNARROW_BENCHMARK_SPHERE_COUNT=$BENCH2_SPHERES \
        -DNARROW_BENCHMARK_CAPSULE_COUNT=$BENCH2_CAPSULES \
        -DNARROW_BENCHMARK_BOX_COUNT=$BENCH2_BOXES

    cmake --build build_bench_narrow/ --config Release --parallel

    cp build_bench_narrow/bin/benchmark_narrowphase "bench_narrow_bin/bench2_${BOX_PERCENT}_boxes"
done

echo "All benchmark binaries have been built and saved in the 'bench_narrow_bin/' directory."
