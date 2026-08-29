#!/bin/bash
set -e

mkdir -p bench_narrow_bin

echo "=== Building Narrow Phase Benchmark 1 ==="
STEPS=200
PARTICLES=300000
CELL_SIZE=5

RES_X=62
RES_Y=17
RES_Z=29

# Origin string expressions evaluated by the C compiler
ORG_X="(${RES_X}.0 * ${CELL_SIZE} * -0.5f)"
ORG_Y="(${RES_Y}.0 * ${CELL_SIZE} * -0.5f)"
ORG_Z="(${RES_Z}.0 * ${CELL_SIZE} * -0.5f)"

for STRATEGY in A B_NAIVE B_HALF_SHELL; do
    echo "Configuring Strategy: $STRATEGY..."

    cmake -S . -B build_bench_narrow/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_CUDA=ON -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
        -DNARROW_BENCHMARK_STEPS=$STEPS \
        -DNARROW_BENCHMARK_PARTICLE_COUNT=$PARTICLES \
        -DGRID_RES_X=$RES_X -DGRID_RES_Y=$RES_Y -DGRID_RES_Z=$RES_Z \
        -DGRID_CELL_SIZE=${CELL_SIZE}.0 \
        -DGRID_ORIGIN_X="$ORG_X" -DGRID_ORIGIN_Y="$ORG_Y" -DGRID_ORIGIN_Z="$ORG_Z" \
        -DGPU_BROADPHASE_STRATEGY=$STRATEGY

    cmake --build build_bench_narrow/ --config Release --parallel

    cp build_bench_narrow/bin/benchmark_narrowphase bench_narrow_bin/bench1_${STRATEGY}
done

echo "All benchmark binaries have been built and saved in the 'bench_narrow_bin/' directory."
