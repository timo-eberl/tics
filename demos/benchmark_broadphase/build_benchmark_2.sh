#!/bin/bash
set -e

mkdir -p bench_bin

echo "=== Building Benchmark 2 (Scaling Cell Size) ==="
STEPS_2=120
PARTICLES_2=100000

# First cells, then strategy to reduce build time
for CELL in 1 5 10 15 20; do
    for STRATEGY in A B_NAIVE B_HALF_SHELL; do
        
        # Determine Grid Resolution to cover at least 65x65x115m using ceiling math: (A + B - 1) / B
        RES_X=$(( (65 + CELL - 1) / CELL ))
        RES_Y=$(( (65 + CELL - 1) / CELL ))
        RES_Z=$(( (115 + CELL - 1) / CELL ))

        # Origin string expressions evaluated by the C compiler
        ORG_X="(${RES_X}.0 * ${CELL}.0 * -0.5f)"
        ORG_Y="(${RES_Y}.0 * ${CELL}.0 * -0.5f)"
        ORG_Z="(${RES_Z}.0 * ${CELL}.0 * -0.5f)"

        echo "Configuring Strategy: $STRATEGY | Cell Size: $CELL.0 | Resolution: ${RES_X}x${RES_Y}x${RES_Z}..."

        cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_CUDA=ON -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
            -DBENCHMARK_STEPS=$STEPS_2 \
            -DBENCHMARK_PARTICLE_COUNT=$PARTICLES_2 \
            -DGRID_RES_X=$RES_X -DGRID_RES_Y=$RES_Y -DGRID_RES_Z=$RES_Z \
            -DGRID_CELL_SIZE=${CELL}.0 \
            -DGRID_ORIGIN_X="$ORG_X" -DGRID_ORIGIN_Y="$ORG_Y" -DGRID_ORIGIN_Z="$ORG_Z" \
            -DGPU_BROADPHASE_STRATEGY=$STRATEGY

        cmake --build build_release/ --config Release --parallel
        
        # Copy and rename the binary with cell size in the file name
        cp build_release/bin/benchmark_broadphase bench_bin/bench2_cell_${CELL}_${STRATEGY}
    done
done

echo "All benchmark binaries have been built and saved in the 'bench_bin/' directory."
