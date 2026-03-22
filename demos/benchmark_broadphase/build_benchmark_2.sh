#!/bin/bash
set -e

mkdir -p bench_bin

echo "=== Building Benchmark 2 (Scaling Cell Size) ==="
STEPS_2=120
PARTICLES_2=100000

# First cells, then strategy to reduce build time
for CELL in 1 5 10 15 20; do
    for STRATEGY in A B_NAIVE B_HALF_SHELL; do
        
        # Determine Grid Resolution to cover at least 100m
        if [ "$CELL" -eq 1 ]; then RES=100
        elif [ "$CELL" -eq 5 ]; then RES=20
        elif [ "$CELL" -eq 10 ]; then RES=10
        elif [ "$CELL" -eq 15 ]; then RES=7   # 7 * 15 = 105m
        elif [ "$CELL" -eq 20 ]; then RES=5
        fi

        echo "Configuring Strategy: $STRATEGY | Cell Size: $CELL.0 | Resolution: $RES..."

        cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
            -DBENCHMARK_STEPS=$STEPS_2 \
            -DBENCHMARK_PARTICLE_COUNT=$PARTICLES_2 \
            -DGRID_RES_X=$RES -DGRID_RES_Y=$RES -DGRID_RES_Z=$RES \
            -DGRID_CELL_SIZE=${CELL}.0 \
            -DGPU_BROADPHASE_STRATEGY=$STRATEGY

        cmake --build build_release/ --config Release
        
        # Copy and rename the binary with cell size in the file name
        cp build_release/bin/benchmark_broadphase bench_bin/bench2_cell_${CELL}_${STRATEGY}
    done
done

echo "All benchmark binaries have been built and saved in the 'bench_bin/' directory."
