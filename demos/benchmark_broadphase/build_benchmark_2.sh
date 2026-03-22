#!/bin/bash
set -e # Exit immediately if a command fails

# Create a directory to store all the compiled benchmark binaries
mkdir -p bench_bin

echo "=== Building Benchmark 2 (Scaling Grid Resolution) ==="
STEPS_2=120
PARTICLES_2=100000

for RES in 100 90 80 70 60 50 40 30 20 10; do
    for STRATEGY in A B_NAIVE B_HALF_SHELL; do
        # Map resolution to the corresponding cell size to maintain the 100m^3 scene bounds
        if [ "$RES" -eq 100 ]; then CELL_SIZE="1.0f"
        elif [ "$RES" -eq 90 ]; then CELL_SIZE="1.11f"
        elif [ "$RES" -eq 80 ]; then CELL_SIZE="1.25f"
        elif [ "$RES" -eq 70 ]; then CELL_SIZE="1.43f"
        elif [ "$RES" -eq 60 ]; then CELL_SIZE="1.67f"
        elif [ "$RES" -eq 50 ]; then CELL_SIZE="2.0f"
        elif [ "$RES" -eq 40 ]; then CELL_SIZE="2.5f"
        elif [ "$RES" -eq 30 ]; then CELL_SIZE="3.33f"
        elif [ "$RES" -eq 20 ]; then CELL_SIZE="5.0f"
        elif [ "$RES" -eq 10 ]; then CELL_SIZE="10.0f"
        fi

        echo "Configuring and building Benchmark 2 with Grid Resolution $RES (Cell Size: $CELL_SIZE) and Strategy: $STRATEGY..."

        cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
            -DBENCHMARK_STEPS=$STEPS_2 \
            -DBENCHMARK_PARTICLE_COUNT=$PARTICLES_2 \
            -DGRID_RES_X=$RES -DGRID_RES_Y=$RES -DGRID_RES_Z=$RES \
            -DGRID_CELL_SIZE=$CELL_SIZE \
            -DGPU_BROADPHASE_STRATEGY=$STRATEGY

        cmake --build build_release/ --config Release

        # Copy and rename the binary
        cp build_release/bin/benchmark_broadphase bench_bin/bench2_res_${RES}_${STRATEGY}
    done
done

echo "All benchmark binaries have been built and saved in the 'bench_bin/' directory."
