#!/bin/bash
set -e # Exit immediately if a command fails

# Create a directory to store all the compiled benchmark binaries
mkdir -p bench_bin

echo "=== Building Benchmark 2 (Scaling Grid Resolution) ==="
STEPS_2=120
PARTICLES_2=100000

for RES in 100 80 60 40 20; do
    # Map resolution to the corresponding cell size to maintain the 100m^3 scene bounds
    if [ "$RES" -eq 100 ]; then CELL_SIZE="1.0"
    elif [ "$RES" -eq 80 ]; then CELL_SIZE="1.25"
    elif [ "$RES" -eq 60 ]; then CELL_SIZE="1.6666667"
    elif [ "$RES" -eq 40 ]; then CELL_SIZE="2.5"
    elif [ "$RES" -eq 20 ]; then CELL_SIZE="5.0"
    fi

    echo "Configuring and building Benchmark 2 with Grid Resolution $RES (Cell Size: $CELL_SIZE)..."

    cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
        -DBENCHMARK_STEPS=$STEPS_2 \
        -DBENCHMARK_PARTICLE_COUNT=$PARTICLES_2 \
        -DGRID_RES_X=$RES -DGRID_RES_Y=$RES -DGRID_RES_Z=$RES \
        -DGRID_CELL_SIZE=$CELL_SIZE

    cmake --build build_release/ --config Release
    
    # Copy and rename the binary
    cp build_release/bin/benchmark_broadphase bench_bin/bench2_res_${RES}
done

echo "All benchmark binaries have been built and saved in the 'bench_bin/' directory."
