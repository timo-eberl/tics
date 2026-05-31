#!/bin/bash
set -e # Exit immediately if a command fails

# Create a directory to store all the compiled benchmark binaries
mkdir -p bench_bin

echo "=== Building Benchmark 1 (Scaling Particles) ==="
STEPS_1=2000
CELL_SIZE_1="1.0"

# Required minimum dimensions: 65x65x115
RES_X=65
RES_Y=65
RES_Z=115

# Origin string expressions evaluated by the C compiler
ORG_X="(${RES_X}.0 * ${CELL_SIZE_1} * -0.5f)"
ORG_Y="(${RES_Y}.0 * ${CELL_SIZE_1} * -0.5f)"
ORG_Z="(${RES_Z}.0 * ${CELL_SIZE_1} * -0.5f)"

for PARTICLES in 2500 5000 7500 10000 12500 15000 17500 20000 22500 25000 27500 30000 32500 35000 37500 40000; do
    echo "Configuring and building Benchmark 1 with $PARTICLES particles..."
    
    cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release -DTICS_ENABLE_CUDA=ON -DTICS_ENABLE_DEBUG_VIEW=OFF -DCMAKE_C_COMPILER=gcc \
        -DBENCHMARK_STEPS=$STEPS_1 \
        -DBENCHMARK_PARTICLE_COUNT=$PARTICLES \
        -DGRID_RES_X=$RES_X -DGRID_RES_Y=$RES_Y -DGRID_RES_Z=$RES_Z \
        -DGRID_CELL_SIZE=$CELL_SIZE_1 \
        -DGRID_ORIGIN_X="$ORG_X" -DGRID_ORIGIN_Y="$ORG_Y" -DGRID_ORIGIN_Z="$ORG_Z"

    cmake --build build_release/ --config Release --parallel
    
    # Copy and rename the binary so it isn't overwritten
    cp build_release/bin/benchmark_broadphase bench_bin/bench1_particles_${PARTICLES}
done

echo "All benchmark binaries have been built and saved in the 'bench_bin/' directory."
