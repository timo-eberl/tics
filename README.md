# Physics Playground

## Build and run

```
cmake -S . -B build/
cmake --build build/
# Run one of the demos
./build/demos/playground/playground
./build/demos/raycasting/raycasting
```

## Release build

```
cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release
cmake --build build_release/ --config Release
# Run one of the demos
./build_release/demos/playground/playground
./build_release/demos/raycasting/raycasting
```
