# Tics Physics & Playground

## Development Build

```bash
cmake -S . -B build/
cmake --build build/

# Run demos
./build/demos/playground/playground
./build/demos/raycasting/raycasting
```

## Release Build

```bash
cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release
cmake --build build_release/ --config Release

# Run demos
./build_release/demos/playground/playground
./build_release/demos/raycasting/raycasting
```

## Library-Only Build

To build the `tics` static library without the demos (and graphics dependencies):

```bash
cmake -S . -B build_lib/ -DTICS_BUILD_DEMOS=OFF
cmake --build build_lib/
```

## Creating a Shippable Package

To distribute a demo, the executable requires the `assets/` folder to be located in the same directory:

```text
/dist
├── playground      # Executable
└── assets/         # Directory containing models
```

## To-Do

- [ ] Remove TICS_GA (Rip)
- [ ] Port tics to C
  - [ ] Replace Terathon math with `raymath.h` (from raylib)
  - [ ] Replace std::vector with fixed arrays and `stb_ds.h`
  - [ ] Get rid of smart pointers. Tics should only take pointers as arguments that are owned by the caller.
  - [ ] Replace inheritance with tagged unions
- [ ] Testing
- [ ] Big Fixes
- [ ] Multi-Threading
