# benchmarks

## 21 rigid bodies, cubes and icospheres, 20 seconds

initial ga:
d: 2771ns, cd: 356866ns, cr: 10497ns

initial la:
d: 2377ns, cd: 332165ns, cr: 7996ns

with lto:
d: 2389ns, cd: 297313ns, cr: 8088ns

After "Refactor playground demo"
d: 1253ns, cd: 260944ns, cr: 7593ns

After Demo: Use shared colliders
d: 1219ns, cd: 261811ns, cr: 7121ns

After C wrapper implementation:
d: 1222ns, cd: 258354ns, cr: 7088ns

After porting dynamics:
d: 823ns, cd: 0ns, cr: 0ns

After porting collision detection:
d: 804ns, cd: 312279ns, cr: 0ns

After porting collision response:
d: 844ns, cd: 245491ns, cr: 2357ns

After fixing GJK (blick disabled):
d: 823ns, cd: 116467ns, cr: 1909ns

After Multi-Threading with GCC (bottlenecked by Overhead, worth it for more bodies):
d: 877ns, cd: 1851474ns, cr: 2582ns

After Multi-Threading with Clang (bottlenecked by Overhead, worth it for more bodies):
d: 981ns, cd: 101612ns, cr: 2232ns

After sorting collisions (no clue why it is that much faster):
d: 827ns, cd: 65188ns, cr: 1898ns

After changing step order:
d: 344ns, cd: 164275ns, cr: 2611ns

After quat nlerp -> slerp:
d: 1302ns, cd: 158376ns, cr: 2697ns

Brute-force broadphase:
d: 1026ns, cd: 41339ns, cr: 1359ns

After fixing collision response:
Apply Forces                  :   0.0004 ms (Avg over 1200)
Collision Detection           :   0.1091 ms (Avg over 1200)
  Proxy Collection            :   0.0038 ms (Avg over 1200)
  Broad Phase                 :   0.0032 ms (Avg over 1200)
  Narrow Phase                :   0.1018 ms (Avg over 1200)
Collision Response            :   0.0014 ms (Avg over 1200)
Apply Velocities              :   0.0010 ms (Avg over 1200)

## 1000 rigid bodies, cubes and icospheres, 5 seconds

Single-threaded:
d: 25964ns, cd: 81047312ns, cr: 56886ns

Multi-threaded GCC:
d: 32453ns, cd: 10227840ns, cr: 84472ns

Multi-threaded Clang:
d: 30474ns, cd: 8836745ns, cr: 72670ns

After changing step and nlerp -> slerp:
d: 30242ns, cd: 9009300ns, cr: 218585ns

Brute-force broadphase:
Apply Forces                  :   0.0340 ms (Avg over 300)
Collision Detection           :   3.2635 ms (Avg over 300)
  Proxy Collection            :   0.1015 ms (Avg over 300)
  Broad Phase                 :   2.0315 ms (Avg over 300)
  Narrow Phase                :   1.1303 ms (Avg over 300)
Collision Response            :   0.2327 ms (Avg over 300)
Apply Velocities              :   0.0328 ms (Avg over 300)

Warm up multi-threading on creation, Fixed solver bug, added 10 velocity solver iterations:
Apply Forces                  :   0.0050 ms (Avg over 300)
Collision Detection           :   2.6801 ms (Avg over 300)
  Proxy Collection            :   0.1123 ms (Avg over 300)
  Broad Phase                 :   1.8504 ms (Avg over 300)
  Narrow Phase                :   0.7171 ms (Avg over 300)
Collision Response            :   0.1658 ms (Avg over 300)
Apply Velocities              :   0.0234 ms (Avg over 300)

warm starting with 10 iterations:
Apply Forces                  :   0.0048 ms (Avg over 300)
Collision Detection           :   2.4608 ms (Avg over 300)
  Proxy Collection            :   0.1062 ms (Avg over 300)
  Broad Phase                 :   1.7399 ms (Avg over 300)
  Narrow Phase                :   0.6146 ms (Avg over 300)
Collision Response            :   0.3587 ms (Avg over 300)
Apply Velocities              :   0.0238 ms (Avg over 300)

without position solver, naive autovec broadphase:
Apply Forces                  :   0.0035 ms (Avg over 300)
Collision Detection           :   2.9490 ms (Avg over 300)
  Proxy Collection            :   0.0766 ms (Avg over 300)
  Proxy Collection SoA        :   0.0786 ms (Avg over 300)
  Broad Phase                 :   0.6707 ms (Avg over 300)
  Narrow Phase                :   2.1228 ms (Avg over 300)
Collision Response            :   0.9985 ms (Avg over 300)
Apply Velocities              :   0.0200 ms (Avg over 300)

## Popcorn Machine 10.000 Objects, aligned on Z, 300 Steps

gcc naive
  Broad Phase                 :  61.8635 ms (Avg over 300)
gcc naive parallel
  Broad Phase                 :  11.5136 ms (Avg over 300)
gcc naive simd
  Broad Phase                 :  13.5345 ms (Avg over 300)

clang naive
  Broad Phase                 :  81.6814 ms (Avg over 300)
clang naive parallel
  Broad Phase                 :  12.5578 ms (Avg over 300)
clang naive simd
  Broad Phase                 :  17.7252 ms (Avg over 300)

## Popcorn Machine 10.000 Objects, 30 Steps

gcc naive
  Broad Phase                 :  59.6405 ms (Avg over 30)
gcc naive parallel
  Broad Phase                 :   8.2750 ms (Avg over 30)
gcc naive simd
  Broad Phase                 :  12.7800 ms (Avg over 30)
gcc naive autovec
  Broad Phase                 :   9.9306 ms (Avg over 30)
gcc naive simd speculative
  Broad Phase                 :   9.7461 ms (Avg over 30)

clang naive
  Broad Phase                 :  79.5087 ms (Avg over 30)
clang naive parallel
  Broad Phase                 :   8.6322 ms (Avg over 30)
clang naive simd
  Broad Phase                 :  18.4865 ms (Avg over 30)
clang naive autovec
  Broad Phase                 :  21.2804 ms (Avg over 20)

## Popcorn Machine 10.000 Objects, 300 Steps (more steps because of fluctuating results)

gcc naive autovec parallel
  Broad Phase                 :   2.1748 ms (Avg over 300)
gcc naive simd speculative parallel
  Broad Phase                 :   2.3762 ms (Avg over 300)
gcc sap (300 steps)
  Broad Phase                 :   2.4772 ms (Avg over 300)
sap parallel insertion sort lookup
  Broad Phase                 :   1.1582 ms (Avg over 300)
cuda naive
  Broad Phase GPU             :   1.8076 ms (Avg over 300)

## Popcorn Machine 100.000 Objects, 30 Steps

naive autovec parallel
  Broad Phase                 : 243.9743 ms (Avg over 30)
sap
  Broad Phase                 :  90.2132 ms (Avg over 30)
sap simd
  Broad Phase                 :  68.4192 ms (Avg over 30)
sap parallel
  Broad Phase                 :  25.8310 ms (Avg over 30)
sap parallel simd
  Broad Phase                 :  25.2218 ms (Avg over 30)
sap parallel insertion sort
  Proxy Collection Typed      :   8.1845 ms (Avg over 30)
  Broad Phase                 :  27.0919 ms (Avg over 30)
    Sort                          :  12.7388 ms (Avg over 30)
    Sweep                         :  14.2971 ms (Avg over 30)
sap parallel insertion sort lookup
  Proxy Collection Typed      :   2.9593 ms (Avg over 30)
  Broad Phase                 :  28.0668 ms (Avg over 30)
    Sort                          :  13.4583 ms (Avg over 30)
    Sweep                         :  14.5666 ms (Avg over 30)
cuda grid_a
  Proxy Collection Typed      :   2.8786 ms (Avg over 30)
  Proxy Collection Packed     :   1.5537 ms (Avg over 30)
  Broad Phase                 :  25.9308 ms (Avg over 30)
  Broad Phase GPU             :   7.0725 ms (Avg over 30)

## Popcorn Machine virtual walls 100.000 Objects, 30 Steps

sap parallel insertion sort lookup
  Proxy Collection Typed      :   2.8272 ms (Avg over 30)
  Broad Phase                 :  27.0795 ms (Avg over 30)
    Sort                          :  13.3076 ms (Avg over 30)
    Sweep                         :  13.7366 ms (Avg over 30)
cuda naive heuristic (object limit removed)
  Proxy Collection Typed      :   2.8123 ms (Avg over 30)
  Proxy Collection Packed     :   1.5204 ms (Avg over 30)
  Broad Phase                 :  26.1622 ms (Avg over 30)
  Broad Phase GPU             :  39.6030 ms (Avg over 30)
  Narrow Phase                :   1.7599 ms (Avg over 30)
cuda grid_b 3x3x3
  Proxy Collection Typed      :   2.8049 ms (Avg over 30)
  Proxy Collection Packed     :   1.5162 ms (Avg over 30)
  Broad Phase                 :  26.7272 ms (Avg over 30)
  Broad Phase GPU             :   1.7859 ms (Avg over 30)
  [cuda] grid (avg over 30) upload=0.471ms assign=0.042ms sort=0.113ms bounds=0.077ms tests=0.909ms readback=0.062ms total=1.675ms
cuda grid_a
  Proxy Collection Typed      :   2.8755 ms (Avg over 30)
  Proxy Collection Packed     :   1.5275 ms (Avg over 30)
  Broad Phase                 :  26.3013 ms (Avg over 30)
  Broad Phase GPU             :   2.0888 ms (Avg over 30)
  [cuda] grid_a (avg over 30) upload=0.471ms 1a: count=0.042ms 1b: scan=0.048ms 1c: assign=0.081ms sort=0.432ms bounds=0.106ms tests=0.748ms readback=0.061ms total=1.988ms

## Popcorn Machine virtual walls 100.000 Objects, 5% Statics, 30 Steps

cuda grid_b 3x3x3 v1
  Proxy Collection Typed      :   2.9555 ms (Avg over 30)
  Proxy Collection Packed     :   1.5101 ms (Avg over 30)
  Broad Phase                 :  26.8016 ms (Avg over 30)
  Broad Phase GPU             :   1.8841 ms (Avg over 30)
  [cuda] grid_b (avg over 30) upload=0.465ms assign=0.115ms sort=0.125ms bounds=0.080ms tests=0.910ms readback=0.059ms total=1.753ms
cuda grid_b 3x3x3 sorted bodies
  Proxy Collection Typed      :   2.8863 ms (Avg over 30)
  Proxy Collection Packed     :   1.4783 ms (Avg over 30)
  Broad Phase                 :  25.9265 ms (Avg over 30)
  Broad Phase GPU             :   1.0601 ms (Avg over 30)
  [cuda] grid_b (avg over 30) pagelock=0.020ms upload=0.368ms assign=0.094ms sort=0.086ms permute=0.053ms bounds=0.080ms tests=0.197ms readback=0.060ms total=0.958ms
cuda grid_a
  Proxy Collection Typed      :   2.8933 ms (Avg over 30)
  Proxy Collection Packed     :   1.4934 ms (Avg over 30)
  Broad Phase                 :  26.9454 ms (Avg over 30)
  Broad Phase GPU             :   2.0088 ms (Avg over 30)
  [cuda] grid_a (avg over 30) pagelock=0.019ms upload=0.370ms 1a: count=0.095ms 1b: scan=0.027ms 1c: assign=0.074ms sort=0.428ms bounds=0.108ms tests=0.731ms readback=0.061ms total=1.911ms
cuda grid_a sorted bodies
  Proxy Collection Typed      :   2.7877 ms (Avg over 30)
  Proxy Collection Packed     :   1.4589 ms (Avg over 30)
  Broad Phase                 :  25.8619 ms (Avg over 30)
  Broad Phase GPU             :   1.4685 ms (Avg over 30)
  [cuda] grid_a (avg over 30) pagelock=0.021ms upload=0.368ms presort=0.208ms 1a-count=0.016ms 1b-scan=0.026ms 1c-assign=0.098ms sort=0.231ms bounds=0.098ms tests=0.242ms readback=0.064ms total=1.373ms

## Sparse Islands 60 islands (cuda naive limit), 60 Steps

sap parallel insertion sort lookup
  Proxy Collection Typed      :   0.8515 ms (Avg over 60)
  Proxy Collection Packed     :   0.7551 ms (Avg over 60)
  Broad Phase                 :   0.9865 ms (Avg over 60) - fluctuating between 1ms and 2ms
  Narrow Phase                :   4.9874 ms (Avg over 60)
cuda naive
  Proxy Collection Typed      :   0.8702 ms (Avg over 60)
  Proxy Collection Packed     :   0.7576 ms (Avg over 60)
  Broad Phase GPU             :   4.8022 ms (Avg over 60)
  Narrow Phase                :   5.0724 ms (Avg over 60)

## Sparse Islands 200 islands, 60 Steps

naive autovec parallel
  Broad Phase                 : 243.6436 ms (Avg over 60)
sap parallel
  Broad Phase                 :   9.4601 ms (Avg over 60)
    Sort                          :   6.8394 ms (Avg over 60)
    Sweep                         :   2.5956 ms (Avg over 60)
sap parallel insertion sort
  Proxy Collection Typed      :   4.4192 ms (Avg over 60)
  Broad Phase                 :   3.1404 ms (Avg over 60)
    Sort                          :   0.1838 ms (Avg over 60)
    Sweep                         :   2.9284 ms (Avg over 60)
sap parallel insertion sort lookup
  Proxy Collection Typed      :   3.0031 ms (Avg over 60)
  Broad Phase                 :   3.1053 ms (Avg over 60)
    Sort                          :   0.3571 ms (Avg over 60)
    Sweep                         :   2.7191 ms (Avg over 60)
