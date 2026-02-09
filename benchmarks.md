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

clang naive
  Broad Phase                 :  79.5087 ms (Avg over 30)
clang naive parallel
  Broad Phase                 :   8.6322 ms (Avg over 30)
clang naive simd
  Broad Phase                 :  18.4865 ms (Avg over 30)
clang naive autovec
  Broad Phase                 :  21.2804 ms (Avg over 20)
