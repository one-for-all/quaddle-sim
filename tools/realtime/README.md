# Realtime benchmarks

Headless tools for answering "how close to realtime does the simulator run, and
what is it spending its time on" without opening the page. Both build the scene
`createQuaddle` builds -- robot, ESP32-S3 board running the firmware in
`www/static/OpenCatEsp32S3`, beverage can, ground -- from this repo's own
sources, and step it at the page's substep (`simulator.run(30, 0)`, 1/1800 s).

Realtime rate here is simulated seconds per wall second with **all** of the
wall clock spent stepping. The page only gives the physics what is left of
each frame after drawing (`physicsBudgetMs` in gorilla-physics-ui's
`Simulator.ts`, roughly 70-80% at 60 fps), so the rate it shows is about that
fraction of what these report.

## native/ -- fast to build, for profiling

```sh
cd tools/realtime/native
cargo run --release -- 8 4.0:kwkF   # 8 simulated s, type "kwkF" at t = 4 s
QUIET=1 cargo run --release -- 8    # totals only
```

Every 0.5 simulated s it prints the realtime rate and the five most expensive
phases of `Hybrid::step` (`hybrid::profile`). At the end it prints a
fingerprint -- the robot's and the can's final pose and a hash of the console
-- for checking that a change did not change the simulation.

It reads the images from `www/static`, as the page does, through the native
branch of `QuaddleESP32S3Controller::new`.

## wasm/ -- what the page actually runs

```sh
cd tools/realtime/wasm
wasm-pack build --release --target nodejs --out-dir pkg
node run.mjs pkg 8 4.0:kwkF
```

The same release build and `wasm-opt` pass `npm run build` gets, run under
Node's V8 (Chrome's engine). Expect it to be ~1.2x slower than native.

To profile it, build with function names kept and use Node's profiler:

```sh
wasm-pack build --profiling --target nodejs --out-dir pkg-prof
node --cpu-prof --cpu-prof-dir=prof run.mjs pkg-prof 6 4.0:kwkF
python3 ../profile/node_cpuprof.py prof/*.cpuprofile self 40   # or: incl
```

## profile/ -- reading native profiles

```sh
# samply: per function / per source line
samply record --save-only -o prof.json.gz --rate 4000 -- target/release/quaddle-realtime 6 4.0:kwkF
dsymutil target/release/quaddle-realtime
python3 ../profile/samply_funcs.py prof.json.gz target/release/quaddle-realtime 40
python3 ../profile/samply_lines.py prof.json.gz target/release/quaddle-realtime 1500 50

# or macOS `sample` on a running bench: inclusive time per function
sample <pid> 10 -file report.txt && python3 ../profile/sample_inclusive.py report.txt "" 60
```

## Checking a change did not change the simulation

The fingerprint is only comparable between runs if they are deterministic, and
by default they are not: the emulated RNG peripheral (`read_rng` in
`esp32rs/src/esp32s3/peripheral/mod.rs`) draws from `thread_rng`, and the
firmware's use of it moves servo pulses by microseconds. For an exact A/B, make
`read_rng` return a fixed sequence in both builds being compared (a splitmix64
counter in a `thread_local` does), compare fingerprints, then put it back.

- An emulator-only change should leave the fingerprint identical to the bit.
- A change to how the contact QP is solved will not, and the robot's final pose
  can move by millimetres. Tell that apart from a real behaviour change by
  bracketing it: solve the same run at a much tighter tolerance and check the
  change lands with it. (A walking gait is where a too-loose solve shows: a
  biased foot friction accumulates over thousands of steps -- see
  `ISLAND_TOLERANCE` in gorilla-physics' `hybrid/cone_islands.rs`.)
