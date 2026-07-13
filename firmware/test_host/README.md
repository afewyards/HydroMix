# Host Test Harness (`ctrl_core`)

Plain CMake + [Unity](https://github.com/ThrowTheSwitch/Unity) host test suite
for `firmware/components/ctrl_core/` — the pure-C control logic layer (zero
IDF/FreeRTOS/driver includes, time passed in as a parameter). Runs on macOS/
Linux with no ESP-IDF toolchain and no hardware.

## Running the tests

```
cd firmware/test_host
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expect `100% tests passed` once everything builds.

Unity (v2.6.0) is vendored via CMake `FetchContent` and pinned by
`GIT_TAG` in `test_host/CMakeLists.txt` — it's downloaded once on the first
`cmake -S . -B build` and cached under `build/`, so no network access is
needed on subsequent runs (as long as `build/` isn't deleted).

## How new tests get picked up

`test_host/CMakeLists.txt` uses `file(GLOB ...)` (without `CONFIGURE_DEPENDS`)
to collect both `../components/ctrl_core/*.c` and `test_*.c`, then creates one
executable + one `ctest` entry per test file automatically — you don't need to
edit `CMakeLists.txt` when adding a new `ctrl_core` module or test file.

**Gotcha:** because the GLOB isn't re-evaluated automatically, adding a *new*
source or test file requires re-running the full configure step
(`cmake -S . -B build`) — just running `cmake --build build` on its own won't
pick it up and will fail with an "undefined reference" link error even though
the new file exists on disk. Always use the full three-command sequence above
(or at least re-run `cmake -S . -B build`) after adding a file.

To run a single test in isolation (useful when other `ctrl_core` modules are
mid-edit and might not build yet), target it by name, e.g.:

```
cmake --build build --target test_degradation
ctest --test-dir build -R test_degradation --output-on-failure
```

## Current test files

`test_alarm.c`, `test_config_map.c`, `test_control.c`, `test_degradation.c`,
`test_feedforward.c`, `test_governor.c`, `test_interlock.c`,
`test_mode_detect.c`, `test_pi.c`, `test_pos_estimator.c`, `test_smoke.c`.
