# 1q

`1q` is a simulation model library for external service modules. It currently covers seven main sensor modules — airborne radar (AR), electronic surveillance radar (ESR), synthetic aperture radar (SAR), electro-optical sensor (EOS), flight dynamics (Flight Dynamic), space-based infrared sensor (SBIRS), and remote identification radar (RIR) — plus electronic countermeasure (ECM), area-coverage planning (`navigation`), multi-source fusion (`fusion`), threat assessment (`threat_assessment`), target inference (`target_inference`), and precision evaluation (`precision_evaluation`) algorithm modules. The project focuses on a stable public API, replaceable internal components, and testable simulation-chain orchestration.

## Module Overview

- `include/1q/airborne_radar/`, `src/airborne_radar/`: airborne radar public API and implementation, covering environment, decision-making, signal processing, tracking, and session orchestration.
- `include/1q/electronic_surveillance_radar/`, `src/electronic_surveillance_radar/`: ESR public API and implementation, covering environment, interception, and pipeline orchestration.
- `include/1q/sar/`, `src/sar/`: synthetic aperture radar public API and implementation.
- `include/1q/electro_optical_sensor/`, `src/electro_optical_sensor/`: electro-optical sensor public API and implementation.
- `include/1q/flight_dynamic/`, `src/flight_dynamic/`: flight dynamics, guidance, and maneuver models.
- `include/1q/sbirs_sensor/`, `src/sbirs_sensor/`: space-based infrared sensor (SBIRS) public API and implementation, covering environment, error models, NFOV scheduling, and processing pipelines.
- `include/1q/remote_identification_radar/`, `src/remote_identification_radar/`: remote identification radar public API and implementation, covering detection, tracking, feature identification, and session orchestration.
- `include/1q/electronic_countermeasure/`, `src/electronic_countermeasure/`: electronic countermeasure public API and implementation.
- `include/1q/{navigation,fusion,threat_assessment,target_inference,precision_evaluation}/`, `src/{navigation,fusion,threat_assessment,target_inference,precision_evaluation}/`: area-coverage planning, multi-source fusion, threat assessment, target inference, and precision evaluation algorithm modules.
- `include/1q/{coordinate,electromagnetics,environment,foundation,replay,trace}/`: cross-module shared coordinate, electromagnetics, environment, foundation-type, replay, and trace interfaces.
- `tests/`: unit, integration, contract, performance, and install-consumer tests.
- `examples/`: per-module quick-start, session usage, and integration examples.

## Dependencies

- C++11 for delivery (VS2015 presets build with `CMAKE_CXX_STANDARD=11`; public headers guard a C++11 subset). Development/CI presets default to C++17 (`PROJECT_DEFAULT_CXX_STANDARD = 17`) for the optional flight-dynamic module, which is not part of the delivery.
- CMake / Conan (Windows also has a no-Conan mode; see "Alternative paths" below)
- GTest / GMock (tests)
- Eigen, nanoflann, Boost, FlatBuffers, zlib, sqlite3
- spdlog / fmt (logging; installed on non-Windows only). Library debug log (`PROJECT_LOG_*`) is off by default (`ONEQ_ENABLE_FILE_LOG=OFF`); Windows stays off; macOS may enable it only when debugging the library. Integrators pin the log directory with CMake `ONEQ_LOG_DIR`.
- JSBSim (a `flight_dynamic`-specific optional dependency, provided prebuilt via Conan on non-Windows only), HighFive (SAR HDF5 output; non-Windows only)

## Build

The repository convention is to drive builds through CMake presets, running `bootstrap → configure → build → test` serially under the same preset: `scripts/bootstrap_conan.sh` derives per-preset arguments, runs `conan install`, and generates `conan_toolchain.cmake` for the configure step to consume. macOS and Windows differ in presets, dependency sets, and build mechanics.

### macOS (development/CI mainline)

- Presets defined in `CMakePresets.json`: `llvm-ninja-debug` / `llvm-ninja-release` (Ninja single-config, Darwin only).
- Daily development typically uses the local variants `llvm-ninja-debug-local` / `llvm-ninja-release-local` (`CMakeUserPresets.json`, not in git).
- Conan provides the full dependency set (including spdlog/fmt, the prebuilt JSBSim package, and HighFive).

```bash
bash scripts/bootstrap_conan.sh llvm-ninja-release-local
cmake --preset llvm-ninja-release-local
cmake --build --preset llvm-ninja-release-local
ctest --preset llvm-ninja-release-local --output-on-failure
cmake --install build/llvm-ninja-release-local   # installs to build/install/<preset>
```

### Windows (v141 toolset mainline)

- Preset: `VisualStudio.15.0-amd64` (VS2026 generator + legacy v141/14.16 toolset, multi-config; the build preset's `--config` selects Debug/Release).
- Dependency differences: no spdlog/fmt (replaced by the built-in file-logging backend), no HighFive, no JSBSim.
- Examples build on Windows too: enable with `-DENABLE_EXAMPLES=ON`. The `component_attachment` integration logger runs its own `std::ofstream` file backend on Windows (`CA_LOG_BACKEND_SPDLOG=0`, same log-line text as the spdlog branch); `integration_events.log` / `integration_views.log` / `1q_library.log` all land under the demo's `--output-dir`.
- Bootstrap, configure, build, and test go through `scripts/1q.sh` (Git Bash or WSL). Since 2026-08-21 the machine's 64-bit registry `KitsRoot10` points at the real Windows SDK again, so raw-directory builds work and presets no longer inject `UCRTContentRoot`; build presets (`scripts/1q.sh` / `cmake --build --preset ...`) remain the recommended entry.

```bash
bash scripts/bootstrap_conan.sh VisualStudio.15.0-amd64
cmake --preset VisualStudio.15.0-amd64
cmake --build --preset VisualStudio.15.0-amd64-release
ctest --preset VisualStudio.15.0-amd64-release --output-on-failure
cmake --install build/VisualStudio.15.0-amd64 --config Release
```

### Alternative paths (unaccepted scaffolding)

- VS2015: preset `VisualStudio.14.0-amd64` (C++11, no tests).
- No Conan: run `scripts\win\fetch_third_party.bat` first to fetch pinned dependency sources into `third_party/` (consumed by `VendorPackages.cmake`), then use preset `VisualStudio.14.0-amd64-none` (C++11, delivery tier — same as `1q_log_vs2015` which adds acceptance logs).
- Encoding: all tracked C/C++ sources carry a UTF-8 BOM (`scripts/utf8_bom.py` maintains; a `.githooks/pre-commit` hook auto-prepends it at commit time — activate via `scripts/win/activate_git_bash.sh`; CI enforces via `check`). MSVC reads BOM files as UTF-8 unconditionally — no `/utf-8` compile flag is needed or propagated, so consumers on VS2015 (any Update) compile Chinese-commented headers correctly without any special options.

Full dual-platform instructions: see the Build and Test section of `CLAUDE.md`; Windows environment setup, troubleshooting, delivery-tier runbook, and historical errata: `docs/practice/windows_local_build.md`; build/test governance: `docs/practice/build_and_test_governance.md`.

## Examples

`examples/` is the **consumer integration reference** (how to write a program that links against the library, using only the public `include/` surface):

- `examples/{core,components,app,scenes,logger,basic_config}/`: the scene suite — one executable per scene (`scenes/<name>/main.cpp`), mounting AR/ESR/EOS/SBIRS/SAR sensor components + fusion + threat assessment + multi-aircraft formations, driven by self-contained scene JSON (`session_config` + `log_dir` per scene; e.g. the multi-aircraft area-patrol `fleet_patrol_multi_zone` scene).
- `examples/common/`: example-layer shared convenience layer (`json_reader` + six-domain `config_loaders/` + the `viz/` shared visualization viewer; not part of the library public surface).
- `examples/basic_config/`: cross-module shared configuration samples.

Validation and development-time tooling has moved out of examples (role separation): the multi-scene batch validation framework lives in `tests/consumer/batch_validation/`; flight-dynamics development trajectory tools in `tests/unit/flight_dynamic/fd_tools/`.

## Documentation

- `CLAUDE.md`: engineering constraints, build/test rules, and refactoring strategy.
- `docs/<module>/design.md`: current design per module (AR / ESR / SAR / EOS / Flight Dynamic / SBIRS / RIR / ECM / navigation / fusion / threat_assessment / target_inference / precision_evaluation).
- `docs/common/`: cross-module contracts and open questions (`contract.md`, `open_questions.md`).
- `docs/practice/`: engineering-practice and infrastructure design docs (the batch scene validation framework, etc.).
- `docs/review/`: module reviews and migration plans.
- `include/1q/README.md`: public header navigation and integration advice.
- `tests/README.md`: test layering conventions and running advice.

## Testing

Tests cover association, tracking, environment, decision-making, session orchestration, contracts, performance, and the install-consumer path (see `tests/{unit,integration,contract,performance,consumer}/`). When changing the public API or key logic, add or update tests under `tests/` accordingly.

## Constraints

- No C++ exceptions.
- No logging on high-speed simulation/math hot paths.
- Prefer containing changes inside `src/` before widening the public header surface.
