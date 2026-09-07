# 1Q Simulation Model Library

## Project Overview

A static library of simulation models for external service modules — airborne radar (AR), electronic surveillance radar (ESR), synthetic aperture radar (SAR), electro-optical sensor (EOS), flight dynamics, space-based infrared sensor (SBIRS), and remote identification radar (RIR) — plus electronic countermeasure (ECM), navigation, fusion, and threat assessment algorithm modules, delivered as a single linkable artifact.

## Tech Stack

- Language: C++17 (minimum C++11)
- Build: CMake, Conan
- Test: GTest/GMock
- Runtime libraries: Eigen, nanoflann, Boost, FlatBuffers, zlib, HighFive; plus spdlog/fmt and JSBSim (non-Windows only). Library debug log (`PROJECT_LOG_*`) is **off by default** (`ONEQ_ENABLE_FILE_LOG=OFF`); Windows stays off; macOS may turn it on only when debugging the library. Log directory is CMake `ONEQ_LOG_DIR`, not a hardcoded path.

## Directory Structure

- **Modules** — `include/1q/<module>/` + `src/<module>/`: `airborne_radar` (AR), `electronic_surveillance_radar` (ESR), `sar` (SAR), `electro_optical_sensor` (EOS), `flight_dynamic`, `sbirs_sensor` (SBIRS), `remote_identification_radar` (RIR), `electronic_countermeasure` (ECM), `navigation`, `fusion`, `threat_assessment`
- **Shared** — public headers in `include/1q/<domain>/` (`coordinate`, `electromagnetics`, `environment`, `foundation`, `replay`, `trace`); shared impl in `src/common/` (`estimation`, `geometry`, `logging`, `numerics`, `output`, `rcs`, `runtime`, `timing`, `trace`, `validation`)
- **Tests** — `tests/`: `unit` (per-module), `integration`, `contract`, `performance`, `consumer`; **Examples** — `examples/`; **Tools** — `tools/` (`schemas/` — FlatBuffers definitions)
- **Build & config** — `CMakeLists.txt`, `CMakePresets.json` (shared) + `CMakeUserPresets.json` (machine-local, not in git), `conanfile.py`, `cmake/`, `.clang-format` / `.clang-tidy` / `.editorconfig`
- **Docs** — `docs/`: `common/contract.md` (cross-module contracts) + per-module design-doc sets (`design.md` + `boundaries.md` + `data-flow.md` + `algorithms.md`); build/test governance in `docs/practice/`

## Build and Test

- Bootstrap + configure are **one-time** per preset; re-run only after dependency or CMake changes.
- Build before test, serially, same preset; presets may run in parallel. ctest `-j 4`.
- Verify focused: `ctest --preset <preset> -R "unit::<module>"`. Do not run full debug-local suites; full suites run on release-local only (`/completeness-review`).
- macOS mainline: `llvm-ninja-*` presets; daily dev uses `*-local` variants from per-machine `CMakeUserPresets.json`; prefer release (JSBSim ~6× faster); take install paths from the preset's actual binaryDir (`llvm-ninja-release` has no `-local` suffix).
- Windows local: `VisualStudio.15.0-amd64` (v141, multi-config). Entry is `scripts/1q.sh` (Git Bash or WSL). Environment, troubleshooting, delivery runbook: `docs/practice/windows_local_build.md`.

### macOS command flow

One-time:

```bash
bash scripts/bootstrap_conan.sh "$preset" >"${log_prefix}-conan.log" 2>&1 || { tail -n 80 "${log_prefix}-conan.log"; false; }
cmake --preset "$preset" >"${log_prefix}-cmake.log" 2>&1 || { tail -n 80 "${log_prefix}-cmake.log"; false; }
```

Daily:

```bash
cmake --build --preset "$preset" >"${log_prefix}-build.log" 2>&1 || { tail -n 80 "${log_prefix}-build.log"; false; }
ctest --preset "$preset" -R "unit::<module>" --output-on-failure -j 4
cmake --install "build/${preset}"   # packaging only
```

### Windows command flow (v141)

Use `scripts/1q.sh` from Git Bash or WSL. `doctor` must show Windows `cmake=` / `ctest=` paths. Do not point a Linux-native cmake at `VisualStudio.*` presets. Build only the needed target; BOM is auto-fixed by the pre-commit hook. After public-header layout changes, batch SEH `0xc0000005` / spurious `bad_alloc` → `scripts/1q.sh clean-stale VisualStudio.15.0-amd64 <module>`, then rebuild.

Bootstrap/configure **once per preset** (or after dependency / CMake changes only). **`1q.sh build` never implicit-reconfigures**: if CMake inputs are newer than `generate.stamp`, build fails and tells you to run `configure` explicitly. Switching presets requires matching configure/build families yourself (e.g. configure `VisualStudio.15.0-amd64`, build/test `VisualStudio.15.0-amd64-release`) — do not reuse another preset's build tree. Base preset sets `CMAKE_SUPPRESS_REGENERATION=ON` so VS/MSBuild does not silently re-run cmake during build.

One-time:

```bash
source scripts/activate_1q_git_bash.sh
scripts/1q.sh bootstrap VisualStudio.15.0-amd64
scripts/1q.sh configure VisualStudio.15.0-amd64
```

Daily:

```bash
scripts/1q.sh build VisualStudio.15.0-amd64-release --target 1q_<module>_unit_tests
scripts/1q.sh test VisualStudio.15.0-amd64-release -R "unit::<module>"
```

**Incremental build & Eigen (recommended daily practice):**

- **Release preset only** for routine dev (`VisualStudio.15.0-amd64-release`); Debug (`/Od` + `/RTC1`) is much slower on Eigen-heavy TUs.
- **Always pass `--target`** (e.g. `1q_fusion_unit_tests`); bare `cmake --build` builds `ALL_BUILD` (~full solution).
- **Avoid touching `src/common/` and `include/1q/` headers** unless necessary — e.g. `common/estimation/EkfFilter.h` is included across fusion / sbirs / rir / AR; one edit cascades to many TUs. Prefer changes in `src/<module>/*.cpp`.
- **Do not re-run `configure`** except after dependency or CMake changes; it triggers a one-time full rebuild. If build fails with stale CMake inputs, run `scripts/1q.sh configure VisualStudio.15.0-amd64` explicitly (expected full rebuild), then resume incremental builds.
- **Sanity check**: a no-op second build should finish in seconds with no `编译源文件 xxx.cpp` lines; long MSBuild logs of `*.vcxproj ->` alone are up-to-date checks, not full recompiles.
- **Repo accelerators** (Windows v141 preset): `ENABLE_PCH=ON` precompiles `<Eigen/Core>` + common STL; `src/common/estimation/EstimationInstantiations.cpp` explicitly instantiates **6×3 / 6×2** Kalman/EKF/UKF/IMM (`extern template` elsewhere). Other test-only dimensions (e.g. 4×2) still instantiate locally in test TUs.
- Full troubleshooting and delivery notes: `docs/practice/windows_local_build.md` (section **Eigen 编译耗时与日常优化**).

### Delivery tier (VS2015 customer integration)

- Contract: C++11 + UTF-8 BOM on every tracked C/C++ file + no `/utf-8` anywhere — customers compile our headers inside their own VS2015 TUs with no extra flags.
- Maintenance chain: `scripts/utf8_bom.py` + pre-commit hook + CI `check` + `.editorconfig`.
- Presets: `1q_log_vs2015` / `VisualStudio.14.0-amd64-none`.
- Verification project: zero-modification consumer mirror of the installed `1q.lib` as C++11 / no-`/utf-8` (VS2015 generator). Pass = zero C4819 + acceptance rows in the configured log directory (do not require the library debug log). Runbook: `docs/practice/windows_local_build.md`.

### Module and coverage switches

- `ONEQ_ENABLE_FLIGHT_DYNAMIC` (default **OFF**) gates `src/flight_dynamic/` and its tests; JSBSim is a flight_dynamic-exclusive dependency. Windows conan never installs jsbsim — with FD=ON use `ONEQ_JSBSIM_FROM_SOURCE` (third_party source) or `ONEQ_JSBSIM_PREBUILT_ROOT_DIR` (prebuilt tree).
- `ONEQ_ENABLE_FILE_LOG` (default **OFF**) is the library debug-log gate. Leave it off on Windows. On macOS only, pass `-DONEQ_ENABLE_FILE_LOG=ON` when debugging library internals. Acceptance metrics stay in the acceptance files, not the debug log.
- `ONEQ_LOG_DIR` (default empty = `<cwd>/log`) is how integrators pin where those files go: `-DONEQ_LOG_DIR=<dir>` at configure. Do not hardcode log paths in source or in this file. First-party examples may still override a single file at runtime for scene-scoped output.
- Coverage (macOS only): `llvm-ninja-coverage` preset + `tools/coverage_report.sh` — **never run ctest by hand** (the script owns `.profraw` placement). Branch coverage is the primary metric; see `docs/practice/coverage.md`.

## Documentation

Each module keeps a design-doc set — `design.md` (navigation entry) + `boundaries.md` + `data-flow.md` + `algorithms.md` — as its design authority: positioning, boundaries, non-goals, veto rationale, and `[evidence: ...]` annotations live there; step-by-step algorithm logic belongs in code. Update the set whenever thresholds, limitations, or veto decisions change.

## Engineering Conventions

**The project has not been delivered: refactoring does not chase excessive compatibility — clean boundaries are the first principle.**

**Style**
- Follow the Google C++ Style Guide.
- Keep namespace-directory mapping consistent.
- Keep changes in `src/` unless a public API change is unavoidable.
- Mark variables and member functions `const` by default; relax only when mutation is required.
- Never introduce C++ exceptions.
- Never reformat existing code that was not touched by the current change.

**Architecture**
- Prefer abstract interfaces at module boundaries.
- Prefer forward declarations to reduce includes and rebuild cost.
- Use PIMPL for critical classes when hiding implementation reduces recompilation propagation.
- Make interfaces easy to use correctly and hard to use incorrectly.
- Guard against known edge cases, not hypothetical ones — don't over-defend at boundaries.

**Process**
- Automated bulk edits: verify the exact command on 1-2 files first, then proceed in batches of ≤5 files.
- Semantic refactors: commit as the smallest buildable and testable closure; enumerate the dependency closure and validate at real compileable boundaries.

**Logging**
- Library debug log (`PROJECT_LOG_*`) defaults **OFF**. Windows stays off — do not enable `ONEQ_ENABLE_FILE_LOG` for daily v141 work. macOS may enable it only when debugging library internals (`-DONEQ_ENABLE_FILE_LOG=ON`).
- Integrators set the log directory with CMake `ONEQ_LOG_DIR` when building the library. Do not hardcode output paths. Acceptance rows go to the acceptance files under that directory.
- Log critical actions and failures using the project's logging facility, when available.
- Every `PROJECT_LOG_*` call site should carry a Chinese comment above it (two-line style: `// 中译：…` + `// 标识：…`, explaining the log's meaning, trigger conditions, and state semantics for non-specialist developers); log message text stays in English.

## Session Workflow

**User work rules**
- Document clarification gate: confirm uncertain issues (direction, terminology, semantics, scope, gate values) with the user before writing — never embed silent assumptions; adjudicated content and frozen contracts need no re-confirmation.
- Plain-language bias: user-facing text favors plain language, terms explained on first use; readability outranks brevity; code comments and normative contracts keep full precision.
- Modular milestone cadence: for complex tasks spanning multiple business modules (AR, ESR, ECM, RIR, EOS, SAR, etc.) or multi-layer dependencies, avoid continuous execution across all modules in a single autonomous tool turn. Pause tool execution and report phase progress to the user upon completing each module (with its localized build and test verification), stating completed work, verified status, and next steps before proceeding.

**Branching**
- Evidence-first work uses `evidence/<topic>` then `feat/<topic>` (see `.claude/skills/evidence-first-freeze-contract`). After the user asks to merge: `git merge --no-ff feat/<topic>` into `main`, then delete **both** process branches.
- Other feature work: `feature/<short-description>` in kebab-case; merge into `main` with `--no-ff` only after user approval; delete the branch after merge.
- Pre-commit does **not** auto-create `feature/auto-*` on `main`/`master`.

**Commits**
- Conventional Commits: `type(scope): description` — types `feat`/`fix`/`refactor`/`test`/`docs`/`chore`/`perf`; scope = primary module; imperative, lowercase.
- End every message with `Co-Authored-By: Claude <noreply@anthropic.com>`.

**Commit gate**
- Pre-commit hook blocks `major` C++ changes (≥3 core files / ≥50 core lines) until `/completeness-review`; then retry with `ONEQ_ALLOW_MAJOR=1 git commit ...`. `minor`/`trivial` pass.

## Done Means
- The chosen preset builds successfully; relevant tests pass; the static library links cleanly into dependent targets.
- New public API or significant logic changes include new or updated tests under `tests/`.
- Documentation stays accurate: the relevant module design-doc set and its `[evidence: ...]` are updated when thresholds, limitations, or veto decisions change.
- When a plan was used, every plan item is implemented or explicitly marked as deferred.
