# 1Q Simulation Model Library

## Project Overview

A static library of simulation models for external service modules — airborne radar (AR), electronic surveillance radar (ESR), synthetic aperture radar (SAR), electro-optical sensor (EOS), flight dynamics, space-based infrared sensor (SBIRS), and remote identification radar (RIR) — plus electronic countermeasure (ECM), navigation, fusion, threat assessment and target-inference algorithm modules (stateless inference over track-state frames), and a precision-evaluation module that scores products against scenario truth, delivered as a single linkable artifact.

## Tech Stack

- Language: C++17 (minimum C++11)
- Build: CMake, Conan
- Test: GTest/GMock
- Runtime libraries: Eigen, nanoflann, Boost, FlatBuffers, zlib, sqlite3; non-Windows builds additionally install HighFive (SAR HDF5 output), spdlog/fmt (log backend) and JSBSim (used by `flight_dynamic` only).

## Directory Structure

- **Modules** — `include/1q/<module>/` + `src/<module>/`: `airborne_radar` (AR), `electronic_surveillance_radar` (ESR), `sar` (SAR), `electro_optical_sensor` (EOS), `flight_dynamic`, `sbirs_sensor` (SBIRS), `remote_identification_radar` (RIR), `electronic_countermeasure` (ECM), `navigation`, `fusion`, `threat_assessment`, `target_inference`, `precision_evaluation`
- **Shared** — public headers in `include/1q/<domain>/` (`coordinate`, `electromagnetics`, `environment`, `foundation`, `replay`); shared impl in `src/common/` (`atmosphere`, `coordinate`, `electromagnetics`, `estimation`, `geometry`, `logging`, `numerics`, `optimization`, `radar`, `rcs`, `replay`, `timing`, `tracking`, `validation`)
- **Tests** — `tests/`: `unit` (per-module), `integration`, `replay`, `contract`, `performance`, plus `compatibility` (script probes) and `consumer` (post-install verification); taxonomy and partition rules in `docs/practice/build_and_test_governance.md`. **Examples** — `examples/`; **Tools** — `tools/`; FlatBuffers schemas — `schemas/`
- **Build & config** — `CMakeLists.txt`, `CMakePresets.json` (shared) + `CMakeUserPresets.json` (machine-local, not in git), `conanfile.py`, `cmake/`, `.clang-format` / `.clang-tidy` / `.editorconfig`
- **Docs** — `docs/`: `common/contract.md` (cross-module contracts) + per-module design-doc sets (`design.md` + `boundaries.md` + `data-flow.md` + `algorithms.md`); the set is a requirement, not current state — some modules are still missing files, so check what exists on disk instead of assuming; build/test governance in `docs/practice/`

## Build and Test

- Build before test, serially, same preset; presets may run in parallel.
- Verify focused: `ctest --preset <preset> -R "unit::<module>"`. Do not run full debug-local suites; full suites run on release-local only (`/completeness-review`).
- macOS mainline: `llvm-ninja-*` presets, daily dev on the `*-local` variants from per-machine `CMakeUserPresets.json`; prefer release (JSBSim ~6× faster). Bootstrap/configure are one-time per preset — re-running them costs a full rebuild.
- Windows local: `VisualStudio.15.0-amd64` (v141, multi-config), entered through `scripts/1q.sh` from Git Bash or WSL. Environment, troubleshooting, delivery runbook: `docs/practice/windows_local_build.md`.

### macOS command flow

Same entry point as Windows — `scripts/1q.sh bootstrap|configure|build|test|install <preset>`:

```bash
scripts/1q.sh bootstrap llvm-ninja-release-local
scripts/1q.sh test llvm-ninja-release-local -R "unit::<module>"
```

The script resolves cmake/ctest, resolves each preset's own binaryDir, and defaults `ctest -j 4`.

### Windows command flow (v141)

Same entry point; `scripts/1q.sh doctor` must resolve Windows `cmake=` / `ctest=` — never point a Linux-native cmake at a `VisualStudio.*` preset, and build only the needed target. `1q.sh build` refuses an implicit reconfigure and names the configure command to run; configure/build families must match (configure `VisualStudio.15.0-amd64`, build/test `VisualStudio.15.0-amd64-release`). One-time setup, daily commands, clean-stale and incremental-build practice: `docs/practice/windows_local_build.md`.

### Delivery tier (VS2015 customer integration)

Contract: C++11 + UTF-8 BOM on every tracked C/C++ file + no `/utf-8` anywhere, maintained by `scripts/utf8_bom.py` + pre-commit + CI `check` + `.editorconfig`; verification runs through presets `1q_log_vs2015` / `VisualStudio.14.0-amd64-none`. Full runbook: `docs/practice/windows_local_build.md`.

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
- Never introduce C++ exceptions.

**Architecture**
- Prefer abstract interfaces at module boundaries.
- Compile-time cost is the repo accelerators' job (`ENABLE_PCH`, explicit instantiation — see `docs/practice/windows_local_build.md`); do not buy it with interface readability. Reach for forward declarations or PIMPL only where they leave the public header clearer.
- Guard against known edge cases, not hypothetical ones — don't over-defend at boundaries.

**Logging**
- Every `PROJECT_LOG_*` call site carries a Chinese comment above it (two-line style: `// 中译：…` + `// 标识：…`, explaining the log's meaning, trigger conditions, and state semantics for non-specialist developers); log message text stays in English. Gate default, platform policy and log directory: **Module and coverage switches** above.

## Session Workflow

**User work rules**
- Document clarification gate: confirm uncertain issues (direction, terminology, semantics, scope, gate values) with the user before writing — never embed silent assumptions; adjudicated content and frozen contracts need no re-confirmation.
- Plain-language bias: user-facing text favors plain language, terms explained on first use; readability outranks brevity; code comments and normative contracts keep full precision.
- Modular milestone cadence: for complex tasks spanning multiple business modules (AR, ESR, ECM, RIR, EOS, SAR, etc.) or multi-layer dependencies, avoid continuous execution across all modules in a single autonomous tool turn. Pause tool execution and report phase progress to the user upon completing each module (with its localized build and test verification), stating completed work, verified status, and next steps before proceeding.

**Branching**
- Evidence-first work uses `evidence/<topic>` then `feat/<topic>` (see `.claude/skills/evidence-first-freeze-contract`). After the user asks to merge: `git merge --no-ff feat/<topic>` into `main`, then delete **both** process branches.
- Other feature work: `feature/<short-description>` in kebab-case; merge into `main` with `--no-ff` only after user approval; delete the branch after merge.

**Plan gate**
- A session that will change code enters plan mode first and presents the implementation plan for approval; edit only after the user approves it. Read-only discussion is exempt. Evidence-first work passes this same gate before forking `feat/<topic>` — see `.claude/skills/evidence-first-freeze-contract`.

## Done Means
- The chosen preset builds successfully; relevant tests pass; the static library links cleanly into dependent targets.
- New public API or significant logic changes include new or updated tests under `tests/`.
- Documentation stays accurate: the relevant module design-doc set and its `[evidence: ...]` are updated when thresholds, limitations, or veto decisions change.
- When a plan was used, every plan item is implemented or explicitly marked as deferred.

<!-- CODEGRAPH_START -->
## CodeGraph

In repositories indexed by CodeGraph (a `.codegraph/` directory exists at the repo root), reach for it BEFORE grep/find or reading files when you need to understand or locate code:

- **MCP tool** (when available): `codegraph_explore` answers most code questions in one call — the relevant symbols' verbatim source plus the call paths between them, including dynamic-dispatch hops grep can't follow. Name a file or symbol in the query to read its current line-numbered source. If it's listed but deferred, load it by name via tool search.
- **Shell** (always works): `codegraph explore "<symbol names or question>"` prints the same output.

If there is no `.codegraph/` directory, skip CodeGraph entirely — indexing is the user's decision.
<!-- CODEGRAPH_END -->
