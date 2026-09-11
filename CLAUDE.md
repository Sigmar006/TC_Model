# 1Q Simulation Model Library

## Project Overview

**The project has not been delivered: refactoring does not chase excessive compatibility — clean boundaries are the first principle.**

A static library of simulation models for external service modules — airborne radar (AR), electronic surveillance radar (ESR), synthetic aperture radar (SAR), electro-optical sensor (EOS), flight dynamics, space-based infrared sensor (SBIRS), and remote identification radar (RIR) — plus electronic countermeasure (ECM), navigation, fusion, threat assessment and target-inference algorithm modules (stateless inference over track-state frames), and a precision-evaluation module that scores products against scenario truth, delivered as a single linkable artifact.

## Directory Structure

```text
include/1q/<module>/    module public headers
include/1q/<domain>/    shared public headers
src/<module>/           module implementations
src/common/             shared implementations
docs/                   navigation entry: docs/README.md
tests/ examples/ tools/ schemas/
```

Modules: airborne_radar (AR), electronic_surveillance_radar (ESR), sar (SAR), electro_optical_sensor (EOS), flight_dynamic, sbirs_sensor (SBIRS), remote_identification_radar (RIR), electronic_countermeasure (ECM), navigation, fusion, threat_assessment, target_inference, precision_evaluation.

## Build and Test

Test taxonomy and partition rules: `docs/practice/build_and_test_governance.md`.
`CMakeUserPresets.json` is machine-local and not in git — the `*-local` presets come from it.

macOS:

```bash
scripts/1q.sh configure llvm-ninja-release-local   # one-time; auto-bootstraps Conan deps
scripts/1q.sh build llvm-ninja-release-local
scripts/1q.sh test llvm-ninja-release-local -R "unit::<module>"
```

Windows (Git Bash / WSL):

```bash
scripts/1q.sh configure VisualStudio.15.0-amd64    # one-time; auto-bootstraps Conan deps
scripts/1q.sh build VisualStudio.15.0-amd64-release
scripts/1q.sh test VisualStudio.15.0-amd64-release -R "unit::<module>"
```

## Engineering Conventions

**Style**
- Write C++11-compatible C++ — VS2015 delivery floor (dev presets build with 17).
- Follow the Google C++ Style Guide.
- Keep namespace-directory mapping consistent.
- Keep changes in `src/` unless a public API change is unavoidable.
- Never introduce C++ exceptions.

**Architecture**
- Prefer abstract interfaces at module boundaries.
- Guard against known edge cases, not hypothetical ones — don't over-defend at boundaries.
- Prioritize public header readability: avoid complex PIMPL or fragmented forward declarations purely to reduce build time.
- Dependencies come from `conanfile.py` — reach for nothing else; JSBSim is flight_dynamic-exclusive and never on Windows.

**Tests**
- New public API or significant logic changes come with new or updated tests under `tests/`.

**Logging**
- Every `PROJECT_LOG_*` call site carries a two-line Chinese comment above it (`// 中译：…` + `// 标识：…`, for non-specialist readers); message text stays in English.
- `ONEQ_ENABLE_FILE_LOG` defaults OFF — always off on Windows, on macOS only when debugging internals; directory via `ONEQ_LOG_DIR`, never hardcoded; acceptance metrics never go to the debug log.

## Session Workflow
- **Modular cadence (Strict)**: For tasks spanning multiple modules (AR, ESR, SAR, etc.), complete and verify one module (build + unit test) before touching the next. Report progress and pause for user check-in between modules.
- **Plan before complex refactor**: Propose an implementation plan before making architectural or multi-module changes; trivial single-file edits do not require a plan.
- **Ask on ambiguity**: Clarify ambiguous requirements or boundary values before editing code; do not make silent assumptions.

## CodeGraph

In repositories indexed by CodeGraph (a `.codegraph/` directory exists at the repo root), reach for it BEFORE grep/find or reading files when you need to understand or locate code:

- **MCP tool** (when available): `codegraph_explore` answers most code questions in one call — the relevant symbols' verbatim source plus the call paths between them, including dynamic-dispatch hops grep can't follow. Name a file or symbol in the query to read its current line-numbered source. If it's listed but deferred, load it by name via tool search.
- **Shell** (always works): `codegraph explore "<symbol names or question>"` prints the same output.

If there is no `.codegraph/` directory, skip CodeGraph entirely — indexing is the user's decision.
