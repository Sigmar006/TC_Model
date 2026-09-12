---
name: completeness-review
description: Heavy, merge-gating review of the current change set — run ONLY when the user explicitly types /completeness-review. Never self-invoke it: a general request to look at, review, or sanity-check some changes must be answered directly, not by this skill. This flow runs review lanes, build/test verification, and offers to merge a branch, so invoking it has to be a deliberate act. Depth follows risk — core-library changes always run the code-review lane; the code-simplifier lane only when the review gates a branch merge; the static test-coverage lane only when the change adds a new module (refactors and deletions skip both); example/doc/deletion layers get a light single-lane review.
argument-hint: "[optional diff range]"
allowed-tools:
  - Bash
  - Read
  - Grep
  - Agent
  - Skill
---

# Completeness Review

Review the current change set for correctness, code quality, and test coverage — against the plan
(when one exists) — with a quality-gated merge flow. The plan anchors the review: it provides the
change's intent, so agents can distinguish deliberate behavior changes from bugs, and can judge
whether the change is complete.

## Step 0: Determine scope

1. **Plan file**: use user-provided path, or search:
   ```bash
   ls -t .claude/plans/*.md .zcode/plans/*.md ~/.claude/plans/*.md 2>/dev/null | head -1
   ```
   If no plan file exists, ask the user for a one-sentence statement of the change's intent —
   it is the review anchor in place of a plan.

2. **Diff range**:
   ```bash
   git diff --stat HEAD && git diff --stat --cached
   ```
   If uncommitted changes exist → review working tree. Otherwise → `git diff main...HEAD`.

3. **Exclude unrelated user changes**: ask the user whether the working tree contains changes not
   belonging to this work; if so, restrict review to the stated files/modules. Do not fold unrelated
   changes into the review or the later merge flow.

## Step 0.5: Classify review depth (follows risk)

Classify the change first:

```bash
python3 scripts/pre-commit-review.py
```

- **tier = major** — C++ changes touching the **core library** (`src/` or `include/1q/`,
  ≥3 files or ≥50 lines): algorithm/contract changes. Run the **full review** (Step 3): the
  code-review lane always; the code-simplifier lane only when this review gates a branch merge;
  the test-coverage lane only when the change adds a new module.
- **tier = minor** — C++ changes confined to non-core layers (`examples/`, `tests/`, `docs/`,
  `tools/`, `cmake/`): example-layer refactors, test-only edits, pure deletions. Review depth is
  **light**: single code-review lane + static test-coverage check +
  build/tests (see Step 3-Light). Compile-time checks (fmt consteval, static asserts) already cover
  much of what a full lane would find in this layer — do not over-review example code.
- **tier = trivial** — no C++ changes. No review lanes; build/tests only if the change affects build
  inputs.

State the tier and the chosen depth in the final report.

## Step 1: Collect changes

```bash
git diff --stat <range>
```
Group changed files by module (Modules, Shared headers, Shared impl, Tests, Docs).

## Step 2: Plan cross-reference (if plan exists)

Read the plan. Extract action items / checklists. Cross-reference each item against changed files:

- ✅ **Done**: plan item has corresponding implementation with substance (not a stub)
- ⚠️ **Partial**: related files changed but missing core logic, stubbed out, no error handling
- ❌ **Missing**: plan item has no corresponding changes at all

Output a table:
```
| Plan Item | Status | Evidence File | Notes |
|-----------|--------|---------------|-------|
| Add AR replay codec | ✅ | src/airborne_radar/session/ar_replay_codec.cpp | Full implementation |
| Fix ESR memory leak | ⚠️ | src/electronic_surveillance_radar/processor.cpp | Comment only, no fix |
| Update SAR design.md | ❌ | - | No changes found |
```

Pass the plan's intent summary to all triggered review lanes as shared context.

## Step 3: Deep review

Review depth follows the Step 0.5 tier. All lanes receive: the diff range, the plan/intent summary
(from Step 0/2), and the excluded-files list (from Step 0).

### Step 3-Full (tier = major, core library)

Lanes are **condition-triggered**; only Lane 1 runs on every full review. Check each trigger
against the diff (Step 1) and the review context (Step 0), state in the final report which lanes
ran and which were skipped as not triggered. Triggered lanes run in parallel via the Agent tool
(subagent types `code-review` and `code-simplifier` are registered at `~/.zcode/agents/`):

#### Lane 1: Correctness (subagent: code-review) — 必跑

Launch the `code-review` subagent (registered at `~/.zcode/agents/code-review.md`) to review the diff
for:
- Logic errors, boundary conditions (nullptr, empty containers, division-by-zero, out-of-range)
- Error-handling completeness, RAII / resource leaks
- Compliance with AGENTS.md Engineering Conventions (const correctness, no exceptions, namespace consistency)
- Architecture-level correctness: missing switch cases, inconsistent parallel implementations across files, state-machine invariants
- Divergence from the plan/intent: deliberate behavior changes should be checked against the plan,
  not flagged as bugs; missing plan pieces should be flagged as incompleteness

> The `code-review` subagent carries the generic correctness-review prompt (severity-tagged
> `[高/中/低] [file:line]` findings). Project conventions come from AGENTS.md, which the subagent
> reads itself.

#### Lane 2: Code quality (subagent: code-simplifier) — 仅合并门触发

**Trigger**: this review is gating a branch merge — the final review before Step 9's
`merge --no-ff`, or invoked as the merge-gate by the hook/user. **Skip** for routine feature
commits, refactors, and deletions: the quality sweep pays off once per branch, at merge time,
not on every intermediate review.

When triggered, launch the `code-simplifier` subagent (registered at
`~/.zcode/agents/code-simplifier.md`) against the recently modified code. It reviews for reuse,
simplification, naming, redundancy, and maintainability, applying project conventions from
AGENTS.md. **It reports findings only — it must not modify code during review** (see its prompt).

#### Lane 3: Test coverage (static check) — 仅新增模块触发

**Trigger**: the change set adds a **new module** — a new top-level directory under `src/`, a new
`include/1q/<module>/` public directory, or a new test partition in `tests/cmake/partitions/`.
**Skip** for refactors, deletions, and in-module changes: existing-partition tests already pin
those surfaces, and Step 7's build+ctest run carries the runtime verification. New public API
**inside an existing module** also skips this lane (its partition's tests are the coverage point).

When triggered, launch an Agent to check — **statically only**:
- Do `tests/` directories contain new/modified tests matching the changed modules?
- Do tests cover critical paths and boundary conditions?
- For the new module: does it register a full test partition and cover its public API?

**Hard constraints for Lane 3 (and the whole review): never stash, never build, never run tests
inside a review lane.** Static verification only: read the diff, grep existing tests, read the
relevant test files. Build/test verification happens once, in Step 7, by the main flow. A lane that
needs a runtime measurement should flag it as a [低] observation instead of measuring it.

### Step 3-Light (tier = minor, example/doc/deletion layers)

Run a **single** correctness lane (Lane 1: code-review subagent). Skip the code-simplifier lane —
the example layer favors readability over abstraction and the change itself is the
simplification. Skip the test-coverage lane unless the change adds a new module (same trigger as
Step 3-Full Lane 3). Time-box the whole light review: target minutes, not tens of minutes. If the
change is a pure deletion with no behavior change (e.g., removing unused files), the code-review
lane may be skipped entirely after a manual leftover-reference grep; build + focused tests carry
the verification.

## Step 4: Verify and triage findings

Before writing the report, verify each finding from all lanes:

1. **Verify `[file:line]`**: confirm the cited location exists and the claim is accurate against
   live code. Drop findings that do not survive a direct check (misread, pre-existing, out of scope).
2. **Deduplicate**: merge overlapping findings across lanes (same issue reported by two lanes).
3. **Triage severity**:
   - **高 (blocking)** — breaks correctness, contract, data flow, or leaves the change incomplete
     (missing core plan piece, unhandled failure path, no tests for new public API)
   - **中 (recommended)** — real but non-blocking: cleanup, style drift, test gaps on non-critical paths
   - **低 (suggestion)** — nitpicks, naming, optional improvements

Only findings that survive verification enter the report. Report one severity per finding; if a
finding is disputed, mark it with the counter-evidence rather than silently dropping or keeping it.

## Step 5: Report

```markdown
## Completeness Review Report

### Plan Coverage
| Plan Item | Status | Risk |
|-----------|--------|------|

### Correctness Issues
- [severity] [file:line] Description

### Code Quality (code-simplifier)
- [severity] [file:line] Issues found (reported only, no code changes)
- （未触发：非合并门审查）when this review is not gating a merge

### Test Coverage
- Existing tests covering changes
- Missing test coverage
- （未触发：未新增模块，由 Step 7 构建与聚焦测试承载）when no new module was added

### Gate Status
- Plan gate: PASSED / BLOCKED
- Correctness gate: PASSED / BLOCKED (N blocking)
- Test coverage gate: PASSED / BLOCKED (M gaps)
- Recommended (non-blocking): N items — may be deferred to follow-up commits
```

## Step 6: Quality gate

Evaluate three gates. **Blocking criteria are severity-gated**: only 高 (blocking) findings block;
中/低 items are reported as recommendations and do not block the merge path.

#### Gate A: Plan completeness
- PASS: all plan items are ✅, or any ⚠️/❌ is explicitly accepted by the user (scope change).
- BLOCKED: a ❌ Missing or core ⚠️ Partial item exists and the user has not accepted it.

#### Gate B: Correctness
- PASS: zero 高 (blocking) correctness issues.
- BLOCKED: any 高 issue exists. 中/低 items do not block.

#### Gate C: Test coverage
- PASS: no missing tests on critical paths (new public API, significant logic changes).
- PASS (lane not triggered): refactors/deletions without a new module — coverage is carried by
  Step 7's build + focused tests on the touched partitions; no static lane required.
- BLOCKED: critical paths lack tests. Gaps on non-critical paths are 中/低 recommendations.

#### If BLOCKED → fix loop

Output the blocking items and direct the user to iterate:

```
🚫 Merge blocked. Blocking issues:

  Plan:       [item] ❌ missing
  Correctness: [file:line] nullptr dereference risk (高)
  Test:      No tests for the new AR replay path (高)

Recommended (non-blocking): 3 items — see report.

Fix the blocking issues, then re-run /completeness-review.
```

**Stop here.** Do not ask about merge, do not proceed to build.

#### If all PASS → proceed to Step 7

```
✅ All gates passed. Proceeding to build verification...
```

## Step 7: Build & test verification

Verify the build with the repository-prescribed workflow. **Use the release preset** (see the
`llvm-ninja-release` preset description — JSBSim runs ~6× faster than debug):

```bash
preset="llvm-ninja-release-local"
log_prefix="/tmp/1q"

cmake --build --preset "$preset" >"${log_prefix}-review-build.log" 2>&1 || { tail -n 40 "${log_prefix}-review-build.log"; echo "BUILD FAILED"; }
ctest --preset "$preset" --output-on-failure -j 4 >"${log_prefix}-review-test.log" 2>&1 || { tail -n 40 "${log_prefix}-review-test.log"; echo "TEST FAILED"; }
```

Prefer focused testing when the change is module-scoped: run the touched modules' ctest labels first
(`-L unit -R <module>` or named targets), then the full suite. If a code-quality lane modified code,
**re-run Step 1–7 from the new working tree** instead of trusting the previous review.

#### If build fails or tests fail → fix loop

```
🚫 Build/test verification failed.

  Build: FAILED (see /tmp/1q-review-build.log)
  Tests: 3 failures (see /tmp/1q-review-test.log)

Fix the failures, then re-run /completeness-review.
```

**Stop here.** Do not ask about merge.

#### If build and tests pass → proceed to Step 8

## Step 8: Main sync check

Before merging, check whether `main` has moved since the branch was created:

```bash
git fetch origin main
git log --oneline feature/<name>..origin/main | head -5
```

#### If main has new commits

Rebase the feature branch onto the latest main:

```bash
git rebase origin/main
```

If rebase has conflicts, abort and report:

```
🚫 Rebase conflict. The following files have conflicts:
  - path/to/conflicted/file.cpp

Resolve the conflicts manually, then re-run /completeness-review.
```

After a clean rebase, re-run the build from Step 7 to confirm nothing broke.

#### If main is up to date

Proceed to Step 9.

## Step 9: Merge & cleanup

All gates passed, build/tests green, main is current. Before asking to merge, check the branch's
remote state:

```bash
git status -sb                      # ahead/behind vs upstream
git log --oneline @{u}..HEAD 2>/dev/null | head   # unpushed commits (fails if no upstream)
```

- If the branch has **no upstream** (never pushed) or has **unpushed commits**, tell the user the
  merge will delete local history beyond the merge record, and offer to push first:
  `git push -u origin feature/<name>`.
- Only delete the branch after the user confirms, and only if its commits are preserved (merged
  into main and/or pushed to remote).

Declare the phase complete and ask:

> ✅ All checks passed. Correctness verified, build & tests green.
>
> Merge `feature/<name>` into `main` and delete the feature branch?

**If user approves:**

```bash
git checkout main
git merge --no-ff feature/<name> -m "merge: <brief summary>"
git branch -d feature/<name>
```

For an evidence-first change the implementation branch is `feat/<topic>` (not `feature/<name>`). Merge that, then delete **both** process branches (`feat/<topic>` and `evidence/<topic>`). See `evidence-first-freeze-contract` Close-out.

Use `--no-ff` to preserve the feature branch as a distinct history marker. If the branch was pushed to
remote, also run:

```bash
git push origin main
git push origin --delete feature/<name>
```

Do not push unless the user asks.

**After merge, verify the merge result** (the merge itself can introduce resolution errors):

```bash
cmake --build --preset llvm-ninja-release-local >/tmp/1q-review-merge-build.log 2>&1 && \
  ctest --preset llvm-ninja-release-local --output-on-failure -j 4 >/tmp/1q-review-merge-test.log 2>&1
```

If the post-merge build/tests fail, stop and report instead of leaving main broken.

**If user declines:** leave the branch in place for the next session.

This step prevents branch proliferation — every merged feature branch is cleaned up immediately.
