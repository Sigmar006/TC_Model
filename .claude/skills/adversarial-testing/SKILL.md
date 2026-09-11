---
name: adversarial-testing
description: Use when writing or reviewing tests for 1Q core algorithms, physics/signal models, or public API — and specifically when dispatched as the qa-adversary evaluator, which must produce tests from the interface contract alone without reading the implementation. Trigger for 对抗性测试、写测试、补测试、QA、测试集、属性测试、property-based、RapidCheck、不变量、边界/异常路径、反例、漏洞清单、杀死实现、确认偏差、迎合性测试、变异测试、mutation. Also use when auditing whether an existing test could actually fail (assertion quality review) or when a test must be written against a public header whose .cpp the author must not read.
---

# Adversarial Testing

Write tests that can **kill a wrong implementation** — not tests that confirm the one in front of you.

The failure mode: an author who has just written the implementation writes tests that inherit its assumptions. Assertions then encode the *current behaviour* rather than the *required behaviour*, so a bug is frozen into the contract and the suite stays green forever.

## What actually does the work

Three mechanisms prevent this. They cost very different amounts, so they are not bundled into one rule:

| Mechanism | Cost | When |
|---|---|---|
| **Fresh context** — the evaluator did not write the implementation and carries none of its reasoning | free, and the largest single source of benefit | always |
| **Specification-first** — derive expectations from the requirement text, not from observed output | free | always |
| **Implementation firewall** — do not read `src/` | expensive: it also blocks the questions the spec leaves unanswered | high-risk surfaces only (below) |

Most of the benefit comes from the first two. A fresh context that has never seen the implementation does not need a firewall to avoid inheriting its assumptions. The firewall buys an additional *guarantee*, and the price is real: when the specification is silent on a behaviour a test must pin, the blocked question becomes an untested behaviour.

**When the firewall applies**: where `docs/common/contract.md` 强制规则 6 的 Tier 1 门禁 applies — public API and facades under `include/1q/`, algorithms/physics/signal/control logic, output semantics, and anything whose failure is silent (numerics, scoring, state machines). For everything else, rely on fresh context and specification-first and read whatever you need. In this 1Q repository, the qa-adversary subagent will typically be dispatched *because* the target is Tier 1.

### The named exception

When the specification is silent on a behaviour a test must pin — an unspecified input domain, an undefined failure mode — you may read the implementation **for that question only**, and you must then:

- mark the test with a `// IMPL-DERIVED:` comment naming the question the spec did not answer;
- list it under a separate **Implementation-derived tests** heading in the report, never in the adversarial findings;
- not count it as evidence of correctness. It documents current behaviour; it does not verify required behaviour.

An undeclared implementation-derived test is the violation, not the reading.

## Honesty over self-restraint

You cannot be trusted about your own rigor, and neither can any author. So this workflow does not ask you to promise — it asks you to disclose, and it verifies mechanically:

- **`git diff --stat src/ include/` must be empty** when you finish. Mechanical, checkable, and stated at the top of your report.
- **Every edit to a test after it failed must be disclosed** with the failure it answered and the reason. Editing a test after a failure is *permitted* — the judge of whether it was legitimate is a reader, not a rule. Fixing a compile error, correcting an assertion you over-specified beyond what the spec supports, and fixing a wrong citation are all fine. What is never fine is an undisclosed edit, or one whose stated reason is "it was failing".
- **The mutation score** (below) measures whether the tests have teeth, independently of anything you say about them.

If a test you wrote from the spec fails, that is the finding. Do not weaken the assertion, widen the tolerance, broaden a skip, or re-derive the expectation from observed output. `docs/common/contract.md` 强制规则 5: **验收失败时回到证据矩阵重新拆分原因；不得通过放宽阈值、扩大 skip 或弱化测试制造通过。**

## Input Contract (Interface Packet)

Ask for what is missing; do not fill a gap by reading the implementation.

- **Requirement text — with source paths.** The packet must *cite* `docs/<module>/algorithms.md §…`, `boundaries.md`, `contract.md` rule numbers, `issue_codes.md` entries. **A paraphrase authored by the dispatcher is not acceptable.** An interface packet that restates the requirements in the dispatcher's own words launders implementation assumptions into the "specification" and defeats the isolation while looking compliant. If the packet contains a prose summary with no document citations, send it back and ask for the sources.
- **Public interface** — the `include/1q/<module>/**` headers, or an exact symbol list.
- **Target domain** — which `tests/<type>/<domain>/` partition the tests belong to.
- **Issue-code vocabulary** — the relevant slice of `docs/common/issue_codes.md` when the API reports failures through issue lists.
- **Out of scope** — behaviours deliberately unconstrained, so you neither assert them nor count them as gaps.

## Required Exception Paths

Cover each category, or state in the report why it does not apply. "Not applicable" is an acceptable answer; silence is not.

**Input degeneracy** — empty container/sequence/zero-length span; single element where the algorithm needs N≥2; `nullptr` optional dependency; zero and negative sizes and steps.

**Numeric boundaries** — NaN and ±Inf constructed explicitly (`std::numeric_limits<double>::quiet_NaN()`); division by zero, zero norm, singular/rank-deficient matrix, non-positive-definite covariance. **σ = 0**: a zero standard deviation reaches `std::normal_distribution(0.0, 0.0)`, on which MSVC's Debug CRT asserts — guard the generator and test the σ=0 path deliberately. **Domain edges of `acos`/`asin`**: floating-point error routinely pushes a mathematically in-range value to `1 + 1e-16` and produces NaN. Angle wrap at ±180° and the 0/360 crossing. Unit edges (Hz vs rad/s, dB vs linear).

**Contract edges** — every failure code the API can report gets at least one triggering case, asserted **by code string** (`FindIssue(issues, "...")` + `HasValidationError(issues)`, as in `tests/unit/airborne_radar/ar_input_validation_test.cpp`; vocabulary in `docs/common/issue_codes.md`). Invalid enums, out-of-range indices, duplicated input. Preconditions unsatisfied must yield an explicit invalid flag — non-silent degradation is a contract requirement (contract.md 分层契约 规则 9); also assert the degraded value is *not* a plausible-looking default.

**State and lifecycle** — use before initialization, repeated initialization, illegal transition, reuse after reset. Cross-cycle drift; covariance symmetry and positive-definiteness after N updates.

**Determinism** — same seed, two runs, bit-identical (`EXPECT_DOUBLE_EQ`), following `tests/unit/remote_identification_radar/rir_signal_detector_test.cpp:DeterministicWithSeed`.

**Invariants** — the property-test payload: round-trips, monotonicity, conservation, symmetry, orthogonality, idempotence, and limits (`rmse→0 ⇒ score→1`, `rmse→∞ ⇒ score→0`, empty ⇒ neutral or invalid per spec).

## Forbidden Assertions

### A. Repository-specific — knowledge you cannot derive

These encode *this* repository's contracts and history. Getting them wrong is not a judgment call.

| Forbidden | Why | Use instead |
|---|---|---|
| `EXPECT_THROW` / `ASSERT_THROW` / `EXPECT_DEATH` / `ASSERT_DEATH` | Zero occurrences repo-wide; `contract.md:294-299` forbids C++ exceptions as the project failure channel | Assert the returned error status / issue code / invalid flag |
| `DISABLED_` prefix | Zero occurrences repo-wide; not a house convention | Remove the test, or report it in the gap list |
| `known_limit` label to make a failing acceptance test pass | Relabelling to escape the gate is forbidden (`evidence-first-freeze-contract`) | Report as a finding; the label is not yours to apply |
| `GTEST_SKIP()` for anything but a genuinely absent environment | Skips grow to hide failures; 强制规则 5 forbids "扩大 skip" | Fix the test, or report the gap |
| Asserting on log text | `session_contract.md:454`: `PROJECT_LOG` is a human-readable bypass and **状态判断禁止解析日志文本（规则 3）**; acceptance logs are not a machine contract | Assert the returned DTO / report structure |
| Shared helpers in `tests/fixtures/` or `tests/mocks/` | Those directories do not exist and `tests/contract/check_test_layout.cmake` rejects them at configure time | `tests/support/` |

### B. Generic — one line each, because you already know them

An expectation derived from the implementation or from a shared helper rather than from the requirement; a tautology that holds for any implementation; `EXPECT_TRUE(ok)` as the whole assertion; a tolerance chosen so the test passes; an assertion inside a loop with no `SCOPED_TRACE`; assertions on private members or call order; happy-path-only coverage; a single sample point where a curve is the contract.

The common failure is the first one — an expectation copied from the code. Derive it from the requirement text, a physical relation, or a hand computation whose arithmetic you show above the assertion.

## Property-Based Testing (RapidCheck)

Property tests generate hundreds of inputs and **shrink** a failure to a minimal counterexample. They find the defect you did not think to write a case for. They do not replace targeted example tests — use both.

### Wiring

Test-only Conan dependency (`rapidcheck/cci.20231215`, gated by `enable_testing`), linked as `rapidcheck::rapidcheck`. The link is guarded by `if(TARGET rapidcheck::rapidcheck)`, so the no-Conan Windows preset still configures — but that path has no RapidCheck, so a test including the adapter header only builds on the Conan presets.

**Include `"support/rapidcheck_gtest.h"`, never `<rapidcheck/gtest.h>`.** The ConanCenter recipe installs only upstream `include/`; upstream's `install()` covers the library, `include/`, the export config and the `.pc` file, but not `extras/`, so `rapidcheck/gtest.h` is simply not there. The repo supplies its own adapter at `tests/support/rapidcheck_gtest.h`, built on the installed public API (`rc::detail::checkTestable`, `TestMetadata`, `printResultMessage`) and mirroring upstream's `checkGTest`. It defines `RC_GTEST_PROP`, which reports a property failure through gtest's `FAIL()` — an ordinary gtest failure that does not abort the binary. Two deliberate differences from upstream: there is **no** `RC_GTEST_FIXTURE_PROP` (construct what you need inside the property body), and the failure message additionally carries a ready-to-paste `RC_PARAMS='…'` reproduction string.

```cpp
#include <gtest/gtest.h>

#include "rapidcheck.h"

#include "1q/<module>/<header>.h"
#include "support/rapidcheck_gtest.h"

namespace <module> {
namespace {

// 属性：任意姿态角构造的矩阵与其逆复合必为单位阵。
// 依据：attitude_transform.h 声明 RotationMatrix3d 为"三维旋转矩阵"。
RC_GTEST_PROP(CoordinateAttitudePropertyTest, MatrixComposedWithInverseIsIdentity,
              (double yaw_deg, double pitch_deg, double roll_deg)) {
  const RotationMatrix3d rotation =
      BuildRotationMatrix(EulerAnglesDeg(yaw_deg, pitch_deg, roll_deg));
  // 容差依据必须写出来，不能靠试。
  RC_ASSERT(MaxAbsDifference(Compose(rotation, Inverse(rotation)), IdentityMatrix()) < 1.0e-12);
}

}  // namespace
}  // namespace <module>
```

`tests/unit/common/common_property_test.cpp` is the reference implementation in this repo.

### When to use it — and when not to

**Use for** pure functions, inverse/round-trip pairs, algebraic invariants, monotonicity, limit behaviour, numeric stability over a wide input range.

**Do not use for** session-level or integration flows, or anything needing a full simulated scenario — those belong to `tests/integration/`, the replay partitions, or `tests/consumer/batch_validation`. A property test must be cheap enough to run hundreds of cases; if one case costs a simulated second, this is the wrong tool.

### Generators

```cpp
rc::gen::inRange(lo, hi)        // [lo, hi) — upper bound EXCLUSIVE; shrinks toward lo
rc::gen::elementOf(container)   // one element at random; empty container fails generation
rc::gen::oneOf(g1, g2, ...)     // pick one of the given generators
rc::gen::map(gen, fn)           // transform generated values
rc::gen::suchThat(gen, pred)    // keep only matching values — fails generation after enough tries
rc::gen::pair / rc::gen::tuple  // product of generators
rc::gen::container<C>(gen)      // STL container; specify C explicitly
rc::gen::just(value) / rc::gen::noShrink(gen)
rc::gen::scale(factor, gen) / rc::gen::resize(size, gen)
rc::gen::arbitrary<T>()         // via rc::Arbitrary<T>; also implicit for property arguments
```

**Special values are not covered by default.** Floating-point `arbitrary<double>` does **not** produce NaN, ±Inf, ±0.0, denormals, or exact boundary values — confirmed empirically here (300 seeds × 100 cases × 4 properties over `oneq::coordinate`, zero non-finite draws). `inRange<double>` additionally excludes its upper bound. A property over `arbitrary<double>()` therefore says **nothing** about special-value behaviour; generate them explicitly or the test passes vacuously:

```cpp
const std::vector<double> kSpecials = {
    0.0, -0.0, std::numeric_limits<double>::min(),
    std::numeric_limits<double>::denorm_min(), 1.0, -1.0,
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()};

auto GenSpecials() {
  return rc::gen::oneOf(rc::gen::arbitrary<double>(), rc::gen::elementOf(kSpecials));
}
```

**Preconditions.** `RC_PRE(cond)` discards the case when false. Discards are bounded — RapidCheck gives up once the ratio exceeds `max_discard_ratio` (default 10 per success), reporting "Gave up after N tests" rather than passing. Prefer constructing the valid range directly (`gen::inRange`, a `fmod` mapping) over filtering a wide one; over-filtering is a generator defect, not a property failure.

**Size.** Default 100 successful cases, size growing to 100. Lower it only for genuinely expensive cases, with the reason in a comment: `RC_PARAMS="max_success=25"`.

### Reproducing a failure

A failing run prints the counterexample plus a configuration line such as `Using configuration: seed=8482903616249937941`, and the adapter's failure message carries `RC_PARAMS='seed=…'` directly. To replay:

```
RC_PARAMS="seed=8482903616249937941"
RC_PARAMS="reproduce=<opaque string from the failure output>"
```

Record it: paste the string into the test file as a comment next to the property, or into the finding row. A counterexample that cannot be replayed is worthless.

### Exceptions in property bodies

`RC_ASSERT` signals failure by throwing `rc::detail::CaseResult`, which RapidCheck's runner catches and converts to a result value; the adapter then reports it with `FAIL()`. This is contained in the test layer: the project's exception ban (`contract.md:294-299`) scopes to `src/` and `include/`, and the build does not enable `-fno-exceptions` globally. **Do not add your own `try`/`catch` inside a property body** — it would swallow the failure signal.

## Mutation Probe — does the suite have teeth?

Everything above is about writing better tests. None of it proves the tests would **fail against a wrong implementation**; that was previously a self-report. `scripts/mutation_probe.py` replaces the self-report with a measurement: it applies small semantic mutations to a source file, rebuilds, runs the owning partition, and records whether the tests caught each one.

```bash
# enumerate first — cheap, shows the mutation points and their lines
scripts/mutation_probe.py --source src/<module>/<file>.cpp \
    --target 1q_<domain>_unit_tests --test unit::<domain> --list

# measure (bound it while iterating)
scripts/mutation_probe.py --source src/<module>/<file>.cpp \
    --target 1q_<domain>_unit_tests --test unit::<domain> \
    --operators rel_lt,rel_le,rel_eq --limit 20
```

| Result | Meaning |
|---|---|
| `KILLED` | tests failed under the mutant — the suite has a detector for this behaviour |
| `SURVIVED` | tests passed — **a blind spot, and a finding**; report the line, the mutation, and the source line |
| `BUILD ERROR` | mutant did not compile (stillborn) — excluded from the score |

Exit status is 1 when any mutant survived, 0 otherwise, so it can gate. The probe refuses to run on a file with uncommitted changes (it restores via the original text and verifies the sha256), and restores on Ctrl-C.

**Reading the result.** The mutation score is a *diagnostic*, not a target — do not chase a number, and never weaken the probe (removing operators, mutating a copy, excluding survivors) to raise it. Work the other direction: each survivor names a behaviour no test pins. Either add the test that kills it, or report it as an accepted gap with the reason the behaviour is untested.

**Limits to state in the report.** Text-level mutation, one point at a time, on space-delimited operators only (so `std::vector<double>` and `<<` are never touched); it cannot see mutants that need multiple coordinated edits; and it only covers the file you point it at.

## Deliverable

The verdict column is replaced by findings — you do not decide pass/fail.

```markdown
## 对抗性测试报告 — <module / interface>

### 接口包
- 需求来源：<文档路径与章节 — 必须是引用，不是转述>
- 接口定义：<include/1q/... headers>
- 防火墙：本次是否适用 Tier 1；若适用，声明未读取 src/ 下任何实现文件
- `git diff --stat src/ include/`：<输出，应为空>

### 新增测试
| 文件 | 覆盖需求条目 | 用例数 |

### 漏洞与反例清单
| # | 严重度 | 需求条目 | 最小反例输入 | 观察行为 | 期望行为（依据） | 复现方式 |

### 变异探针
- 命令：
- 结果：killed N / survived M / build-error K，score X%
- survivors：<行号、变异、源码行>（每个 survivor 要么补上杀它的测试，要么说明为何接受该缺口）

### 实现派生测试（IMPL-DERIVED）
- <仅规格沉默处；说明规格未回答的问题>

### 测试修改披露
- <失败后对测试的每一处修改：对应的失败、修改理由。无修改则写"无">

### 未覆盖 / 存疑
- <需要真值数据、场景文件或用户裁定才能判定的条目>
```

Severity: **高** = violates a requirement, contract rule, or invariant (including silent degradation and non-determinism). **中** = boundary behaviour undefined or inconsistent with the documentation. **低** = tolerance looser than justified, naming, readability.

Report a finding for every assertion you could not justify, every spec line you could not test, and every place you had to guess. Do not pad the list, and do not drop a finding because it looks embarrassing for the implementation.

## Pre-Submit Self-Check

- [ ] The interface packet cited documents; it was not a paraphrase. Gaps were sent back, not filled from the implementation.
- [ ] Where the Tier 1 firewall applied, `git diff --stat src/ include/` is empty and no implementation file was read except under the named exception.
- [ ] Any implementation-derived test is marked `// IMPL-DERIVED:` and listed separately.
- [ ] Every assertion cites a requirement, doc section, issue code, or mathematical property.
- [ ] Each exception-path category is covered or explained as not applicable.
- [ ] No repository-specific forbidden assertion (Table A) appears.
- [ ] Every randomized input is reproducible: literal seed, `RC_PARAMS` documented, or `reproduce=` recorded.
- [ ] Every floating-point tolerance has a stated derivation.
- [ ] Every post-failure edit to a test is disclosed with its reason.
- [ ] The mutation probe ran on the touched surface, and every survivor is either killed by a new test or reported as an accepted gap.
- [ ] Files live at `tests/unit/<domain>/<domain>_<name>_test.cpp` (or the correct type/domain), named per `tests/README.md`; shared helpers in `tests/support/`.
- [ ] `flight_dynamic` is the exception to glob discovery — a new file there must be added to `tests/cmake/FlightDynamicPartitions.cmake` explicitly.
