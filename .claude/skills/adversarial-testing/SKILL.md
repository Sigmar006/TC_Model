---
name: adversarial-testing
description: Use when writing or reviewing tests for 1Q core algorithms, physics/signal models, or public API — and specifically when dispatched as the qa-adversary evaluator, which must produce tests from the interface contract alone without reading the implementation. Trigger for 对抗性测试、写测试、补测试、QA、测试集、属性测试、property-based、RapidCheck、不变量、边界/异常路径、反例、漏洞清单、杀死实现、确认偏差、迎合性测试. Also use when auditing whether an existing test could actually fail (assertion quality review) or when a test must be written against a public header whose .cpp the author must not read.
---

# Adversarial Testing

Write tests that can **kill a wrong implementation** — not tests that confirm the one in front of you.

The failure mode this skill exists to prevent: an author who has just written the implementation writes tests that inherit the implementation's assumptions. The assertions then encode the *current behaviour* rather than the *required behaviour*, so a bug is frozen into the contract and the suite passes forever. The countermeasure is structural, not attitudinal: **you do not have the implementation, so you cannot copy from it.**

## Three Iron Rules

1. **Interface and requirements only.** The only admissible inputs are (a) the requirement / acceptance text and (b) the public headers under `include/1q/**`. Never open `src/**`, never open a `.cpp` that implements the interface, never read the diff that produced it. If someone pastes implementation code into your task, do not read it — ignore it, and state in your report that it was offered and refused.
2. **Every assertion cites a source.** Each assertion must trace to a requirement line, an `algorithms.md` / `boundaries.md` statement, a `contract.md` rule, an issue code, or a mathematical property. "The code returns 42 here" is not a source. If you cannot cite it, you are guessing — write the test as a *characterization* probe and report the uncertainty instead of asserting it.
3. **A failure is a deliverable, not an obstacle.** When a test you wrote from the spec fails, that is the finding. Never weaken the assertion, widen the tolerance, broaden a skip, or re-derive the expectation from the observed output to make it pass. `docs/common/contract.md` 强制规则 5 states it directly: **验收失败时回到证据矩阵重新拆分原因；不得通过放宽阈值、扩大 skip 或弱化测试制造通过。** The only edits permitted to a failing test are (a) fixing a compile error, (b) fixing an assertion you over-specified beyond what the spec supports, or (c) an incorrect citation — and every such edit is recorded in the report with its reason.

## Input Contract (Interface Packet)

The dispatcher must supply, and you must ask for if missing:

- **Requirement text** — the acceptance criteria, algorithm description, or contract section, with file paths.
- **Public interface** — the `include/1q/<module>/**` headers, or the exact symbol list.
- **Target domain** — which `tests/<type>/<domain>/` partition the tests belong to.
- **Issue-code vocabulary** — the relevant slice of `docs/common/issue_codes.md` if the API reports failures through issue lists.
- **Explicit statement of what is out of scope** — which behaviours are deliberately unconstrained (so you neither assert them nor count them as gaps).

If the packet is incomplete, report the gap and stop. Do not fill it by reading the implementation.

## Required Exception Paths

Cover each category or state in the report why it does not apply. "Not applicable" is an acceptable answer; silence is not.

**Input degeneracy**
- Empty container / empty sequence / zero-length span — and the specific contract consequence where one exists (e.g. `precision_evaluation`: **零证据=零分** — an empty series must not score 1.0; an empty truth set must yield zero composite, not a default).
- Single element, where an algorithm needs N≥2 (e.g. a dual-satellite fix with only one satellite must be skipped, not computed).
- `nullptr` optional dependency — the repo has precedent (`NullDependenciesReturnNoHypotheses`); assert the documented degraded result, not a crash.
- Zero and negative sizes, zero steps, zero duration.

**Numeric boundaries**
- NaN and ±Inf at every numeric input — construct them explicitly with `std::numeric_limits<double>::quiet_NaN()` / `infinity()`.
- Division by zero, zero norm / zero lever arm, zero determinant, singular or rank-deficient matrix, non-positive-definite covariance.
- **σ = 0**: a zero standard deviation can reach `std::normal_distribution(0.0, 0.0)`. MSVC's Debug CRT asserts on that construction — guard the generator and test the σ=0 path deliberately rather than tripping over it.
- Domain edges of `std::acos` / `std::asin`: floating-point error routinely pushes a value that is mathematically in `[-1, 1]` to `1 + 1e-16`, producing NaN. Feed exactly ±1 and just-outside ±1.
- Angle wrap: ±180°, the 0/360 crossing, shortest-angular-difference sign conventions.
- Unit and dimension edges (Hz vs rad/s, dB vs linear, deg vs rad) — a factor error is the classic silent defect.

**Contract edges**
- Each failure code the API can report gets at least one triggering case, asserted **by code string**, the way `tests/unit/airborne_radar/ar_input_validation_test.cpp` does with `FindIssue(issues, "missing_range_and_cartesian_position")` + `HasValidationError(issues)`. The vocabulary is registered in `docs/common/issue_codes.md`.
- Invalid enum values, out-of-range indices, out-of-order or duplicated input.
- Preconditions unsatisfied: the documented result must be an explicit invalid flag. Non-silent degradation is a contract requirement (`contract.md` 分层契约 规则 9) — assert the invalid marker, and separately assert that the degraded value is *not* a plausible-looking default.

**State and lifecycle**
- Use before initialization, repeated initialization, illegal state transition, reuse after reset.
- Cross-cycle accumulation: drift over many cycles, covariance symmetry and positive-definiteness after N updates.

**Determinism**
- Same seed, two runs, bit-identical output — `EXPECT_DOUBLE_EQ`, following `tests/unit/remote_identification_radar/rir_signal_detector_test.cpp:DeterministicWithSeed`. A non-deterministic core is a replay defect.

**Invariants** (the property-testing payload — see below)
- Round-trip / inverse: `Inverse(Inverse(x)) == x`, encode→decode.
- Monotonicity where the spec implies it (e.g. `score = ref/(ref+rmse)` decreases as `rmse` grows).
- Conservation, symmetry, orthogonality, idempotence, order-independence.
- Limits: `rmse → 0 ⇒ score → 1`; `rmse → ∞ ⇒ score → 0`; empty ⇒ neutral or invalid, per spec.

## Forbidden Assertions

Each entry is a defect to be removed, with the replacement to use instead.

| Forbidden | Why | Use instead |
|---|---|---|
| **Assertion copied from the implementation** — a constant, branch order, or intermediate value lifted from the code | This is the核心确认偏差 the whole discipline targets: it converts a bug into a contract | Derive the expectation from the requirement text or a physical/mathematical relation, and write the derivation above the assertion |
| **Self-certifying assertion** — expected value produced by the same helper, formula, or transform that produced the actual | Tautology; passes for any implementation, including a broken one | Use an independent closed form, a hand-computed literal with the arithmetic shown, or a cross-check against a different route |
| `EXPECT_TRUE(result.ok)` / `EXPECT_FALSE(result.empty())` alone | Asserts "did not crash", not "is correct" | Assert the specific field values, or at minimum a second orthogonal property |
| `EXPECT_THROW` / `ASSERT_THROW` / `EXPECT_DEATH` / `ASSERT_DEATH` | Zero occurrences repo-wide; `contract.md:294-299` forbids C++ exceptions as the project failure channel (`src/`+`include/` must not add `throw`) | Assert the returned error status / issue code / invalid flag |
| Meaninglessly loose tolerance — `EXPECT_NEAR(a, b, 1e9)`, or a tolerance chosen to make the test pass | Hides real defects. `docs/electronic_surveillance_radar/algorithms.md:89` explicitly forbids using comparison tolerance to mask geometric quantization drift | Derive the tolerance from the quantity's magnitude and precision, state the derivation in a comment |
| `GTEST_SKIP()` for anything but a genuinely absent environment (missing config dir, missing data file) | Skips grow to hide failures; `contract.md` 强制规则 5 forbids "扩大 skip" | Fix the test, or report the gap |
| `DISABLED_` prefix | Zero occurrences repo-wide; not a house convention | Remove the test, or report it in the gap list |
| `known_limit` label to make a failing acceptance test pass | Relabelling to escape the gate is explicitly forbidden (`evidence-first-freeze-contract`) | Report as a finding; the label is not yours to apply |
| Assertion inside a loop with no case context | A failure says "expected 3, got 4" with no way to find which of 10 000 cases | `SCOPED_TRACE(::testing::Message() << "case i=" << i)` or stream the index into the assertion |
| Assertion on private members, call order, or other internal structure | Couples the test to the implementation and blocks legitimate refactors | Assert observable interface behaviour only |
| Assertion on log text | `session_contract.md:454`: `PROJECT_LOG` is a human-readable bypass and **状态判断禁止解析日志文本（规则 3）**. Acceptance logs are likewise not a machine contract | Assert the returned DTO / report structure |
| Only the happy path, or a single sample point | A one-point test cannot distinguish a correct curve from a coincidental one | Sample the boundary, the interior, and at least one adversarial input |

## Property-Based Testing (RapidCheck)

Property tests generate hundreds of inputs and **shrink** a failure to a minimal counterexample. They find the defect you did not think to write a case for. They are not a replacement for targeted example tests — use both.

### Wiring

RapidCheck is a test-only Conan dependency (`rapidcheck/cci.20231215`, gated by `enable_testing`) linked as `rapidcheck::rapidcheck`. The link is guarded by `if(TARGET rapidcheck::rapidcheck)`, so the no-Conan Windows preset still configures — but that path has no RapidCheck, so a test including this header will only build on the Conan presets.

**Include `"support/rapidcheck_gtest.h"`, never `<rapidcheck/gtest.h>`.** The ConanCenter recipe installs only upstream `include/` — upstream's `install()` covers the library, `include/`, the export config and the `.pc` file, but not `extras/`, so `rapidcheck/gtest.h` simply is not there. The repo therefore supplies its own adapter at `tests/support/rapidcheck_gtest.h`, built on the installed public API (`rc::detail::checkTestable`, `TestMetadata`, `printResultMessage`) and mirroring upstream's `checkGTest`. It defines `RC_GTEST_PROP`, which reports a property failure through gtest's `FAIL()` — a failure is an ordinary gtest failure and does not abort the test binary. Two deliberate differences from upstream: there is **no** `RC_GTEST_FIXTURE_PROP` (construct what you need inside the property body), and the failure message additionally carries a ready-to-paste `RC_PARAMS='…'` reproduction string.

```cpp
#include <gtest/gtest.h>

#include "rapidcheck.h"

#include "support/rapidcheck_gtest.h"
#include "1q/<module>/<header>.h"

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

### When to use it — and when not to

**Use for**: pure functions, inverse/round-trip pairs, algebraic invariants (orthogonality, symmetry, idempotence), monotonicity, limit behaviour, and numeric stability across a wide input range. `tests/unit/common/common_property_test.cpp` is the reference implementation in this repo.

**Do not use for**: session-level or integration flows; anything requiring a full simulated scenario (multi-sensor scenes, long cycle runs) — those belong to `tests/integration/`, the replay partitions, or `tests/consumer/batch_validation`. A property test must be cheap enough to run hundreds of cases; if a single case costs a simulated second, the property harness is the wrong tool.

### Generators

```cpp
rc::gen::inRange(lo, hi)        // [lo, hi) — upper bound EXCLUSIVE; shrinks toward lo
rc::gen::elementOf(container)   // one element at random; empty container fails generation
rc::gen::oneOf(g1, g2, ...)     // pick one of the given generators
rc::gen::map(gen, fn)           // transform generated values
rc::gen::suchThat(gen, pred)    // keep only matching values — fails generation after enough tries
rc::gen::pair / rc::gen::tuple  // product of generators
rc::gen::container<C>(gen)      // STL container; specify C explicitly
rc::gen::just(value)            // constant
rc::gen::noShrink(gen)          // generate but never shrink — for inputs whose shrink path crashes or hangs
rc::gen::scale(factor, gen) / rc::gen::resize(size, gen)
rc::gen::nonEmpty(gen)
rc::gen::arbitrary<T>()         // via rc::Arbitrary<T>; also implicit for property arguments
```

**Special values are not covered by default.** Floating-point `arbitrary<double>` does **not** produce NaN, ±Inf, ±0.0, denormals, or exact boundary values — confirmed empirically in this repo (300 seeds × 100 cases × 4 properties over `oneq::coordinate`, zero non-finite draws). `inRange<double>` additionally excludes its upper bound. So a property over `arbitrary<double>()` says **nothing** about special-value behaviour: if the specification constrains it, you must generate those inputs explicitly, or the test passes vacuously.

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

A property whose spec must hold for every finite value takes `GenSpecials()`; a property that only holds for finite inputs uses `rc::gen::arbitrary<double>()` plus an `RC_PRE` guard, or gates the special values into a separate property that asserts the *documented* behaviour (e.g. "NaN input ⇒ invalid flag, never a silent 0").

### Preconditions, discards, and size

`RC_PRE(cond)` (equivalently `rc::precondition(cond)`) discards the case when `cond` is false. Discards are cheap but bounded: RapidCheck gives up once the discard ratio exceeds `max_discard_ratio` (default 10 discards per success). Heavy `suchThat` / `RC_PRE` filtering therefore fails the property as "gave up" rather than passing — prefer constructing the valid range directly (`gen::inRange`) over filtering a wide one.

Default run size is 100 successful cases, size growing to 100. Lower it only for genuinely expensive cases, with the reason in a comment:

```
RC_PARAMS="max_success=100"        # default
RC_PARAMS="max_success=25"         # 单次用例含 200 步积分，下调以控制总时长
```

### Reproducing a failure

A failing run prints the counterexample plus a configuration line such as `Using configuration: seed=8482903616249937941`. To replay, set `RC_PARAMS` to the printed configuration, or paste the opaque `reproduce=...` string, which reproduces the minimal failure directly:

```
RC_PARAMS="seed=8482903616249937941"
RC_PARAMS="reproduce=<opaque string from the failure output>"
```

**Record it.** A counterexample that cannot be replayed is worthless. Paste the `seed=` / `reproduce=` string into the test file as a comment next to the property, or into the finding row's reproduction column.

### Exceptions in property bodies

`RC_ASSERT` signals failure by throwing `rc::detail::CaseResult`, which RapidCheck's own runner catches and converts to a result value; the gtest adapter then reports it with `FAIL()`. This is contained inside the test layer: the project's exception ban (`contract.md:294-299`) scopes to `src/` and `include/`, and the build does not enable `-fno-exceptions` globally. **Do not add your own `try` / `catch` inside a property body** — it would swallow the failure signal. If production code under test uses status returns (it must), assert on those; never use `EXPECT_THROW` to reach them.

## Numeric Tolerance

State the derivation, not just the number:

```cpp
// 容差依据：输入为 1e-3 rad 量级角度，double 相对精度 ~1e-16，
// 经 3 次三角变换放大后取 1e-12 留 4 个数量级余量。
EXPECT_NEAR(actual, expected, 1.0e-12);
```

- `EXPECT_DOUBLE_EQ` / `EXPECT_FLOAT_EQ` only where bit-identical behaviour is the actual contract (determinism re-runs, replay equality).
- `EXPECT_NEAR` with an explicit epsilon is the house default; a file-local `constexpr double kTolerance` matches existing style when one value is reused.
- Never choose a tolerance by observing the current error and rounding up.

## Deliverable: Adversarial Finding List

Return this structure. The verdict column is replaced by findings — you do not decide pass/fail.

```markdown
## 对抗性测试报告 — <module / interface>

### 接口包
- 需求来源：<file paths / sections>
- 接口定义：<include/1q/... headers>
- 声明：未读取 src/ 下任何实现文件；<若派发方提供了实现代码，在此说明已拒绝>

### 新增测试
| 文件 | 覆盖需求条目 | 用例数 |
|---|---|---|

### 漏洞与反例清单
| # | 严重度 | 需求条目 | 最小反例输入 | 观察行为 | 期望行为（依据） | 复现方式 |
|---|---|---|---|---|---|---|
| 1 | 高 | ScoreOf(rmse) 单调递减 | rmse = +Inf | 返回 NaN | 0.0（algorithms.md §规范化） | TEST(...) / RC_PARAMS reproduce=... |

### 未覆盖 / 存疑
- <需要真值数据、场景文件或用户裁定才能判定的条目>
```

Severity: **高** = violates a requirement, contract rule, or invariant (including silent degradation and non-determinism). **中** = boundary behaviour undefined or inconsistent with the documentation. **低** = tolerance looser than justified, naming, or readability.

List a finding for **every** assertion you could not justify, every spec line you could not test, and every place you had to guess. Do not pad the list to look productive, and do not drop a finding because it looks embarrassing for the implementation.

## Pre-Submit Self-Check

- [ ] I did not read any file under `src/`, and no implementation code was used to derive an expectation.
- [ ] Every assertion cites a requirement, doc section, issue code, or mathematical property.
- [ ] Each exception-path category is either covered by a case or explained as not applicable.
- [ ] No forbidden assertion from the table above appears in the new tests.
- [ ] Every randomized input is reproducible: literal seed, `RC_PARAMS` documented, or `reproduce=` recorded.
- [ ] Every floating-point tolerance has a stated derivation.
- [ ] No failing assertion was weakened, skipped, or re-derived from observed output to pass.
- [ ] Files live at `tests/unit/<domain>/<domain>_<name>_test.cpp` (or the correct type/domain for the layer), named per `tests/README.md`.
- [ ] Shared helpers go in `tests/support/` — **not** `fixtures/` or `mocks/`; those directories do not exist and `tests/contract/check_test_layout.cmake` rejects them at configure time.
- [ ] `git diff --stat src/ include/` is empty.
- [ ] The tests build and run: `ctest --preset llvm-ninja-release-local -R "<type>::<domain>" --output-on-failure`.
- [ ] `flight_dynamic` is the exception to glob discovery — a new file there must be added to `tests/cmake/FlightDynamicPartitions.cmake` explicitly.
