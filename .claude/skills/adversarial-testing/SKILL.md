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

## Choosing the Verification Instrument

**Classify the defect form before writing anything.** The commonest authoring failure is reaching for the instrument you know instead of the one the defect needs. This repository has **three verification lanes** — GoogleTest, export-and-analyze, and batch validation — plus characterization as a GoogleTest *sub-form* rather than a lane of its own. Only one of the three is GoogleTest.

| 缺陷形态 | 工具 | 载体 | 交付物 |
|---|---|---|---|
| 某个输入上的错误取值 | GTest 常量用例 | `tests/<type>/<domain>/` | 触发输入 + 期望/实际 |
| 必须对所有输入成立的不变量 | 属性测试（RapidCheck） | 同上 | 收缩后的最小反例 |
| 边界 / 退化 / 非法输入 | GTest 常量用例 | 同上 | 触发输入 |
| **过程形态：轨迹形状、收敛、漂移、振荡、几何关系** | **逐帧导出 + 脚本/图表分析** | `fd_tools/` 式工具（随测试编译，**不注册 ctest**）或 `tools/*.py` | **判据 + 序列 + 结论** |
| 跨组件端到端场景 | batch validation | `tests/consumer/batch_validation/` | 场景级指标 |
| 已测行为的度量与下限 | characterization 测试 | `*_characterization_test.cpp` + `RecordProperty` | 测量值 + 物理下限 |

**过程形态是这里唯一没有"最小反例"的一类**：每个采样点都可能正确，而过程仍然是错的。收缩、单点反例、点条件异常路径对它全都不适用。六自由度轨迹、滤波器长时间收敛、扫描图案的周期一致性、图像质量随参数退化的趋势，都属于这一类。`tests/unit/flight_dynamic/fd_tools/README.md` 是制度化先例，它把这条决策写得比本节更早："六自由度轨迹的'对不对'很难用单断言表达（GTest 适合确定性不变式，不适合轨迹形状/收敛过程判断）"。SAR 走同一条路（导出 raw 图像 + `tools/analyze_sar_psf.py`）。

新建同类工具时照 `fd_tools` 的形态：独立 `main`、随测试构建、**不注册 ctest**。它可以使用 `src/` 内部头（开发期验证的合法需求，FD 的工具就读取起落架/推进器状态），这不代表消费方集成方式。

### 过程形态的判据：先声明，后测量

判据（收敛时间、超调量、p95 误差、半径一致性…）通常是一条工程期望，未必写在任何文档里。三条规则：

1. **判据必须在跑数据之前写下来**，连同依据（物理推导 / 装备指标 / 既有文档），放在工具或分析记录的头部。看到曲线之后再定阈值，等于用观察到的行为反推期望——同一种确认偏差，只是发生在连续域，而且更难发现，因为曲线"看起来对"。
2. **观察层不阻断，因此不触发阈值门禁**：导出分析的结果供人判断，不写硬断言。
3. **升格才是门禁事件**：一旦判据写成 `EXPECT_*` 断言、batch validation 的阻断阈值，或文档验收条款，它就成了仓库规则意义上的 test-threshold / acceptance-bar，按 `contract.md` 强制规则 6a 走全流程门禁（证据分支 + Stage A 证据矩阵）。**触发门禁的是"升格"这个动作，不是"第一次写判据"**——判据写在分析工具里不需要证据文档，把它变成阻断断言才需要。

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

**Do not use for** session-level or integration flows, anything needing a full simulated scenario (→ `tests/integration/`, the replay partitions, or `tests/consumer/batch_validation`), or **process-form behaviour** — trajectory shape, convergence, drift, oscillation (→ the export-and-analyze lane in "Choosing the Verification Instrument"). A property test must be cheap enough to run hundreds of cases; if one case costs a simulated second, this is the wrong tool, and widening the input range will not rescue it.

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

Everything above is about writing better tests. None of it proves the tests would **fail against a wrong implementation**; that was previously a self-report. `scripts/mutation_probe.py` replaces the self-report with a measurement.

**Read the result asymmetrically, because the evidence is asymmetric:**

- **A survivor is evidence.** It proves that no test in the partition distinguishes that mutant. The conclusion is closed.
- **A kill is not.** It proves only that there is no blind spot *at the points the tool mutated*. A high score therefore says as much about the operator set as about the tests.

This is not a hypothetical caveat. This repository's actual `fix` history is dominated by wrong-variable, wrong-unit, wrong-sign, missing-update, wrong-ordering and wrong-formula defects — and the textual operator set (space-delimited ` < `, ` + `, ` && `, `0.0` …) produces **none** of those. What it produces is operator swaps, which are exactly what a test author covers first. So a 100% score is *not* evidence that the suite is strong; it is frequently evidence that the probe only poked where the tests already looked. **Never report a bare score — report the operator or seed set that produced it.**

### Two seeding modes

**Operator mode** — mutate a file you name:

```bash
scripts/mutation_probe.py --source src/<module>/<file>.cpp \
    --target 1q_<domain>_unit_tests --test unit::<domain> --list   # enumerate first
scripts/mutation_probe.py --source src/<module>/<file>.cpp \
    --target 1q_<domain>_unit_tests --test unit::<domain> \
    --operators rel_lt,rel_le,rel_eq --limit 20
```

**History mode** — seed with defects this repository actually shipped:

```bash
scripts/mutation_probe.py --history [--include-multi] [--history-limit 60]
```

It takes `fix` commits touching `src/`/`include/`, reverses them with `git apply -R` — the seed is the exact pre-fix state, not a reconstruction — rebuilds, and runs the owning partition. The question is strictly stronger than operator mode's:

> 不是"测试能否区分我构造的变异"，而是"测试能否抓住我们实际犯过的错"。

Nothing is interpreted, so no selection bias enters on the part of whoever chose the seeds. Commits whose context has drifted can no longer be reversed and are reported as **unreachable**; that set only ever grows, so run history mode before the seeds decay.

**Triage every MISSED seed — not every `fix:` commit is a behavioural defect.** The seed set includes comment and log-string rewrites, warning fixes, layering changes (dropping a cross-layer include) and compiler-compat shims. Reversing those is *supposed* to go unnoticed; counting them as gaps inflates the finding list with noise and destroys the value of the mode. The tool prints each seed's subject and diffstat to make the call quick, and deliberately prints no score — the denominator contains non-defects, so a percentage would be a number nobody should act on. Only a seed that reverts **control flow, a value, a validation or an update order** and still passes is a real finding.

**Module attribution is by path.** A fix in a shared file whose behaviour is consumed elsewhere (e.g. a change under `src/common/` made for an ESR-facing defect) will be tested against `unit::common`, which may not be where the behaviour is pinned. If a MISSED seed looks behavioural but the partition seems wrong, rerun it with an explicit `--test`/`--target` before recording it.

**Marked commits.** How many defects live in one commit cannot be decided mechanically — a commit touching two files may be one defect spread across a header and its implementation, or several unrelated fixes. The tool does not guess: it runs single-file commits by default and **marks multi-file ones for separate review**, listing their paths. Check each by hand; use `--include-multi` once you know it is one defect.

| Result | Meaning |
|---|---|
| `KILLED` / `CAUGHT` | tests failed under the mutant — a detector for this behaviour exists |
| `SURVIVED` / `MISSED` | tests passed — **classify before acting** (below) |
| `BUILD ERROR` | mutant did not compile (stillborn) — excluded. A mutant that fails to compile only because the *test* is over-specified is not stillborn in any meaningful sense; check the build output before dismissing it |
| `UNREACHABLE` | history mode: the fix no longer applies backwards (context drifted) |
| `MARKED` | history mode: several files in one commit — review separately |

Exit status is 1 when anything survived or was missed. The probe refuses to run when the target has uncommitted changes, restores the original (sha256-verified in operator mode, `git checkout` plus a cleanliness check in history mode), and restores on Ctrl-C.

**Classify every survivor before acting on it.** The kill criterion is *the unit tests*, so a survivor does not automatically mean "a test is missing":

| survivor 的性质 | 判断 | 处置 |
|---|---|---|
| **点值缺失** | 该行为该由单元断言判定，但没写 | 补单元测试 |
| **不变量缺失** | 该行为该由属性判定，但没写 | 补属性测试 |
| **等价变异体** | 变异前后语义相同，没有任何测试能区分 | 就地记录，不补测试（补不出）。例：`min(n-1, ceil(0.95n)-1)` 里 `n-1` 恒不绑定 |
| **过程形态** | 该行为的正确性本来就不由单元测试判定 | **不补单元测试**——移交导出分析层或 batch validation，并在报告里注明"非单元测试管辖" |
| **历史缺陷 MISSED** | 仓库实际发过的 bug，今天仍测不出 | 优先级最高——它证明的不是"某处没测"，而是"这类错我们犯过，且仍然抓不住" |

把过程形态的 survivor 当成测试缺口，会逼出一个假的点式测试——它锁定不了真正的问题，却看起来合规。等价变异体不补测试，也不是"接受的缺口"，而是**探针的已知盲区**。

**Never turn the score into a gate.** A proxy metric that blocks a merge gets optimised rather than used — the same reason `docs/practice/ci.md` is cautious about coverage thresholds.

**Limits to state in the report.** Text-level, one point at a time, space-delimited operators only (`std::vector<double>` and `<<` are never touched). Operator mode does **not** produce wrong-variable, wrong-unit, wrong-sign, reordered-update or wrong-formula mutants. History mode sees only fixes whose context has not drifted, and cannot attribute a result when a commit holds several defects.

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

### 漏洞与反例清单（点式）
| # | 严重度 | 需求条目 | 最小反例输入 | 观察行为 | 期望行为（依据） | 复现方式 |

### 过程形态分析（若适用）
- 缺陷形态判定：<为何属过程形态而非点式——轨迹形状/收敛/漂移/振荡/几何>
- 判据（**声明时间与依据**；须先于跑数据写下）：
- 序列产物：<CSV/raw 路径 + 生成命令>
- 结论：<是否满足判据；不满足时的具体偏离>
- 升格状态：<未升格（仅观察，不阻断）/ 已升格为阻断断言，已走 evidence-first 门禁>

### 变异探针
- 模式与种子集：<算子模式：算子清单 / 历史模式：候选 N、可反向应用 M、标记待查 K、不可达 J>
- 命令：
- 结果：killed/caught N，survived/missed M，build-error K
- **证据边界（必填）**：本结果只覆盖 <算子集 / 历史种子集>；未覆盖 <错误变量、单位、符号、更新顺序、公式> 等类别。**不得只报分数**——高分反映的是种子集，不是测试强度
- survivors / MISSED：<逐条：行号或提交号、变异或缺陷、**分类**（点值缺失 / 不变量缺失 / 等价变异体 / 过程形态 / 历史缺陷）+ 处置>
- 标记待查的提交：<逐条列出并说明单独复核结论>

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

- [ ] **The defect form was classified, and the instrument matches it** — a process-form defect (trajectory shape, convergence, drift) was not forced into point assertions.
- [ ] Interface packet cited documents; it was not a paraphrase. Gaps were sent back, not filled from the implementation.
- [ ] Where the Tier 1 firewall applied, `git diff --stat src/ include/` is empty and no implementation file was read except under the named exception.
- [ ] Any implementation-derived test is marked `// IMPL-DERIVED:` and listed separately.
- [ ] Every assertion cites a requirement, doc section, issue code, or mathematical property.
- [ ] Each exception-path category is covered or explained as not applicable.
- [ ] No repository-specific forbidden assertion (Table A) appears.
- [ ] Every randomized input is reproducible: literal seed, `RC_PARAMS` documented, or `reproduce=` recorded.
- [ ] Every floating-point tolerance has a stated derivation.
- [ ] Every post-failure edit to a test is disclosed with its reason.
- [ ] The mutation probe ran on the touched surface, and every survivor is **classified** (点值缺失 / 不变量缺失 / 等价变异体 / 过程形态 / 历史缺陷) with the disposition that classification calls for — not reflexively turned into a unit test.
- [ ] The report states the **seed/operator set** the probe result came from, and does not present a bare score as evidence of test strength.
- [ ] Any process-form judgement was written down **before** the data was produced, with its basis, and its promotion status is stated (observation-only vs promoted to a blocking assertion).
- [ ] Files live at `tests/unit/<domain>/<domain>_<name>_test.cpp` (or the correct type/domain), named per `tests/README.md`; shared helpers in `tests/support/`.
- [ ] `flight_dynamic` is the exception to glob discovery — a new file there must be added to `tests/cmake/FlightDynamicPartitions.cmake` explicitly.
