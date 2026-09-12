---
name: adversarial-testing
description: Use when writing or reviewing tests for 1Q core algorithms, physics/signal models, or public API — and specifically when dispatched as the qa-adversary evaluator, which must produce tests from the interface contract alone without reading the implementation. Trigger for 对抗性测试、写测试、补测试、QA、测试集、属性测试、property-based、RapidCheck、不变量、边界/异常路径、反例、漏洞清单、杀死实现、确认偏差、迎合性测试. Also use when auditing whether an existing test could actually fail (assertion quality review) or when a test must be written against a public header whose .cpp the author must not read.
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

### Execution Models & Isolation Levels

1. **Subagent 物理派发（首选）**：在具备子代理能力的宿主中，主 Agent 仅装配接口包（仅包含公开头与文档路径），派发全新的 `qa-adversary` 子代理。子代理拥有全新上下文，指令与工具严格限制触碰 `src/`。
2. **单 Agent 两阶段盲写协议（单会话兜底）**：若只能在单一会话内执行，不得假装具备物理隔离，必须严格遵循两阶段：
   - **Phase 1（盲写阶段）**：只读 `include/` 与 `docs/`，严禁发出任何针对 `src/` 的读取/搜索工具调用。写出测试用例并执行。
   - **Phase 2（缺陷核对阶段）**：测试断言若与当前实现行为不符，**严禁自行查阅 `src/` 调整断言迎合现有代码**。该不符之处即为核心发现，直接记入反例漏洞清单。
3. **诚实性原则（Honesty clause）**：若会话中模型已事先读过实现代码（带有先验记忆），必须在报告头部明示为“降级单元测试（已阅读过实现）”，严禁伪装为严格对抗性测试。

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

### 准入模式（Gate Mode vs. Agile Mode）

- **门禁模式（Gate Mode / Tier 1 Stage A）**：严格执行文档先行。接口包必须**显式引用** `docs/<module>/algorithms.md §…`、`boundaries.md`、`contract.md` 规则编号、`issue_codes.md` 条目。**严禁派发者口头转述**（转述会把实现假设洗白为“需求”）。缺少文档引用的，必须原样退回要求补充来源。
- **敏捷模式（Agile Mode / 日常开发辅助）**：若开发者以自然语言提出测试需求（如“为新加的旋转矩阵补测试”），**Agent 不得生硬拒收**，而应执行**主动溯源（Self-Sourcing）**：主动检索 `docs/<module>/` 与 `include/1q/` 的 Doxygen 契约，向用户反向确认：“已定位需求依据为 `docs/...§X`，将以此作为断言基准”，确认后再行推进。

### 接口包要素

- **Requirement text — with source paths.** 文档章节或 Doxygen 契约规范。
- **Public interface** — the `include/1q/<module>/**` headers, or an exact symbol list.
- **Target domain** — which `tests/<type>/<domain>/` partition the tests belong to.
- **Issue-code vocabulary** — the relevant slice of `docs/common/issue_codes.md` when the API reports failures through issue lists.
- **Out of scope** — behaviours deliberately unconstrained, so you neither assert them nor count them as gaps.

### 规范沉默的处置

若文档与公开头文件确实对某类边界行为（如空输入、越界容错）未作定义，**严禁通过查阅 `src/` 来推测期望**。应将该未定义行为直接列入报告的 `### 未覆盖 / 存疑`，作为待决架构议题（Class D / OQ）交付，而不是私自脑补。

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

### 代数不变量双轨覆盖原则（Dual-Track Principle）

- **全域验证与跨平台底线**：RapidCheck 属性测试（`RC_GTEST_PROP`）负责生成数百组随机输入并收缩出最小反例，是 macOS/Linux/Conan 预设下的高级验证手段。
- **跨平台交付托底（Windows VS2015）**：Windows 交付预设（无 Conan 模式）下属性测试不参与构建。**严禁把核心代数不变量或边界条件完全押在 RapidCheck 上**。
- **强制双轨规则**：每一个用属性测试表达的不变量（如旋转逆复合为单位阵、协方差正定、指标归一化范围），**必须在同模块测试中配有至少一组确定性的 GTest 常量用例（`TEST`）**，显式覆盖典型主轴、零点、边界点。确保在不支持 RapidCheck 的交付构建链上，不变量依然具备确定性的回归防护底线。

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

## Does the suite have teeth? Break it and see.

Nothing above proves the tests would **fail against a wrong implementation**. That is not a gap needing a tool — it is a habit, and for code you just wrote it is the cheapest check available:

> Pick the behaviour you are **least confident** about, deliberately break it in the implementation, run the partition, put it back. If the tests do not complain, they are not testing that behaviour.

You know where your own implementation is fragile; a generic fault injector does not. Two or three hand-picked breakages on the parts you distrust beat a hundred mechanical ones.

**If you do this at scale, read the result asymmetrically:**

| | |
|---|---|
| **A caught fault is not evidence** | it only proves there is no blind spot *at the points you happened to break*; a high number says more about which faults you chose than about the tests |
| **A missed fault is evidence** | it proves no test in the partition distinguishes it — the conclusion is closed |

State which faults you injected whenever you quote a result. Operator swaps (`<` → `<=`, `+` → `-`) are the worst possible choice at scale: they are exactly the dimension a test author covers first, while the defects this repository actually ships are wrong-variable, wrong-unit, wrong-sign and wrong-ordering faults that operators never produce.
## Deliverable

The verdict column is replaced by findings — you do not decide pass/fail.

```markdown
## 对抗性测试报告 — <module / interface>

### 接口包
- 隔离级别：<物理子代理隔离 / 单会话两阶段盲写 / 降级单元测试（已阅读过实现）>
- 需求来源：<文档路径与章节 — 门禁模式为显式引用；敏捷模式为主动溯源确认之路径>
- 接口定义：<include/1q/... headers>
- 防火墙：本次是否适用 Tier 1；若适用，声明未读取 src/ 下任何实现文件（单会话盲写声明 Phase 1 阶段零触碰）
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

### 断点验证（破一次看看）
- 你挑的那处行为：<为什么它是最没把握的>
- 故意改错的方式：<改了哪里、改成什么>
- 结果：<测试报错了吗；没报错说明它没在测这个行为——那本身是发现>

### 实现派生测试（IMPL-DERIVED）
- <仅规格沉默处；说明规格未回答的问题>

### 测试修改披露
- <失败后对测试的每一处修改：对应的失败、修改理由。无修改则写"无">

### 未覆盖 / 存疑
- <规格未定义/沉默的边界行为（转为 Class D / 待决议题 OQ 交付）；或需要真值数据、场景文件、用户裁定才能判定的条目>
```

Severity: **高** = violates a requirement, contract rule, or invariant (including silent degradation and non-determinism). **中** = boundary behaviour undefined or inconsistent with the documentation. **低** = tolerance looser than justified, naming, readability.

Report a finding for every assertion you could not justify, every spec line you could not test, and every place you had to guess. Do not pad the list, and do not drop a finding because it looks embarrassing for the implementation.

## Pre-Submit Self-Check

- [ ] **The defect form was classified, and the instrument matches it** — a process-form defect (trajectory shape, convergence, drift) was not forced into point assertions.
- [ ] 隔离级别已真实披露（物理子代理 / 单会话盲写 / 降级单元测试），未在已读代码的情况下伪装严格对抗。
- [ ] Interface packet cited documents; in agile mode, self-sourcing cited specific sections and confirmed with user. Spec gaps were filed as findings, not filled from implementation.
- [ ] Where the Tier 1 firewall applied, `git diff --stat src/ include/` is empty and no implementation file was read except under the named exception.
- [ ] Any implementation-derived test is marked `// IMPL-DERIVED:` and listed separately.
- [ ] 代数不变量遵循双轨原则：除 RapidCheck 属性测试外，具备确定性 GTest 常量用例（`TEST`）作为跨平台（Windows VS2015 无 Conan）基线托底。
- [ ] Every assertion cites a requirement, doc section, issue code, or mathematical property.
- [ ] Each exception-path category is covered or explained as not applicable.
- [ ] No repository-specific forbidden assertion (Table A) appears.
- [ ] Every randomized input is reproducible: literal seed, `RC_PARAMS` documented, or `reproduce=` recorded.
- [ ] Every floating-point tolerance has a stated derivation.
- [ ] Every post-failure edit to a test is disclosed with its reason.
- [ ] **A fault was deliberately injected into the behaviour you are least confident about, and the tests were checked to notice it** — not left as an assumption that they would.
- [ ] Any process-form judgement was written down **before** the data was produced, with its basis, and its promotion status is stated (observation-only vs promoted to a blocking assertion).
- [ ] Files live at `tests/unit/<domain>/<domain>_<name>_test.cpp` (or the correct type/domain), named per `tests/README.md`; shared helpers in `tests/support/`.
- [ ] `flight_dynamic` is the exception to glob discovery — a new file there must be added to `tests/cmake/FlightDynamicPartitions.cmake` explicitly.
