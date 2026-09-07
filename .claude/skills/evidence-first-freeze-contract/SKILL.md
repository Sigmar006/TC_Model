---
name: evidence-first-freeze-contract
description: Use when asked to plan, review, freeze, or implement high-risk algorithm or architecture changes where evidence must precede implementation（先证明再实现）. Trigger for 冻结项、证据矩阵、证据矩阵交付、Stage A 文档、evidence 分支、裁定、冻结契约、分级门禁、Tier 1/Tier 2、SAR/ESR/EOS/RIR/SBIRS 等模块的高风险改动、test-threshold or acceptance-bar changes、risky refactors、design gates、merging evidence/feat branches into main、deleting process branches、or cases where rejecting implementation is an acceptable outcome（否决也是有效交付）.
---

# Evidence-First Freeze Contract

Use this skill to prevent "implement first, justify later" behavior in high-risk technical work. The core rule is: **Stage A builds an evidence matrix; Stage B implementation is allowed only if the evidence proves the need and scope. Evidence rejection is a valid final deliverable（否决也是交付）.**

This skill is the executable workflow for `docs/common/contract.md` section "证据优先开发模式"（含强制规则 6 的分级门禁）. The contract defines the mandatory gate and its tiers; this skill defines how to run them.

The workflow follows a fixed lifecycle:
- Stage A delivers a dated review document initialized by the generator script and committed on a new `evidence/<topic>` process branch.
- After the user rules on the proposed decisions, implementation proceeds on a `feat/<topic>` branch forked from it.
- Stage C writes durable conclusions into the authority docs named in the writeback checklist below, appends the run record, and verifies the checklist.
- At close-out the process scaffold document is removed on the feature branch before `--no-ff` merging into `main`, and both process branches (`feat/<topic>` and `evidence/<topic>`) are deleted; full history stays retrievable via git.

## Operating Rule & Tiered Gates

Do not start implementation during Stage A unless the user explicitly overrides the contract. If asked to "start implementing" a high-risk item without evidence, run Stage A first and report the gate result.

### 分级门禁（镜像 contract.md 强制规则 6；范围归属存疑时一律按 Tier 1 处理，就高不就低）

- **Tier 1（全流程门禁）——强制范围**：
  - algorithms, physics models, signal/image processing, control logic, or maneuver logic;
  - architecture refactors, module-internal responsibility changes, or dependency cleanup;
  - output semantics, trace/replay/schema behavior, lifecycle/debug boundaries, or config semantics;
  - any public `include/1q` API, module facade, builder, validator, or cross-domain contract change;
  - test-threshold changes, skipped tests, known-limit splits, or acceptance-bar changes.
  - 交付物：`evidence/<topic>` 分支 + 脚本初始化的评审文档 + 冻结契约 + `feat/<topic>` 分支。
- **Tier 2（轻量内联门禁）——仅允许零公共接口、零行为边界、零 schema、零测试阈值变更的模块内部小型改动（如私有辅助函数的局部整理）**：
  - 允许单分支执行；不开独立评审文档，在提交信息内嵌迷你证据块（假设 Hypothesis / 探针结果 Probe result / 允许范围 Allowed scope）。
  - 证据仍然先于实现；超出此边界的任何一项即升级为 Tier 1。
- **无门禁（排除项）**：
  - mechanical formatting;
  - typo-only comments or documentation wording that does not change behavior;
  - small local fixes where the failing behavior and acceptance condition are already explicit in the user request.

For every Tier 1 freeze item, answer four questions before editing production code:

1. What exact requirement, risk, or failure mode is being frozen?
2. What evidence would prove it?
3. What evidence would disprove it?
4. What is the smallest allowed change if proven?

## Workflow Summary

1. **Stage A（证据矩阵）**：prove, reject, narrow, or defer the requirement; deliver the matrix as a script-initialized document on a new `evidence/<topic>` branch.
2. **Discussion（用户裁定）**：the user rules on proposed decisions; rulings land in the document as numbered 修订记录 entries.
3. **Contract Freeze（冻结契约）**：append the frozen contract to §3 of the same document before touching any production code.
4. **Stage B（范围内实现）**：fork `feat/<topic>` from `evidence/<topic>`——fork 前须先经计划模式提交实现计划并获用户批准（见 Stage B Plan-approval gate）——then implement only within the frozen scope.
5. **Stage C（验证与权威回写）**：run verifications, fulfill the 强制回写清单 below, and append the run record to §4 of the scaffold document.
6. **Close-out（拆脚手架与合并）**：once the user asks to merge, remove the scaffold document on `feat/<topic>`, merge `--no-ff` into `main`, delete both process branches. Do not push unless asked.

## Stage A: Evidence Matrix

Whenever this workflow applies, the matrix is delivered as a repo document — never chat-only. The chat reply carries only the decision summary plus a link to the file. Chat-only output is acceptable only for items under 无门禁（排除项）.

### Deliverable form

1. Initialize the document with the script; never hand-write the skeleton — the script is the single source of the format, which is what prevents drift:
   `python .claude/skills/evidence-first-freeze-contract/scripts/gen_stage_a_doc.py <topic>`
2. Path: `docs/review/<topic>_stage_a_<YYYY-MM-DD>.md`, `<topic>` kebab-case. The script fills `Date` and `Review-Baseline`（branch @ HEAD）and refuses to overwrite an existing file.
3. One file grows by stage: `Status` moves draft → frozen（契约已附）→ final（运行记录已附）.
4. All-reject and all-defer matrices are also committed — rejection is a valid deliverable.

### Writing rules（whole document, enforced in every section）

1、证据一律写成一行：`- **证据**：[evidence: 路径]`；可加 `::符号名`；禁止行号——行号会过期、无法维护。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、禁止"见契约规则 N"式裸引用；直接写出规则内容，再用证据形式锁定来源（如 `- **证据**：[evidence: docs/common/contract.md]`）。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的，写明结果；无法直接验证的判断以"推理："开头标注。

### Matrix columns

The script-generated table headers are the plain-Chinese form below (English shown for column meaning):

| 待裁定项 (Freeze item) | 假设（要证明什么）(Hypothesis) | 证据来源 (Evidence source) | 探针/测试（已执行）(Probe/Test) | 通过条件 (Pass criterion) | 否定条件 (Rejection criterion) | 建议判定 (Decision) |
|-------------|------------|-----------------|------------|----------------|--------------------|----------|

- `待裁定项`: the decision under review — a need/risk question（"这个需求是否成立"）, not an implementation idea or a feasibility question.
- `假设`: the claim that must be true before implementation.
- `证据来源`: code paths, tests, traces, docs, schemas, physics constraints, or user requirements.
- `探针/测试`: concrete inspection, experiment, or focused test command — 已实际执行并写明结果.
- `通过条件`: objective condition that permits Stage B.
- `否定条件`: objective condition that stops, narrows, or defers implementation.
- `建议判定`: `pass`, `reject`, `narrow`, or `defer` — 最终以用户裁定为准.

### Delivery constraints

When the matrix is delivered (before the user closes the discussion):

1、§3 冻结契约与 §4 运行记录保持脚本占位——不写冻结契约、不写字段级实现规格、不写 Stage C 内容。
2、矩阵里的判定是建议（建议判定）；用户裁定落成编号修订条目，禁止静默改写。
3、待裁定项必须是"需求/风险是否成立"的决定；实现可行性问题不是合法的待裁定项。

## Evidence Quality

Prefer direct evidence in this order:

1. Existing failing behavior, trace divergence, reproducible test, or concrete acceptance requirement.
2. Source-level contract mismatch, schema/API inconsistency, or documented physical/system constraint.
3. Focused experiment or small characterization test.
4. Reasoned inference from code structure, explicitly labeled with the document's `推理：` prefix. For example: "推理：this helper is a composition-root leak because the only caller is session assembly and it pulls pipeline internals into config validation."

Avoid treating broad intuition, aesthetic preference, or "future flexibility" as sufficient evidence for risky implementation.

## Stage A Outcomes

Stage A may end in any of these outcomes:

- `pass`: enter Stage B with a narrow implementation boundary.
- `reject`: do not implement; record why the requirement was not proven. A rejected implementation is not failure; it preserves engineering quality.
- `narrow`: implement only the proven subset.
- `defer`: identify missing evidence and the next probe needed.

## Branch and Commit Flow

The flow produces two clearly separated commit phases, each on its own branch.

1. Evidence branch — created when the Stage A matrix is delivered.
   - Branch: `evidence/<topic>`.
   - Commits: the script-initialized matrix document, discussion revisions, and the frozen contract (§3). Example commit: `docs(review): stage-a evidence matrix for <topic>`.
   - Production code never lands here. If Stage A ends in all `reject`/`defer`, this branch with the matrix commit is the final deliverable and no implementation branch is created.
2. Implementation branch — created only after the user closes the discussion and the contract is frozen.
   - Branch: `feat/<topic>`, forked from `evidence/<topic>` so merged history reads: matrix commit(s) → implementation commit(s).
   - All Stage B/C code, test, and authority-doc writeback commits land here, following the `docs-governance-standard` commit shapes (one module per commit; docs commits separate from code commits).

Rules:

1. Never mix matrix-document edits and production-code edits in one commit.
2. **Pivot Protocol（实现推翻契约时的转向流程）**：if Stage B implementation disproves a frozen hypothesis or hits an unresolvable boundary:
   1、先留证：把反证测试/探针作为可复现的失败用例提交或暂存在 `feat/<topic>` 上；
   2、回矩阵：切回 `evidence/<topic>`，追加编号修订条目记录新反证，并调整建议判定（如 pass → narrow / reject）；
   3、重裁定再重建：用户重新裁定、冻结更新后的 §3 契约之后，基于新基线重建（rebase 或重建）`feat/<topic>`。禁止在已失效的契约上继续补丁式改代码。

### Close-out: scaffold removal, merge, and branch cleanup

`docs/review/<topic>_stage_a_<date>.md` is a **process scaffold（过程脚手架）**: it exists to facilitate review and contract freezing; it is not a durable documentation destination. Completing the 强制回写清单 in Stage C is the prerequisite for dismantling it.

After Stage C verification passes and the writeback checklist is complete:

1. Remove the scaffold on the feature branch:
   ```bash
   git rm docs/review/<topic>_stage_a_<date>.md
   git commit -m "docs: clean up process scaffold for <topic>"
   ```
2. Merge to main and delete process branches (only upon user instruction):
   ```bash
   git checkout main
   git merge --no-ff feat/<topic> -m "merge: <brief summary>"
   git branch -d feat/<topic>
   git branch -d evidence/<topic>
   ```

- Use `--no-ff` so the feat history stays a distinct merge commit. Delete **both** process branches; after a normal fork `evidence/<topic>` is an ancestor of `feat/<topic>`, so both are fully contained in `main` and `git branch -d` should succeed.
- Do not push `main` or delete remote branches unless the user asks.
- Why this balance works: `main` stays clean（评审目录不无限堆积过期文档）; durable knowledge lives in the authority docs; the complete review history and evidence matrix stay retrievable via git history（`git log -- docs/review/<topic>_stage_a_<date>.md` 或合并提交）.
- If Stage A ended in all `reject`/`defer`（无 feat 分支）: first record the negative decisions and open questions in the authority docs（否决记录 + 开放议题，见回写清单）, then merge `evidence/<topic>` into `main` only if the user explicitly asks for the record; otherwise leave the branch local.

## Contract Freeze（§3 冻结契约）

Only create an implementation contract when Stage A has at least one `pass` or `narrow` decision. The contract must be written before production-code edits, appended as §3 of the stage document (the skeleton reserves the section), committed on the evidence branch; record the user's closing ruling as a 修订记录 entry.

Use this template:

```markdown
## §3 冻结契约（用户讨论结束后填写）

已证明的需求：
- ...

允许范围：
- 模块/目录：...
- 类/函数：...
- 测试/文档：...

明确禁止范围：
- 公共头：...
- 跨模块类型：...
- schema/trace/replay：...
- 测试阈值/skip：...
- 兼容层：...

行为边界：
- 输入：...
- 输出：...
- 错误/回退：...
- 生命周期/调试/trace：...

爆炸半径与回滚：
- 下游消费方影响：...
- 回退难度：无损 / 破坏性 / 回滚注意点

验收门：
- 构建：...
- 聚焦测试：...
- 契约测试：...
- 特征化测试：...
- 探针转正：Stage A 探针中转为正式测试的清单

非目标：
- ...
```

Rules:

1. If implementation needs a file, module, public header, schema, or behavior not named in the contract, stop and return to Stage A.
2. If a smaller implementation satisfies the proven requirement, choose the smaller implementation.
3. Do not add speculative abstractions, compatibility layers, cross-module generalization, or public API surface unless the matrix proved they are needed.
4. For this repo's current state, assume `include/1q` is mostly stable; prefer `src/<module>/` internal design cleanup unless evidence proves the public contract itself is wrong.

## Stage B: Implementation Gate

**Plan-approval gate (mandatory, 2026-09-03)**: after the contract is frozen and before writing any production code, enter plan mode（计划模式） and present the implementation plan for user approval; only after the user approves the plan may you fork `feat/<topic>` and edit code. The plan itself must state: the proven requirement, files/modules in scope and out of scope, implementation steps, commit split, and validation commands. If implementation hits a boundary not covered by the approved plan, stop and return to planning.

Before editing code, fork the implementation branch `feat/<topic>` from `evidence/<topic>` (only after the user closes the discussion and approves the plan), then state:

- the proven requirement;
- the files or modules in scope;
- the files or modules explicitly out of scope;
- the validation commands that will prove the change.

Implementation rules:

- Fix real model, coordinate, state, config, data-flow, or boundary problems before touching tests.
- Do not weaken thresholds, broaden skips, or mark behavior unstable unless Stage A proved the original acceptance criterion was wrong.
- Add an abstraction only when it removes proven complexity, duplication, or an incorrect dependency boundary.
- Keep cross-domain symmetry meaningful: copy shape only when responsibilities match.
- Keep code, tests, and the relevant `docs/*/design.md` boundary text synchronized when behavior changes.
- Prefer focused tests first, then contract tests, then broader CI labels appropriate to the touched module.
- **Modular milestone cadence（模块化分步交付与汇报）**：跨多个业务模块的 Stage B 实现，禁止在单轮自治循环中连续执行所有模块直至终局验收。每完成一个模块的代码修改与局部构建/测试验证，必须主动结束当轮工具调用，输出该模块的完成进展与下一步计划，由用户触发或确认后再进入下一个模块。

## Stage C: Verification and Authority Writeback

Stage C closes the loop: run verifications, write durable conclusions into the authority docs that own them, append the run record. The matrix document is a working log, not the destination of record. **Fulfilling the 强制回写清单 is the strict prerequisite for deleting the process scaffold and merging.**

### 强制回写清单（Mandatory Authority Writeback Checklist）

Every completed Tier 1 item must check off all four:

1、**正向边界（设计/行为结论）** → 所属模块的 `docs/<module>/` 文档集（`design.md` / `boundaries.md` / `data-flow.md` / `algorithms.md`，按归属选择）：验证过的功能边界、数据流修订、输入输出契约、明确排除的范围。
2、**否决记录** → `docs/<module>/design.md` 的"架构裁定与否决记录"专节（体裁定义见 `docs-governance-standard`）：Stage A 判 `reject` 的项在此留档——提议了什么、为何被否、否决证据，防止将来重复提议同一被否方案。
3、**开放议题登记** → `docs/common/open_questions.md`：`defer` 项和被收窄掉的范围登记条目并拿到编号（条目格式遵循 `open-questions-doc-standard`），闭合跟踪环。
4、**证据锁** → 每条新增/修订的规则/边界后面附 `- **证据**：[evidence: 路径]`（可 `::符号名`），指向永久代码/测试锚点；禁止行号。

跨模块规则、会话语义、issue code 类结论 → `docs/common/contract.md` / `docs/common/session_contract.md` / `docs/common/issue_codes.md`。

Durability test: if a future reader who was not part of this task needs the conclusion, it belongs in an authority doc, not only in git history. Follow the `docs-governance-standard` Stage-4 writeback commit shape; docs and code stay in separate commits.

### Probe graduation（探针转正）

Passing Stage A probes that validated core mathematical or physical assumptions must not remain throwaway terminal commands:

- Any probe with regression-detection value graduates into a permanent contract or unit test during Stage B/C.
- Lock authority-doc evidence pointers to these graduated test cases.

### Run record（§4 运行记录）

Before removing the scaffold in the close-out phase, append the run record to §4 of `docs/review/<topic>_stage_a_<date>.md` on `feat/<topic>`:

```markdown
## §4 运行记录（Stage C 后填写）

1、实现范围：...
2、验证命令与结果：`command`: pass/fail（含转正的探针测试）
3、权威回写去向：
   1、正向边界：<文件路径>
   2、否决记录：<文件路径>（否决项与理由）
   3、开放议题：docs/common/open_questions.md（编号 ...）
   4、证据锁：[evidence: <测试或代码锚点>]
4、残留风险：...
5、后续冻结项：...
```

If tests fail, do not weaken thresholds or widen skips unless the evidence matrix proves the original acceptance criterion was wrong.

## Recommended Output Shape

For planning/review-only tasks (chat reply):

1. Decision summary（§2 判定汇总）plus a link to `docs/review/<topic>_stage_a_<date>.md` on branch `evidence/<topic>`.
2. Decision per freeze item.
3. Frozen contract or Stage B scope only for passed/narrowed items.
4. Rejected/deferred items with next evidence needed.

For implementation tasks (on branch `feat/<topic>`):

1. Brief Stage A result with a link to the matrix document.
2. Frozen contract summary.
3. Stage B changes.
4. Verification commands and results (including graduated probe tests).
5. Authority-writeback checklist summary mapping each conclusion to its destination authority doc.
6. Remaining freeze items and residual risks.
