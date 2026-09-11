---
name: docs-governance-standard
description: Use when planning, writing, or reviewing cross-module contract changes and their documentation in the 1Q repo — "契约收敛"、"规则 14"、"冻结契约"、"docs 回写"、"Stage C"、"对齐状态表"、"审查文档"、"docs/review"、"DES-1/BND/Q/RED 编号"、跨模块统一/对齐/收敛、以及这类工作的提交信息。Governs the closed loop: freeze contract → per-module implementation → docs writeback → review doc → fix writeback, plus the writing standards for contract.md / session_contract.md / docs/review/ and their commit messages. Also use when auditing whether a module change left its docs (boundaries/data-flow/algorithms/contract) in sync.
---

# Docs Governance Standard

Define the **writing standards and closed-loop workflow** for cross-module contract
changes in this repo, distilled from the rule-14 unification branch
(`feature/docs-20260807-093902`, 42 commits: freeze → five-module implementation →
Stage C writeback → COMMON-OQ-9 registration/convergence → review doc → fix writeback).

The repo treats `docs/common/contract.md`, `docs/common/session_contract.md`, and the
per-module `design.md`/`boundaries.md`/`data-flow.md`/`algorithms.md` doc sets as the
design authority. A contract change is not done when code compiles — it is done when
the docs writeback and review loop close. This skill makes that loop reproducible.

## When to use

- Freezing or amending a cross-module contract rule (e.g. "统一问题列表模型", "三写",
  "电源单源"), or extending a rule to a new domain (cycle → config).
- Implementing a frozen contract across multiple modules and writing the docs back
  (alignment table, boundaries sections, evidence pointers).
- Registering or converging an open question (`COMMON-OQ-*`) — decide whether it must
  migrate out of `open_questions.md`.
- Writing, updating, or reviewing a review doc under `docs/review/` (findings with
  DES/BND/Q/RED classification, severity, evidence, fix status).
- Composing commit messages for any of the above (Conventional Commits with module scope).
- Auditing a change for doc-code drift: docs claim one thing, code does another.

## When not to use

- Writing a single module's design detail → the module's own `design.md` set.
- Writing an open-question entry itself → `open-questions-doc-standard` skill (this
  skill references it; do not duplicate its 5-field template here).
- High-risk algorithm/architecture gate process → `evidence-first-freeze-contract`.
- Module contract-gap audit / repair → `evidence-first-freeze-contract`.
- Test/coverage strategy → `test-coverage-strategy`; commit gate → `completeness-review`.

## The closed loop (7 stages)

A cross-module contract change follows this order. Each stage is a separate commit
(or small commit group); docs stages are never skipped or merged into code stages.

| # | Stage | Commit shape | Example (rule 14 branch) |
|---|---|---|---|
| 1 | **Freeze** | `docs(common): freeze ...` — contract rule written first, code untouched | `dac83834` |
| 2 | **Per-module implementation** | one commit per module, scope = module | `2c07ccc0`(ESR), `61eca816`(EOS), `86a71fc3`(SBIRS), `bbfa2a74`(AR), `30b0052b`(SAR) |
| 3 | **Test lock** | `test(<module>):` assert the contract (phase/code/roundtrip) | `9fe56112`, `29cf7d48` |
| 4 | **Stage C writeback** | `docs(common): mark ... alignment complete` — alignment table all-aligned, evidence pointers refreshed, module boundaries rewritten | `015db4a6` |
| 5 | **Open-question register/converge** | `docs(common): add COMMON-OQ-N` when divergences surface; `docs: COMMON-OQ-N 收敛回写` when settled — migrate OUT of open_questions.md, harden into contract/boundaries | `17b3025a` → `a7809026` |
| 6 | **Review doc** | `docs(<scope>): 审查文档落库` — findings report under `docs/review/` | `2a0bd3da` |
| 7 | **Fix writeback** | `docs(common): 审查文档补修复状态...` — annotate the review doc with fix commits per finding | `408f523a` |

Detailed per-stage checklists: read `references/workflow.md` before starting a stage.

## Writing density (可扫描性)

Docs in this repo are read by both humans and reviewers; long prose paragraphs hide
facts. Follow these rules:

- **One semantic point per bullet/item.** A paragraph that stacks 3+ independent
  facts (format baseline + reference implementations + alignment status) must be
  split into a list or table.
- **Rules ≤ ~5 lines each.** A `规则 N` item that grows beyond ~5 rendered lines
  needs sub-items (`a/b/c/d`) or a table; reference-implementation lists belong in
  a list/table, not inline prose.
- **Evidence list is a list.** Multiple `[evidence: ...]` lines go one per line
  (already the repo style); never comma-join them into one line.
- **No paragraph > 8 rendered lines.** Continuous prose blocks longer than 8 lines
  in contract/session_contract/boundaries/review docs are a defect — restructure
  into bullets, tables, or sub-headings. (Automated scan: see the check below.)
- **Interface trees as bullet lists.** Directory/symbol inventories (e.g.
  `src/<module>/` layout) are bullet lists or tables, not prose runs.

Quick self-check before committing a docs change:

```bash
# 连续正文（非标题/列表/表格/代码块）超过 8 行的段落即违规
python3 - <<'EOF'
import os, re
for root, _, files in os.walk('docs'):
    for f in files:
        if not f.endswith('.md'): continue
        p = os.path.join(root, f)
        lines = open(p).readlines()
        start, n, in_code = None, 0, False
        for i, ln in enumerate(lines):
            s = ln.strip()
            if s.startswith('```'):
                in_code = not in_code
                if in_code: start, n = None, 0
                continue
            if in_code: continue
            if (not s) or s.startswith(('#','-','*','|','>')) or re.match(r'^(\d+|[a-z])\.\s', s):
                if n > 8 and start: print(f"{p}:{start+1} 大段({n}行)")
                start, n = None, 0
            else:
                if start is None: start = i
                n += 1
EOF
```

## Document writing standards

Each doc file has a fixed skeleton. Read the matching reference before writing:

- `docs/common/contract.md` / `docs/common/session_contract.md` — rule numbering
  (`规则 N`, sub-items `a/b/c/d`), `[evidence: ...]` pointers, alignment-status tables,
  scope framing (who must obey), frozen code/`snake_case` literal formats.
  → `references/contract-docs.md`
- `docs/review/*.md` — frontmatter (`Status`/`Date`/`Review-Baseline`/`Authority`),
  §0 conclusion + machine verification, §1 method, §2 findings (DES/BND/Q/RED tables
  with 编号/严重度/发现/证据 columns), §3 priority, §4 conclusion, and the
  **后续状态 block** appended after fixes land (finding → commit mapping, 暂缓项).
  → `references/review-doc.md`
- `docs/<module>/design.md` 的"架构裁定与否决记录"专节（Architecture Decisions &
  Rejection Log）— 由 `evidence-first-freeze-contract` Stage C 强制回写清单产生：
  Stage A 判 `reject` 的冻结项在此留档。每条写明提议内容、否决原因、否决证据
  （`- **证据**：[evidence: 路径]`，可 `::符号名`，禁行号）；一条一个条目，编号或
  分小节，不写大段叙述。模块此前无此节时在 design.md 末尾新建；已收敛开放议题的
  结论若为否决，迁回写时同落此节。
- Commit messages for this work — `type(scope): description`, imperative, module
  scopes, docs commits carry a body listing what changed where.

For `open_questions.md` itself, follow `open-questions-doc-standard` — this skill only
decides *when* an entry is born (Stage 5 divergence) and *when* it must leave.

## Core principles (why the loop exists)

1. **Contract before code.** Freeze first (`docs(common)` commit), implement second.
   Code-only changes that skip the freeze produce drift the review stage will catch.
2. **Docs are written by the same change that obsoletes them.** Stage C writeback is
   not optional cleanup; it is what keeps `boundaries.md`/`data-flow.md` truthful.
3. **One module per commit.** Cross-module work splits by module so each commit is
   buildable, testable, and reviewable in isolation.
4. **Settled open questions leave, they are not archived.** Convergence writeback
   moves the rule into contract/boundaries and deletes the entry (see
   `open-questions-doc-standard`).
5. **Review docs record evidence, not opinions.** Every finding cites
   `file:line`; the 后续状态 block maps each finding to the commit that fixed it,
   and records 暂缓项 (explicitly deferred) separately.
6. **Every PROJECT_LOG call site keeps the two-line Chinese annotation**
   (`// 中译：…` + `// 标识：…`) when docs work touches logging code.

## Verification before commit

- The chosen preset builds; relevant ctest passes (record it in the docs commit body
  when the commit is a writeback, e.g. `015db4a6`).
- Grep the whole repo for retired symbols (old fields, old code prefixes) — review
  stage cross-checks with independent agents; do it yourself first.
- `[evidence: ...]` pointers in edited docs still point at existing files/tests.
