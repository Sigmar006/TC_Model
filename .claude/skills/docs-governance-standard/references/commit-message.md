# Commit message standard — 契约收敛与 docs 工作

提交标题格式由 `.githooks/commit-msg` 强制（Conventional Commits，违规即拒绝提交）；
提交信息不带署名尾行。本文件给出契约收敛工作下的具体写法（母本分支 42 个提交的模式）。

## 模板

```
<type>(<scope>): <imperative 描述>

<可选 body：关键决策 / 改动清单 / 验证结果>
```

## type 与 scope 对照

| 阶段 | type | scope | 描述形态 |
|---|---|---|---|
| 契约冻结 / 回写 / 审查 | `docs` | `common` 或具体模块 | `freeze …`、`mark … complete`、`…收敛回写` |
| 逐模块实施 | `refactor`/`feat`/`fix` | **模块名**（`airborne_radar`、`electronic_surveillance_radar`、`sar`、`eos`、`sbirs_sensor`） | `unify cycle issue list (ArIssue + phase + optional location)` |
| 测试锁定 | `test` | 模块名 | `补 phase 断言锁定规则 14 来源标签` |
| 消费侧样例 | `chore` | 模块名 | `修正 consumer 样例过时 issue 引用（规则 14）` |

scope 取值（模块/域）：`airborne_radar`、`electronic_surveillance_radar`、`sar`、
`electro_optical_sensor`（惯用 `eos`）、`sbirs_sensor`、`flight_dynamic`、`common`、
`replay`、`examples`、`practice`。

## 描述写法

1. **imperative mood、小写开头**（英文），或中文动词短句。
2. **描述含关键新符号**：`unify cycle issue list (ArIssue + phase + optional location)` —
   括号内列出新类型/新概念，便于按符号检索历史。
3. **docs 提交描述说明文档动作与对象**：`boundaries 补 config 域统一映射（规则 14 Stage C 回写）`、
   `审查文档补修复状态与 codec 定位缺陷注记`。
4. 收敛回写用破折号带出结论：`COMMON-OQ-9 收敛回写——规则 14 固化校验层归属与 issues 流向`。

## body 写法

- **冻结提交**：body 写规则核心决策与字面约定（如 code 编码规则、既有 code 保持冻结）。
- **收敛回写提交**：body 按文件分条列出改动（`session_contract.md 规则 14 新增…`、
  `contract.md 非执行周期条款措辞改为…`、`open_questions.md 迁出 COMMON-OQ-9`）。
- **Stage C 回写提交**：body 末尾加 Validation 块：

  ```
  Validation:
  - full build (llvm-ninja-release-local): pass
  - ctest: 61/61 pass (incl. sar unit/replay/contract/batch_validation)
  ```

- **审查落库提交**：描述可含豁免注记（`运行期 invalid_config 聚合码契约 14c 文档豁免；审查文档落库`）。

## 反例（母本审查发现的提交问题，避免重犯）

- 描述空泛无符号：`update docs` → 应 `mark rule 14 alignment complete across all five modules`。
- 代码提交混 docs：契约/回写与代码分提交。

## 自检清单

- [ ] `type(scope):` 合法（type ∈ feat/fix/refactor/docs/test/chore/perf；scope 为模块/域）
- [ ] 描述 imperative/短句，含关键新符号
- [ ] body 有内容（冻结决策 / 文件清单 / Validation 块），不只靠描述
- [ ] 无署名尾行
