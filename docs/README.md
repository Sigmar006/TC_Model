# docs/

Documentation tree navigation. Project overview lives in the root `README.md`.

## Layout

- `<module>/` — one directory per module, holding its design-doc set (below)
- `common/` — `contract.md`, the cross-module contract authority
- `practice/` — build/test governance, Windows runbook, CI
- `review/` — review documents; completed items close out as tables with `Status: final`
- `explainers/` — background explainers for non-specialist readers

## Per-module design-doc sets

Each module keeps a design-doc set as its design authority:

- `design.md` — navigation entry
- `boundaries.md` — positioning, boundaries, non-goals, veto rationale
- `data-flow.md` — data flow across the module's stages
- `algorithms.md` — algorithm-level decisions and registered parameters

`[evidence: ...]` annotations live in the set; step-by-step algorithm logic belongs in code. The set is a requirement, not current state — some modules are still missing files, so check what exists on disk instead of assuming.

Update the set whenever thresholds, limitations, or veto decisions change.

## Integration guides

- [LRDR 集成指南（1q.lib）](practice/lrdr_integration_guide.md) — 静态库交付清单、Visual Studio 配置、会话调用与 BuildScanPattern 链接排障。
- [评估数据基础信息表（宿主结果表）](review/evaluation_data_foundation_tables_2026-10.md) — 六类 12 张评估表的字段定义、写入来源与当前模型能力缺口。
