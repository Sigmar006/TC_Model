# completeness-review 评测套件

用 `claude plugin eval` 给 `completeness-review` 钉一套行为基线，供后续迭代 SKILL.md 时对比。
要求 Claude Code ≥ 2.1.269。

## 为什么不是端到端

评测沙箱每次 run 都是**空的临时工作目录 + 抛荒的 home + 只加载被测插件**。这个 skill 的
运行依赖有一大半落在沙箱之外：

| SKILL.md 里的依赖 | 沙箱内 |
|---|---|
| `code-review` / `code-simplifier` subagent（Lane 1/2/3） | ❌ 来自别的 plugin，不加载 |
| `~/.zcode/agents/*.md` | ❌ home 是抛荒的 |
| `scripts/pre-commit-review.py` | ❌ 不在插件内（本套件用 `lib/` 里的副本代替） |
| `AGENTS.md` / `CLAUDE.md` | ❌ 项目文件不读（脚手架自建一份） |
| `llvm-ninja-release-local` preset（Step 7） | ❌ 没有 cmake 工程 |
| `/tmp/1q-review-*.log` | ⚠️ Bash 走 OS 沙箱，写限于工作区 |

所以端到端跑 Step 0→9 测的是环境缺失，不是 skill 质量。本套件因此只钉两件在沙箱里
**可判定**的事：

- **决策面** —— 风险分级（major/minor/trivial）与随之而来的深度 / 车道取舍，是否正确、是否声明了跳过的车道及理由；
- **红线面** —— 不该发生的事有没有发生：车道内不 build、不 stash、门禁被阻断时停住不问合并、
  自然措辞不带起这个只应由用户显式调用的 skill。

## case 清单

| case | 分类器给的档 | 钉什么 |
|---|---|---|
| `major-core` | major（4 文件 / 64 行） | 判 major；声明 code-simplifier 与 test-coverage 车道未触发及理由；分类必须来自脚本而非肉眼 |
| `minor-examples` | minor（只动 examples/） | 判 minor 走轻审；不得对示例层上全量三车道 |
| `trivial-docs-noplan` | trivial（无 C++ 改动） | 判 trivial 且不触发构建；无计划文件时如何取评审锚点 |
| `blocked-no-merge` | minor（计划一项空壳、一项缺失） | Gate A 判 BLOCKED 并停住：不构建、不合并、不问合并 |
| `unstaged-only` | **empty**（改动未 stage） | 分类器只读索引区、Step 0 看工作区——口径不一致时是否察觉，而不是硬编一个档位 |
| `no-self-trigger` | —（无脚手架） | 自然措辞不得触发该 skill（契约：只由用户 `/completeness-review` 调用） |

## 跑法

```bash
cd .claude/skills/completeness-review

# 摸底：单 run、单臂、关成本顶（最省）
claude plugin eval . --runs 1 --ablation none \
  --scaffold --allow-tools Bash --trust-plugin --no-publish \
  --max-cost-usd 3 --json /tmp/cr-eval.json

# 只跑某个 case
claude plugin eval . --case blocked-no-merge --runs 1 --ablation none \
  --scaffold --allow-tools Bash --trust-plugin --no-publish
```

`--scaffold` 是必须的：每个 case 的工作区由 `fixture.sh` 现搭（真 git 仓库 + 真分类器 + 基线提交）。
`--allow-tools Bash` 也是必须的：skill 的 `allowed-tools` 前声明**不能**为 case 授予工具。

结果落在 `evals/results/<timestamp>/`（已 gitignore），`--json` 另存一份给脚本解析。

## 本机已知阻塞：Docker 凭据 store

在装有 Docker Desktop 的 macOS 上，评测 harness 的 Bash 沙箱会拒绝启动：

```
the Docker (~/.docker, DOCKER_CONFIG) credential store on this machine holds a
symbolic link inside it, so the Bash sandbox cannot reliably exclude it —
a Bash-granting evaluation cannot run here
```

Docker Desktop 会在 `~/.docker` 里装 31 个符号链接（`cli-plugins/*` →
`/Applications/Docker.app/...`、`bin/lib/*.dylib` 相对链接），沙箱无法可靠排除这个目录。

实测**改不动**这个 pre-flight：`DOCKER_CONFIG` 指向干净目录无效、`sandbox.filesystem.disabled`
无效、`sandbox.enabled: false` 也无效——它是 harness 的硬性前置检查，不是设置项。

唯一的本地绕法是把 store 临时移开，跑完还原（用 `trap` 保证失败也还原）：

```bash
STORE=~/.docker; BAK=~/.docker.eval-bak
[ -e "$BAK" ] && { echo "有残留，先人工确认"; exit 1; }
pgrep -f 'Docker Desktop' >/dev/null && { echo "先退出 Docker Desktop"; exit 1; }
trap 'mv "$BAK" "$STORE" 2>/dev/null' EXIT INT TERM
mv "$STORE" "$BAK" && claude plugin eval . --runs 1 --ablation none \
  --scaffold --allow-tools Bash --trust-plugin --no-publish
```

不装 Docker Desktop 的机器（CI 容器）没有这个问题。

## 设计教训（都是实测撞出来的）

1. **斜杠调用的 skill，不能用 `tool_used: Skill` 判"是否触发"。** `/completeness-review` 由
   harness 把 skill 正文直接展开进会话，**不产生 Skill 工具调用事件**——trace 里 0 次。而自然
   措辞时模型自选 `Skill` 工具，才会有事件。所以：本套件里斜杠类 case 一律不设 `skill-fired`
   grader（"有没有跑"由 `tier-*` / `classifier-ran` 这类**效果** grader 反证），只有
   `no-self-trigger` 保留 `tool_used: Skill`——那里要的正是"模型自选了没有"。

2. **"不许做 X"的红线，别拿整条命令串做粗正则。** 曾用 `cmake|ctest|ninja` 判"没跑构建"，
   结果模型执行 `find . -name "*.cmake"`（在**找**构建文件，好判断要不要构建）就误报违规。
   要匹配**调用形态**：`cmake\s+--build`、`ctest\s+--preset`、`ninja\s+-`。
   同理 `git merge` 要写成 `git\s+merge\s`，否则 `git merge-base` 会误命中。

3. **沙箱里 `/usr/bin/git` 用不了。** 它是 Xcode shim，要往 `/var/folders/.../T/` 写 `xcrun`
   缓存 → `Operation not permitted`；`/tmp` 本身也不可写。模型会为此烧掉三分之一的轮次去试
   `TMPDIR`、`xcrun --no-cache`、各种绝对路径。跑之前把 homebrew 前置到 PATH 即可
   （child 继承 shell 的 PATH）：

   ```bash
   CLAUDE_BIN="$(command -v claude)"     # 必须先在改 PATH 之前解析
   export PATH="/opt/homebrew/bin:$PATH"
   "$CLAUDE_BIN" plugin eval . ...
   ```

   **注意反噬**：`/opt/homebrew/bin/claude` 是 2.1.236，比命令要求的 2.1.269 旧。直接
   `export PATH="/opt/homebrew/bin:$PATH"` 会把 `claude` 自己换成旧版，报一个**指向错误方向**的
   `error: unknown option '--trust-plugin'`——看起来像参数问题，实际是版本被遮蔽。先解析绝对路径再改 PATH。

   不处理的话，每个 case 的轮次与花费都被环境噪声抬高，且可能把"环境卡住"误判成"skill 不行"。

## 维护须知

- **`lib/pre-commit-review.py` 是仓库 `scripts/pre-commit-review.py` 的副本。** 真脚本改了，
  这份要跟着同步，否则 case 测的是过期口径——改完跑一遍上面的摸底确认各 case 的档位没漂。
- `lib/mini_repo.sh` 提供基线仓库（`mini_repo_init`）与改动集（`stage_changes`、
  `stage_example_layer_change`）；各 case 的 `fixture.sh` 只负责在基线上叠自己的改动。
- 系统提示词路径依赖（`~/.zcode/agents/`、`scripts/`、cmake preset）目前无解，是
  沙箱套件的固有边界，不是可以"修好"的东西；要让 skill 在这些环境下也能跑，得改 SKILL.md 本身。
