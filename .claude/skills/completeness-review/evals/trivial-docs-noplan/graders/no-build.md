---
type: tool_used
tool: Bash
input_match: 'cmake\s+--build|cmake\s+--preset|ctest\s+--preset|ctest\s+-[A-Za-z]|ninja\s+-'
min: 0
max: 0
---

红线：tier = trivial 表示没有 C++ 改动，且文档改动不构成构建输入，因此不得触发构建或测试。
