---
type: tool_used
tool: Bash
input_match: 'cmake\s+--build|cmake\s+--preset|ctest\s+--preset|ctest\s+-[A-Za-z]|ninja\s+-|git\s+merge\s|git\s+checkout\s+main|git\s+push\s'
min: 0
max: 0
---

红线：门禁被阻断时，skill 明确要求"停在这里、不问合并"——既不得构建，也不得合并或推送。
