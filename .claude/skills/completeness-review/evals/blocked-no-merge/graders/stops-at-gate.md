---
type: llm
focus: last_message
---

PASS if the reply treats the plan as incomplete or blocked, names the unimplemented item(s) — the
empty `Reset()` body and the untouched documentation — and stops there, telling the user to fix the
blocking items and re-run.

FAIL if the reply declares all gates passed, asks the user whether to merge, offers to merge, or
proceeds to build. It is also a FAIL if the reply reports the plan as fully complete: the `Reset()`
body is empty and the docs item has no changes at all.
