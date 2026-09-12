---
type: llm
focus: last_message
---

PASS if the reply states the risk tier as major AND states which review lanes it ran versus skipped
with a reason, specifically that the code-quality lane (code-simplifier) and the test-coverage lane
did not trigger — the former because this review is not gating a branch merge, the latter because
the change adds no new module. Naming the lanes as "Lane 1/2/3", "correctness", "code-simplifier",
"test coverage", or their Chinese equivalents all count.

FAIL if the reply omits the tier, claims all lanes ran unconditionally, or gives no reason for a
skipped lane.

Explicitness about which lanes did NOT run is the point of this grader. A reply that only lists what
it reviewed, without saying what it deliberately skipped and why, fails.
