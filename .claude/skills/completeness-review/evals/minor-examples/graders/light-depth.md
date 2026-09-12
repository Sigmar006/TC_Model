---
type: llm
focus: last_message
---

PASS if the reply states the risk tier as minor AND describes a light review: a single correctness
lane, with the code-quality lane (code-simplifier) and the test-coverage lane explicitly skipped for
this layer.

FAIL if the reply claims it ran a full three-lane review, or launches a code-quality / simplification
sweep over example code. Example-layer code favors readability over abstraction, so a quality sweep
there is over-review.

Either language (the lanes named in English, by "Lane 1/2/3", or in Chinese) counts.
