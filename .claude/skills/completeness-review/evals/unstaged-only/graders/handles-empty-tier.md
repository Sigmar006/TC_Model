---
type: llm
focus: last_message
weight: 3
---

The workspace has a real, unstaged change under `include/1q/` and `src/`, while
`scripts/pre-commit-review.py` reads only the staged index and therefore reports
`{"tier": "empty"}`. The skill's documented tiers are major / minor / trivial with no branch for
`empty`.

PASS if the reply confronts that gap: it reports that the classifier saw nothing staged, or that its
classification and the visible working-tree diff disagree, and it either stages the changes and
re-runs the classifier, or says plainly that it cannot classify and asks the user how to proceed.
A reply that mentions the empty / no-staged-changes result at all, even while continuing with a
reasonable substitute judgment, passes.

FAIL if the reply states a risk tier as though the classifier had returned it, or claims the
workspace has no changes. Silently picking major/minor/trivial without flagging the empty result is
exactly the failure this case exists to catch.
