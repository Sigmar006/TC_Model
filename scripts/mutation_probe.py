#!/usr/bin/env python3
"""Mutation probe — measure whether a test partition can actually kill wrong code.

The adversarial-testing workflow produces tests from the specification alone. That
is necessary but not sufficient: nothing in it proves the tests would *fail* if the
implementation were wrong. A suite that passes against a broken implementation and
against a correct one is worth nothing, and "I did not weaken the assertion" is a
self-report, not evidence.

This probe restores verifiability. It applies small semantic mutations to a source
file, rebuilds, runs the owning test partition, and records whether the tests caught
the mutant:

    KILLED    tests failed  -> the suite has a detector for this behaviour
    SURVIVED  tests passed  -> the suite has a blind spot; this is a finding
    BUILD ERROR  mutant did not compile -> stillborn, excluded from the score

A survivor is a concrete, reproducible gap: the line, the mutation, and the command
to reproduce it. That is the same deliverable shape the adversarial tests use, but
mechanically derived instead of self-reported.

Usage:
  scripts/mutation_probe.py --source src/<module>/<file>.cpp \\
      --target 1q_<domain>_unit_tests --test unit::<domain> \\
      [--preset llvm-ninja-release-local] [--operators rel_lt,arith_add] \\
      [--limit N] [--list] [--json OUT] [--verbose]

Safety:
  * The source file must be clean in git — otherwise the original cannot be
    restored and the probe refuses to start.
  * The original content is restored in a finally block, and its hash is verified
    afterwards. An interrupted probe (Ctrl-C) still restores.

Exit status: 0 when every viable mutant was killed, 1 when any survived (so a
caller can gate on it), 2 on usage/precondition errors.
"""

import argparse
import codecs
import hashlib
import json
import os
import shlex
import signal
import subprocess
import sys

# Mutation operators. Patterns are matched on space-delimited operator forms, which
# is how this codebase writes binary operators (Google style mandates the spaces).
# Space delimiting is also what keeps `<` in `std::vector<double>` and `<<` in
# stream chains from being mutated — they are not space-delimited and so never match.
OPERATORS = [
    ("rel_lt", " < ", " <= ", "relational boundary shift (strict < becomes <=)"),
    ("rel_le", " <= ", " < ", "relational boundary shift (<= becomes strict <)"),
    ("rel_gt", " > ", " >= ", "relational boundary shift (strict > becomes >=)"),
    ("rel_ge", " >= ", " > ", "relational boundary shift (>= becomes strict >)"),
    ("rel_eq", " == ", " != ", "equality inverted"),
    ("rel_ne", " != ", " == ", "inequality inverted"),
    ("arith_add", " + ", " - ", "addition becomes subtraction"),
    ("arith_sub", " - ", " + ", "subtraction becomes addition"),
    ("arith_mul", " * ", " / ", "multiplication becomes division"),
    ("arith_div", " / ", " * ", "division becomes multiplication"),
    ("logic_and", " && ", " || ", "conjunction becomes disjunction"),
    ("logic_or", " || ", " && ", "disjunction becomes conjunction"),
    ("const_one_zero", "1.0", "0.0", "constant 1.0 becomes 0.0"),
    ("const_zero_one", "0.0", "1.0", "constant 0.0 becomes 1.0"),
]
OPERATORS_BY_NAME = {op[0]: op for op in OPERATORS}

C_RESET = "\033[0m"
C_KILLED = "\033[32m"
C_SURVIVED = "\033[31m"
C_ERROR = "\033[33m"
C_DIM = "\033[2m"


def color(text, code):
    if not sys.stdout.isatty() or os.environ.get("NO_COLOR"):
        return text
    return code + text + C_RESET


def run(cmd, timeout=None):
    """Run a command, returning (returncode, combined_output)."""
    proc = subprocess.run(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, timeout=timeout,
    )
    return proc.returncode, proc.stdout


def require_clean(path):
    """The probe rewrites the file and must be able to restore it via git."""
    code, out = run(["git", "status", "--porcelain", "--", path])
    if code != 0:
        raise SystemExit("mutation_probe: not a git work tree? (%s)" % out.strip())
    if out.strip():
        raise SystemExit(
            "mutation_probe: %s has uncommitted changes.\n"
            "Commit or stash it first — the probe mutates the file and restores it "
            "with `git checkout`, which would otherwise discard your edits." % path
        )


def mask_string_literals(line):
    """Blank out the contents of string/char literals, preserving length.

    Keeps byte offsets valid so a match found on the masked text can be spliced
    back into the original, while preventing mutations inside literals.
    """
    out = list(line)
    index = 0
    length = len(line)
    while index < length:
        char = line[index]
        if char in ('"', "'"):
            quote = char
            index += 1
            while index < length:
                if line[index] == "\\":
                    out[index] = " "
                    index += 1
                    if index < length:
                        out[index] = " "
                        index += 1
                    continue
                if line[index] == quote:
                    index += 1
                    break
                out[index] = " "
                index += 1
            continue
        index += 1
    return "".join(out)


def read_source(path):
    """Return (text-without-BOM, had_bom).

    src/ files in this repo carry a UTF-8 BOM (the pre-commit hook adds one), so the
    probe must round-trip it exactly — reading with utf-8-sig and writing it back
    would otherwise strip a BOM-less file's nothing and add one to a BOM-less file.
    """
    with open(path, "rb") as handle:
        raw = handle.read()
    return raw.decode("utf-8-sig"), raw.startswith(codecs.BOM_UTF8)


def write_source(path, text, had_bom):
    data = text.encode("utf-8")
    if had_bom:
        data = codecs.BOM_UTF8 + data
    with open(path, "wb") as handle:
        handle.write(data)


def sha256_of(path):
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def code_region(line):
    """Return the index where the trailing comment starts, or len(line).

    Naive but adequate here: a `//` inside a string literal is masked first, so the
    only remaining way to hit a false comment is a URL in a literal, which this
    codebase does not contain.
    """
    masked = mask_string_literals(line)
    position = masked.find("//")
    return len(line) if position < 0 else position


def enumerate_mutants(path, operators):
    """Return (lines, mutants) — one dict per single-point mutation of `path`."""
    text, _ = read_source(path)
    lines = text.split("\n")

    mutants = []
    for line_index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("//"):
            continue
        limit = code_region(line)
        if limit == 0:
            continue
        code = line[:limit]
        masked = mask_string_literals(code)
        for name, needle, replacement, description in operators:
            start = masked.find(needle)
            while start >= 0:
                mutated_code = code[:start] + replacement + code[start + len(needle):]
                mutants.append({
                    "operator": name,
                    "description": description,
                    "line": line_index + 1,
                    "column": start + 1,
                    "before": line,
                    "after": mutated_code + line[limit:],
                    "needle": needle.strip(),
                    "replacement": replacement.strip(),
                    "_line_index": line_index,
                })
                start = masked.find(needle, start + 1)
    return lines, mutants


def write_mutant(path, lines, mutant, had_bom):
    mutated = list(lines)
    mutated[mutant["_line_index"]] = mutant["after"]
    write_source(path, "\n".join(mutated), had_bom)


# ---------------------------------------------------------------------------
# History mode — seed the probe with defects this repository actually shipped.
#
# Operator mutation answers "can the suite distinguish a mutation I invented?".
# History seeds answer a stronger question: "can the suite catch a mistake we
# actually made?" The seed is not a reconstruction — it is `git apply -R` of the
# fix commit, i.e. the exact pre-fix state. Nothing is interpreted, so no
# selection bias creeps in on the part of whoever picked the seeds.
#
# The catch is that "how many defects are in this commit" cannot be decided
# mechanically, so the tool does not pretend to decide it. It marks conservatively
# and hands the ambiguous ones over for separate review.
# ---------------------------------------------------------------------------


def git_output(args, stdin=None):
    proc = subprocess.run(args, input=stdin, stdout=subprocess.PIPE,
                          stderr=subprocess.DEVNULL)
    return proc.returncode, proc.stdout


def git_lines(args):
    code, out = git_output(["git"] + args)
    if code != 0:
        return []
    return [line for line in out.decode("utf-8", "replace").split("\n") if line.strip()]


def history_candidates(limit):
    """Fix commits touching src/ or include/, newest first."""
    candidates = []
    for sha in git_lines(["log", "--grep=^fix", "-%d" % limit, "--format=%H"]):
        paths = git_lines(["show", "--name-only", "--format=", sha, "--", "src/*", "include/*"])
        if not paths:
            continue
        candidates.append({
            "sha": sha,
            "short": sha[:8],
            "subject": (git_lines(["log", "-1", "--format=%s", sha]) or [""])[0],
            "paths": paths,
            "diffstat": diffstat_of(sha, paths),
        })
    return candidates


def diffstat_of(sha, paths):
    """`+A -D` totals for the commit's source changes, for the MISSED triage below."""
    added = deleted = 0
    for line in git_lines(["show", "--numstat", "--format=", sha, "--"] + paths):
        parts = line.split("\t")
        if len(parts) >= 2 and parts[0].isdigit() and parts[1].isdigit():
            added += int(parts[0])
            deleted += int(parts[1])
    return "+%d -%d" % (added, deleted)


def is_reversible(candidate):
    """Whether the fix still applies backwards — later commits may have moved the context."""
    code, patch = git_output(["git", "show", candidate["sha"], "--"] + candidate["paths"])
    if code != 0:
        return False
    check = subprocess.run(["git", "apply", "-R", "--check"], input=patch,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return check.returncode == 0


def module_of(path):
    """`src/<module>/…` or `include/1q/<module>/…` -> `<module>`; "" if neither."""
    parts = path.split("/")
    if len(parts) >= 3 and parts[0] == "src":
        return parts[1]
    if len(parts) >= 4 and parts[0] == "include" and parts[1] == "1q":
        return parts[2]
    return ""


def apply_reverse(candidate):
    code, patch = git_output(["git", "show", candidate["sha"], "--"] + candidate["paths"])
    if code != 0:
        return False
    applied = subprocess.run(["git", "apply", "-R"], input=patch,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return applied.returncode == 0


def restore_paths(paths):
    """Undo a reverse-applied patch.

    Calls git directly rather than through `git_output`, which does NOT prepend
    `git` — routing through it here tried to exec a binary named `checkout` and
    silently left the mutant applied.
    """
    if not paths:
        return
    subprocess.run(["git", "checkout", "--"] + list(paths),
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_history(args):
    # Untracked trees elsewhere are none of our business; what matters is that
    # src/ and include/ hold no edits we would clobber when restoring.
    dirty = git_lines(["status", "--porcelain", "--untracked-files=no", "--", "src", "include"])
    if dirty:
        raise SystemExit(
            "mutation_probe: src/ or include/ has uncommitted changes:\n  %s\n"
            "Commit or stash first — history mode reverses and restores, which would "
            "otherwise discard your edits." % "\n  ".join(dirty))

    candidates = history_candidates(args.history_limit)
    print("history mode — seeds are defects this repository actually shipped")
    print("  candidates: %d fix commits touching src/ or include/\n" % len(candidates))

    seeds, marked, unreachable = [], [], []
    for candidate in candidates:
        if not is_reversible(candidate):
            unreachable.append(candidate)
        elif len(candidate["paths"]) > 1 and not args.include_multi:
            marked.append(candidate)
        else:
            seeds.append(candidate)

    print("  reversible: %d  (auto-run %d, marked for separate review %d, unreachable %d)\n"
          % (len(seeds) + len(marked), len(seeds), len(marked), len(unreachable)))

    if marked:
        print("  MARKED — several files in one commit may mean several defects at once;")
        print("  reversing all of them would make the result unattributable. Look at these")
        print("  one by one (or rerun with --include-multi if you have checked them):")
        for candidate in marked:
            print("    %s  %s" % (candidate["short"], candidate["subject"][:62]))
            for path in candidate["paths"]:
                print("        %s" % path)
        print("")

    if unreachable:
        print("  UNREACHABLE — the fix no longer applies backwards (context drifted):")
        for candidate in unreachable:
            print("    %s  %s" % (candidate["short"], candidate["subject"][:62]))
        print("")

    if not seeds:
        print("no runnable seeds")
        return 0

    results = []
    interrupted = {"flag": False}

    def handle_signal(signum, frame):
        interrupted["flag"] = True
        raise KeyboardInterrupt()

    previous_handler = signal.signal(signal.SIGINT, handle_signal)
    try:
        for index, candidate in enumerate(seeds, start=1):
            module = module_of(candidate["paths"][0])
            target = args.target or ("1q_%s_unit_tests" % module)
            test = args.test or ("unit::%s" % module)
            prefix = "  [%d/%d] %s  %s" % (index, len(seeds), candidate["short"],
                                           candidate["subject"][:50])

            if not module:
                print("%s  SKIPPED (cannot derive a module from %s)"
                      % (prefix, candidate["paths"][0]))
                candidate["status"] = "skipped"
                results.append(candidate)
                continue

            if not apply_reverse(candidate):
                print("%s  APPLY FAILED" % prefix)
                candidate["status"] = "apply_failed"
                results.append(candidate)
                restore_paths(candidate["paths"])
                continue

            build_code, build_out = run(
                ["cmake", "--build", "--preset", args.preset, "--target", target], timeout=1800)
            if build_code != 0:
                print("%s  %s" % (prefix, color("%-12s" % "BUILD ERROR", C_ERROR)))
                if args.verbose:
                    print(build_out)
                candidate["status"] = "build_error"
            else:
                test_code, test_out = run(
                    ["ctest", "--preset", args.preset, "-R", test, "--output-on-failure"],
                    timeout=1800)
                candidate["status"] = "killed" if test_code != 0 else "survived"
                print("%s  %s" % (prefix, color(
                    "%-12s" % ("CAUGHT" if test_code != 0 else "MISSED"),
                    C_KILLED if test_code != 0 else C_SURVIVED)))
                if args.verbose:
                    print(test_out)
            candidate["target"] = target
            candidate["test"] = test
            results.append(candidate)
            restore_paths(candidate["paths"])
    except KeyboardInterrupt:
        print("\nhistory mode: interrupted")
    finally:
        signal.signal(signal.SIGINT, previous_handler)
        for candidate in results:
            restore_paths(candidate["paths"])

    dirty = git_lines(["status", "--porcelain", "--untracked-files=no", "--", "src", "include"])
    if dirty:
        # The tree was clean when we started, so anything still dirty is ours.
        # Restore wholesale rather than report and walk away from a mutated tree.
        print("\n  restore incomplete — reverting src/ and include/ wholesale")
        for line in dirty:
            print("    %s" % line)
        restore_paths(["src", "include"])
        dirty = git_lines(["status", "--porcelain", "--untracked-files=no", "--", "src", "include"])
    if dirty:
        print("\n!! RESTORE FAILED — working tree still has changes. Inspect `git status`.")
        for line in dirty:
            print("   %s" % line)
        return 2

    caught = [c for c in results if c["status"] == "killed"]
    missed = [c for c in results if c["status"] == "survived"]
    stillborn = [c for c in results if c["status"] == "build_error"]
    other = [c for c in results if c["status"] in ("skipped", "apply_failed")]
    viable = len(caught) + len(missed)

    print("\n  caught %d, missed %d, build-error %d, skipped %d"
          % (len(caught), len(missed), len(stillborn), len(other)))
    if viable:
        # Deliberately not phrased as a score: the denominator includes non-behavioural
        # `fix:` commits, so a percentage here would be a number nobody should act on.
        print("  seeded %d, tests noticed %d — triage the missed list below; "
              "not every `fix:` commit is a defect" % (viable, len(caught)))

    if missed:
        print("\n  MISSED — seeds whose revert the tests did not notice.")
        print("  TRIAGE EACH ONE before calling it a gap: a `fix:` commit is not always a")
        print("  behavioural defect. Reversing a comment, a log string, a warning fix, a")
        print("  layering change or a compiler-compat shim is *supposed* to go unnoticed —")
        print("  counting those as gaps inflates the finding list with noise. The diffstat")
        print("  and subject are printed to make that call quick:")
        for candidate in missed:
            print("    %s  %s   [%s]" % (candidate["short"], candidate["subject"],
                                         candidate["diffstat"]))
            print("      paths: %s   run: ctest -R %s" % (", ".join(candidate["paths"]),
                                                          candidate["test"]))

    if args.json:
        payload = {
            "mode": "history", "preset": args.preset,
            "caught": len(caught), "missed": len(missed),
            "build_error": len(stillborn), "skipped": len(other),
            "marked_for_review": [
                {k: v for k, v in c.items()} for c in marked],
            "unreachable": [{"sha": c["short"], "subject": c["subject"]} for c in unreachable],
            "results": [
                {"sha": c["short"], "subject": c["subject"], "status": c["status"],
                 "paths": c["paths"], "target": c.get("target", ""), "test": c.get("test", "")}
                for c in results
            ],
        }
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        print("\n  json written to %s" % args.json)

    if interrupted["flag"]:
        return 2
    return 1 if missed else 0


def main():
    parser = argparse.ArgumentParser(
        description="Measure whether a test partition kills mutated implementations.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--source", help="C++ source under src/ to mutate")
    parser.add_argument("--target", help="ninja build target owning the tests")
    parser.add_argument("--test", help="ctest -R regex, e.g. unit::precision_evaluation")
    parser.add_argument("--preset", default="llvm-ninja-release-local")
    parser.add_argument("--history", action="store_true",
                        help="history mode: seed from defects this repo actually shipped")
    parser.add_argument("--history-limit", type=int, default=60,
                        help="how many recent fix commits to consider (default 60)")
    parser.add_argument("--include-multi", action="store_true",
                        help="history mode: also run seeds that touch several files "
                             "(default: mark them for separate review instead)")
    parser.add_argument("--operators", default="", help="comma-separated subset; default all")
    parser.add_argument("--limit", type=int, default=0, help="stop after N mutants (0 = all)")
    parser.add_argument("--list", action="store_true", help="enumerate mutants and exit")
    parser.add_argument("--json", default="", help="write machine-readable results here")
    parser.add_argument("--verbose", action="store_true", help="stream build/test output")
    args = parser.parse_args()

    # A probe runs a rebuild and a test per mutant — minutes, not seconds. Without
    # line buffering, redirecting to a file shows nothing until it finishes, which
    # makes a long run look hung.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:  # Python < 3.7
        pass

    if args.history:
        return run_history(args)

    if not args.source or not args.target or not args.test:
        raise SystemExit(
            "mutation_probe: --source, --target and --test are required unless --history is given")

    if not os.path.isfile(args.source):
        raise SystemExit("mutation_probe: no such source file: %s" % args.source)

    if args.operators:
        names = [name.strip() for name in args.operators.split(",") if name.strip()]
        unknown = [name for name in names if name not in OPERATORS_BY_NAME]
        if unknown:
            raise SystemExit(
                "mutation_probe: unknown operator(s): %s\navailable: %s"
                % (", ".join(unknown), ", ".join(OPERATORS_BY_NAME))
            )
        operators = [OPERATORS_BY_NAME[name] for name in names]
    else:
        operators = OPERATORS

    require_clean(args.source)

    original_text, had_bom = read_source(args.source)
    original_hash = sha256_of(args.source)

    lines, mutants = enumerate_mutants(args.source, operators)
    if args.limit > 0:
        mutants = mutants[:args.limit]

    print("mutation probe")
    print("  source: %s" % args.source)
    print("  target: %s   test: %s   preset: %s" % (args.target, args.test, args.preset))
    print("  operators: %s" % ", ".join(op[0] for op in operators))
    print("  mutants: %d" % len(mutants))
    print("")

    if args.list:
        for mutant in mutants:
            print("  line %4d col %3d  %-16s  %s -> %s" % (
                mutant["line"], mutant["column"], mutant["operator"],
                mutant["needle"], mutant["replacement"]))
        return 0

    if not mutants:
        print("no mutants found — check --source and --operators")
        return 0

    interrupted = {"flag": False}

    def handle_signal(signum, frame):
        interrupted["flag"] = True
        raise KeyboardInterrupt()

    previous_handler = signal.signal(signal.SIGINT, handle_signal)

    results = []
    try:
        for index, mutant in enumerate(mutants, start=1):
            write_mutant(args.source, lines, mutant, had_bom)
            prefix = "  [%d/%d] line %d col %d  %s" % (
                index, len(mutants), mutant["line"], mutant["column"], mutant["operator"])

            build_code, build_out = run(
                ["cmake", "--build", "--preset", args.preset, "--target", args.target],
                timeout=1800,
            )
            if build_code != 0:
                status = "BUILD ERROR"
                outcome = color("%-12s" % status, C_ERROR)
                print("%s  %s" % (prefix, outcome))
                if args.verbose:
                    print(build_out)
                mutant["status"] = "build_error"
                results.append(mutant)
                continue

            test_code, test_out = run(
                ["ctest", "--preset", args.preset, "-R", args.test,
                 "--output-on-failure"],
                timeout=1800,
            )
            mutant["status"] = "killed" if test_code != 0 else "survived"
            outcome = (color("%-12s" % "KILLED", C_KILLED) if test_code != 0
                       else color("%-12s" % "SURVIVED", C_SURVIVED))
            print("%s  %s" % (prefix, outcome))
            if args.verbose:
                print(test_out)
            results.append(mutant)
    except KeyboardInterrupt:
        print("\nmutation probe: interrupted")
    finally:
        signal.signal(signal.SIGINT, previous_handler)
        write_source(args.source, original_text, had_bom)

    if sha256_of(args.source) != original_hash:
        print("\n!! RESTORE FAILED — %s differs from the original. Check `git diff`." % args.source)
        return 2
    print("\n%s restored (sha256 verified)" % args.source)

    killed = [m for m in results if m["status"] == "killed"]
    survived = [m for m in results if m["status"] == "survived"]
    stillborn = [m for m in results if m["status"] == "build_error"]
    viable = len(killed) + len(survived)
    score = (100.0 * len(killed) / viable) if viable else 0.0

    print("")
    print("  killed %d, survived %d, build-error %d" % (len(killed), len(survived), len(stillborn)))
    print("  mutation score: %.1f%% (%d/%d viable)" % (score, len(killed), viable))

    if survived:
        print("\n  survivors — the suite has no detector for these behaviours:")
        for mutant in survived:
            print("    line %d col %d  %s  (%s -> %s)" % (
                mutant["line"], mutant["column"], mutant["operator"],
                mutant["needle"], mutant["replacement"]))
            print("      %s" % mutant["before"].strip())
        print("\n  reproduce one with:")
        print("    %s --source %s --target %s --test %s --preset %s --operators %s --limit 1"
              % (shlex.quote(sys.argv[0]), args.source, args.target, args.test,
                 args.preset, survived[0]["operator"]))

    if args.json:
        payload = {
            "source": args.source, "target": args.target, "test": args.test,
            "preset": args.preset, "score": score,
            "killed": len(killed), "survived": len(survived), "build_error": len(stillborn),
            "mutants": [
                {key: value for key, value in mutant.items() if not key.startswith("_")}
                for mutant in results
            ],
        }
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        print("\n  json written to %s" % args.json)

    if interrupted["flag"]:
        return 2
    return 1 if survived else 0


if __name__ == "__main__":
    sys.exit(main())
