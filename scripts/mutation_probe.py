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


def main():
    parser = argparse.ArgumentParser(
        description="Measure whether a test partition kills mutated implementations.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--source", required=True, help="C++ source under src/ to mutate")
    parser.add_argument("--target", required=True, help="ninja build target owning the tests")
    parser.add_argument("--test", required=True, help="ctest -R regex, e.g. unit::precision_evaluation")
    parser.add_argument("--preset", default="llvm-ninja-release-local")
    parser.add_argument("--operators", default="", help="comma-separated subset; default all")
    parser.add_argument("--limit", type=int, default=0, help="stop after N mutants (0 = all)")
    parser.add_argument("--list", action="store_true", help="enumerate mutants and exit")
    parser.add_argument("--json", default="", help="write machine-readable results here")
    parser.add_argument("--verbose", action="store_true", help="stream build/test output")
    args = parser.parse_args()

    if not os.path.isfile(args.source):
        raise SystemExit("mutation_probe: no such source file: %s" % args.source)

    # A probe runs a rebuild and a test per mutant — minutes, not seconds. Without
    # line buffering, redirecting to a file shows nothing until it finishes, which
    # makes a long run look hung.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:  # Python < 3.7
        pass

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
