#!/usr/bin/env python3
"""Add idempotent architecture notes to sensor source snapshots."""

from __future__ import annotations

import argparse
from pathlib import Path


MARKER = "Supplemental architecture notes for acceptance review"

MODEL_CONTEXT = {
    "infrared_sensor": {
        "model": "SBIRS infrared sensor",
        "layers": {
            "config": (
                "validated sensor configuration and policy contracts",
                "mission, hardware, orientation, environment, and runtime settings",
                "session construction and pipeline configuration",
            ),
            "environment": (
                "environmental state evaluation for infrared observation",
                "scene geometry and configured environmental facts",
                "radiometry and detection pipeline inputs",
            ),
            "foundation": (
                "shared SBIRS geometry, radiometry, noise, and error calculations",
                "sensor configuration, target geometry, and physical facts",
                "boresight, acquisition, and measurement processing",
            ),
            "pipeline": (
                "wide-field search, pointing, acquisition, and tracking processing",
                "cycle scene, cue, sensor state, and resolved physical parameters",
                "detections, estimates, diagnostics, and lifecycle observations",
            ),
            "runtime": (
                "runtime configuration resolution and controller orchestration",
                "validated session configuration and staged runtime patches",
                "the executable SBIRS pipeline and cycle result assembly",
            ),
            "session": (
                "public cycle contract, validation, adapters, recording, and replay",
                "external cycle data and validated session configuration",
                "stable output frames, issues, debug views, and replay records",
            ),
            "tracking": (
                "angle-domain filtering and internal tracking state",
                "accepted infrared measurements and prior track state",
                "filtered estimates consumed by lifecycle and output assembly",
            ),
        },
    },
    "lrdr_radar": {
        "model": "LRDR remote identification radar",
        "layers": {
            "config": (
                "validated radar hardware, mission, environment, and policy contracts",
                "external initialization values and staged runtime settings",
                "session, RF, detection, tracking, and recognition configuration",
            ),
            "dwell": (
                "per-dwell emission, receiver, beam, echo, and detection processing",
                "dwell plan, RF scene, target geometry, and radar hardware state",
                "resolved RF cells, detection decisions, and track measurements",
            ),
            "internal": (
                "module-private radar physics, propagation, clutter, and scan geometry",
                "resolved environment, target geometry, and hardware parameters",
                "physical facts consumed by dwell detection and scheduling",
            ),
            "recognition": (
                "feature extraction, template matching, and recognition accumulation",
                "confirmed track snapshots, observation context, and feature database",
                "feature measurements, recognition results, and cycle summaries",
            ),
            "runtime": (
                "top-level RF, detection, tracking, and recognition orchestration",
                "validated cycle input, dwell plan, runtime policy, and environment",
                "cycle products, emissions, diagnostics, and attribution snapshots",
            ),
            "session": (
                "public cycle contract, validation, adapters, recording, and replay",
                "external targets, platform position, RF scene, and session settings",
                "stable output frames, issues, task feedback, and replay records",
            ),
            "tracking": (
                "measurement association, filtering, and track lifecycle management",
                "accepted detections, measurement covariance, and prior track seeds",
                "ordered track snapshots consumed by recognition and output assembly",
            ),
        },
    },
}


def resolve_layer(path: Path) -> str:
    known_layers = set(MODEL_CONTEXT["infrared_sensor"]["layers"]) | set(
        MODEL_CONTEXT["lrdr_radar"]["layers"]
    )
    for part in reversed(path.parts[:-1]):
        if part in known_layers:
            return part
    return "session"


def make_comment(model_name: str, path: Path) -> str:
    context = MODEL_CONTEXT[model_name]
    layer = resolve_layer(path)
    responsibility, upstream, downstream = context["layers"][layer]
    file_role = "interface declarations and data contracts" if path.suffix == ".h" else "executable implementation"
    return "\n".join(
        [
            "/*",
            f" * {MARKER}.",
            f" * Unit: {path.stem}.",
            f" * Model: {context['model']}.",
            f" * Layer: {layer}.",
            f" * File role: {file_role}.",
            f" * Responsibility: {responsibility}.",
            f" * Upstream boundary: {upstream}.",
            f" * Downstream boundary: {downstream}.",
            " * State ownership: state remains with the enclosing session or controller.",
            " * Units and frames: preserve the explicit units and coordinate-frame contracts.",
            " * Validation: reject invalid boundary data before mutating persistent state.",
            " * Failure behavior: preserve the declared neutral-result or fail-closed path.",
            " * Dependency rule: use public DTOs at boundaries and keep private types internal.",
            " * Maintenance rule: update comments when inputs, outputs, or ownership change.",
            " */",
        ]
    )


def annotate(path: Path, model_name: str) -> bool:
    raw = path.read_bytes()
    has_bom = raw.startswith(b"\xef\xbb\xbf")
    text = raw.decode("utf-8-sig")
    if MARKER in text:
        return False
    newline = "\r\n" if b"\r\n" in raw else "\n"
    comment = make_comment(model_name, path).replace("\n", newline)
    encoded = (comment + newline + newline + text).encode("utf-8")
    path.write_bytes((b"\xef\xbb\xbf" if has_bom else b"") + encoded)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot_root", type=Path)
    parser.add_argument(
        "--models", nargs="+", choices=sorted(MODEL_CONTEXT), default=sorted(MODEL_CONTEXT)
    )
    args = parser.parse_args()

    if not args.snapshot_root.is_dir():
        raise SystemExit(f"snapshot root does not exist: {args.snapshot_root}")

    for model_name in args.models:
        model_root = args.snapshot_root / model_name
        if not model_root.is_dir():
            raise SystemExit(f"model snapshot does not exist: {model_root}")
        changed = 0
        for path in sorted(model_root.rglob("*")):
            if path.is_file() and path.suffix.lower() in {".h", ".cpp"}:
                changed += int(annotate(path, model_name))
        print(f"{model_name}: annotated {changed} files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
