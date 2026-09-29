#!/usr/bin/env python3
"""Fail CI if the non-actuating shadow subsystem gains control authority."""

from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
FILES = [
    ROOT / "lib/Espfc/src/Control/ShadowFeatures.cpp",
    ROOT / "lib/Espfc/src/Control/ShadowFeatures.h",
]

# ShadowFeatures may read authoritative state to calculate diagnostics, but it
# may not assign to any controller/output state. setDebug() is intentionally
# allowed because debug slots are diagnostics.
FORBIDDEN_ASSIGNMENTS = [
    "setpoint",
    "innerPid",
    "angleV2",
    "assistedMode",
    "output",
    "currentMixer",
    "failsafe",
]

FORBIDDEN_CALLS = [
    "_model.disarm(",
    "_model.updateModes(",
    "_model.updateSwitchActive(",
    "_model.setArmingDisabled(",
    "_model.setOutputSaturated(",
    "_model.save(",
]

FORBIDDEN_INCLUDES = [
    "Control/Controller.h",
    "Control/Pid.h",
    "Output/Mixer",
    "Output/Output",
]


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.S)
    source = re.sub(r"//.*", "", source)
    return source


def main() -> int:
    errors: list[str] = []

    for path in FILES:
        source = path.read_text(encoding="utf-8")
        code = strip_comments(source)

        for field in FORBIDDEN_ASSIGNMENTS:
            pattern = re.compile(
                rf"_model\.state\.{re.escape(field)}"
                rf"(?:(?:\[[^\]]*\])|(?:\.[A-Za-z_][A-Za-z0-9_]*))*"
                rf"\s*(?:\+=|-=|\*=|/=|%=|=(?!=))"
            )
            for match in pattern.finditer(code):
                line = code.count("\n", 0, match.start()) + 1
                errors.append(
                    f"{path.relative_to(ROOT)}:{line}: "
                    f"shadow assignment to authoritative state '{field}'"
                )

            alias_pattern = re.compile(
                rf"(?:auto|[A-Za-z_][A-Za-z0-9_:<>]*)\s*&\s*"
                rf"[A-Za-z_][A-Za-z0-9_]*\s*=\s*"
                rf"_model\.state\.{re.escape(field)}\b"
            )
            for match in alias_pattern.finditer(code):
                line = code.count("\n", 0, match.start()) + 1
                errors.append(
                    f"{path.relative_to(ROOT)}:{line}: "
                    f"non-const alias to authoritative state '{field}'"
                )

        for call in FORBIDDEN_CALLS:
            if call in code:
                line = code.count("\n", 0, code.index(call)) + 1
                errors.append(
                    f"{path.relative_to(ROOT)}:{line}: "
                    f"forbidden authority call {call}"
                )

        for include in FORBIDDEN_INCLUDES:
            if include in code:
                line = code.count("\n", 0, code.index(include)) + 1
                errors.append(
                    f"{path.relative_to(ROOT)}:{line}: "
                    f"forbidden control/output include {include}"
                )

    if errors:
        print("Shadow authority boundary violation(s):", file=sys.stderr)
        for error in errors:
            print(f" - {error}", file=sys.stderr)
        return 1

    print("Shadow authority boundary: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
