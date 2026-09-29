#!/usr/bin/env python3
"""Check that the documented Configurator contract matches source boundaries."""

from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

SUPPORT_DOC = ROOT / "docs/CONFIGURATOR_SUPPORT.md"
MSP = ROOT / "lib/Espfc/src/Connect/MspProcessor.cpp"
CLI = ROOT / "lib/Espfc/src/Connect/Cli.cpp"
MODEL_CONFIG = ROOT / "lib/Espfc/src/ModelConfig.h"

SHADOW_MODES = [
    "MODE_HORIZON_SHADOW",
    "MODE_GPS_RESCUE_SHADOW",
    "MODE_POSHOLD_SHADOW",
    "MODE_HEADFREE_SHADOW",
    "MODE_ACRO_TRAINER_SHADOW",
    "MODE_WAYPOINT_SHADOW",
]

UNSUPPORTED_MSP = [
    "MSP2_BETAFLIGHT_BIND",
    "MSP2_MOTOR_OUTPUT_REORDERING",
    "MSP2_SET_MOTOR_OUTPUT_REORDERING",
    "MSP2_SEND_DSHOT_COMMAND",
    "MSP2_SENSOR_OPTICALFLOW",
]

SUPPORTED_COMPAT_MSP = [
    "MSP2_GET_OSD_WARNINGS",
    "MSP2_GET_LED_STRIP_CONFIG_VALUES",
    "MSP2_SET_LED_STRIP_CONFIG_VALUES",
    "MSP_GPS_RESCUE",
    "MSP_SET_GPS_RESCUE",
    "MSP_GPS_RESCUE_PIDS",
    "MSP_SET_GPS_RESCUE_PIDS",
    "MSP_VTXTABLE_BAND",
    "MSP_SET_VTXTABLE_BAND",
    "MSP_VTXTABLE_POWERLEVEL",
    "MSP_SET_VTXTABLE_POWERLEVEL",
]

REQUIRED_DOC_PHRASES = [
    "**ACTIVE**",
    "**SHADOW**",
    "**PERSISTED ONLY**",
    "**FIXED CAPABILITY**",
    "**UNSUPPORTED**",
    "GPS Rescue / RTH",
    "Position Hold",
    "Waypoints",
    "GPS-driven motor authority",
    "MSP2_GET_VTX_DEVICE_STATUS",
    "single battery configuration",
]


def require(text: str, needle: str, where: str, errors: list[str]) -> None:
    normalized_text = " ".join(text.split())
    normalized_needle = " ".join(needle.split())
    if normalized_needle not in normalized_text:
        errors.append(f"{where}: missing {needle!r}")


def main() -> int:
    errors: list[str] = []

    doc = SUPPORT_DOC.read_text(encoding="utf-8")
    msp = MSP.read_text(encoding="utf-8")
    cli = CLI.read_text(encoding="utf-8")
    model = MODEL_CONFIG.read_text(encoding="utf-8")

    for phrase in REQUIRED_DOC_PHRASES:
        require(doc, phrase, str(SUPPORT_DOC.relative_to(ROOT)), errors)

    for symbol in SHADOW_MODES:
        require(model, symbol, str(MODEL_CONFIG.relative_to(ROOT)), errors)

    for command in UNSUPPORTED_MSP:
        require(msp, f"case {command}:", str(MSP.relative_to(ROOT)), errors)

    for command in SUPPORTED_COMPAT_MSP:
        require(msp, f"case {command}:", str(MSP.relative_to(ROOT)), errors)

    require(
        cli,
        "shadow_output_authority: NONE",
        str(CLI.relative_to(ROOT)),
        errors,
    )

    require(
        msp,
        "case MSP2_BATTERY_PROFILE:",
        str(MSP.relative_to(ROOT)),
        errors,
    )

    require(
        msp,
        "case MSP2_GET_VTX_DEVICE_STATUS:",
        str(MSP.relative_to(ROOT)),
        errors,
    )

    if errors:
        print("Configurator contract check failed:", file=sys.stderr)
        for error in errors:
            print(f" - {error}", file=sys.stderr)
        return 1

    print("Configurator contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
