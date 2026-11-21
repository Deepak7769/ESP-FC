# Betaflight source baseline — step 2

This development branch pins **official Betaflight firmware source** separately
from the legacy ESP-FC firmware. It does not replace the existing flight-control
implementation, and it does not yet expand the supported hardware in any binary.

## Provenance (fixed and reproducible)

- Upstream project: https://github.com/betaflight/betaflight
- Stable upstream release tag: `2026.6.2`
- Verified release commit: `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`
- Checkout path: `upstream/betaflight/` (Git submodule)
- Upstream license: GPL-3.0
- Existing ESP-FC source stays in `src/`, `lib/Espfc/`, and its own
  `lib/betaflight/` integration. **Those files are not replaced by the upstream
  submodule.**

The official release is deliberately pinned to a *commit*, rather than a
moving branch or GitHub's latest version. Do not update it implicitly.

## Fetch and inspect

```sh
git clone https://github.com/Deepak7769/ESP-FC.git
cd ESP-FC
git switch development/betaflight-full-compatibility
git submodule update --init upstream/betaflight
git -C upstream/betaflight rev-parse HEAD
# Expected: e0b7bb01b17b21351057e9ead2d1ab39dd44fa16
```

The separate `Betaflight upstream baseline` GitHub Actions workflow verifies
that the gitlink, configured upstream URL, checked-out SHA, and source layout
match this documented version. The existing PlatformIO workflow continues to
test the legacy ESP-FC firmware independently.

## Architecture boundary and scope

- **Upstream Betaflight**: reference implementation, MCU/platform drivers,
  receiver and motor protocols, feature configuration, MSP, and board targets.
- **Existing ESP-FC**: current working code and custom assisted-mode logic.
  Its firmware images remain ESP-FC builds until an explicit migration.
- **Configurator**: availability of a UI control does not imply the firmware
  implements the setting, or that a particular board has the necessary hardware.
- **Supported hardware**: a board/driver is only supported here after its
  firmware target is integrated, compiled, and hardware-validated. The pinned
  upstream source alone does **not** make ESP-FC binaries support every
  Betaflight-compatible target.
- **Flight safety**: do not ship or flash experimental migration builds on
  an aircraft before actuator, failsafe, timing, and sensor checks pass.

## Next discrete step

Build one official Betaflight host/simulator or documented hardware target
**from this pinned source**, in isolation from ESP-FC. Record target/toolchain
constraints and prove the build; only then begin a per-feature/per-hardware
migration and compatibility matrix. No unsupported MSP commands or devices
should be advertised as functional.
