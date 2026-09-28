# Anti-Gravity non-actuating bench validation

This is the authoritative Anti-Gravity controller implementation validation stage.

## Safety boundary

The active validation path is compiled when:

- `ESPFC_SAFE_BENCH_BUILD`

is defined.

`ESPFC_SAFE_BENCH_BUILD` is the single compile-time authority gate for
Anti-Gravity validation. It allows the calculated Anti-Gravity P/I demand to
reach the rate PID while keeping the mixer math running and preventing the
motor and servo drivers from being initialized or attached. This validation
firmware is therefore intended only for non-actuating bench observation, not
flight.

The ordinary ESP32 firmware remains non-authoritative for Anti-Gravity at this
milestone. Runtime enablement still comes from the Anti-Gravity feature or mode;
the bench flag decides whether that demand is permitted to reach the rate PID.

## Validation build

PlatformIO environment:

```
esp32_antigravity_bench
```

GitHub Actions publishes the matching firmware as:

```
esp32_antigravity_validation_<commit>
```

## Controller behavior under the bench gate

When Anti-Gravity is enabled by either the Anti-Gravity feature or mode and
manual throttle owns the vertical output:

- throttle-transient demand is calculated with the existing PT2-filtered
  Betaflight-style detector;
- roll and pitch receive the calculated P boost;
- roll and pitch receive the additive I-term accelerator;
- yaw receives neither Anti-Gravity P boost nor I acceleration;
- Anti-Gravity yields when Assisted V2 owns vertical thrust or receiver input
  is unhealthy.

The active bench path uses the same configured `antiGravityGain` and the
existing fixed cutoff/P-gain constants already used by the controller diagnostics.

## DEBUG_ANTI_GRAVITY

The existing fields remain:

| field | meaning |
| ---: | --- |
| debug[0] | raw throttle derivative x100 |
| debug[1] | filtered throttle derivative x100 |
| debug[2] | pitch-equivalent I gain multiplier x1000 |
| debug[3] | pitch P gain multiplier x1000 |

The SAFE_BENCH authority path adds:

| field | meaning |
| ---: | --- |
| debug[4] | Anti-Gravity rate-PID application active (0/1) |
| debug[5] | gain-scaled filtered derivative x100 |
| debug[6] | additive I accelerator x1000 |

A useful first validation is to confirm that `debug[4]` changes only during
an Anti-Gravity transient, that debug gain values return toward their neutral
state after the transient, and that the ESC/servo drivers remain unattached.
