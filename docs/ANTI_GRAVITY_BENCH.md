# Anti-Gravity non-actuating bench validation

This is the authoritative Anti-Gravity controller implementation validation stage.

## Safety boundary

The active validation path is compiled only when both:

- `ESPFC_ANTI_GRAVITY_ACTIVE`
- `ESPFC_SAFE_BENCH_BUILD`

are defined. Compilation fails if the active-test macro is used without the
safe-bench macro.

`ESPFC_SAFE_BENCH_BUILD` keeps the mixer math running but prevents the motor
and servo drivers from being initialized or attached. This validation firmware
is therefore intended only for non-actuating bench observation, not flight.

The ordinary ESP32 firmware remains non-authoritative for Anti-Gravity at this milestone. The controller implementation is no longer a separate shadow algorithm: the same Anti-Gravity calculation is used, and only the compile-time authority gate decides whether its P/I demand reaches the rate PID.

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
existing fixed cutoff/P-gain constants already used by the shadow diagnostics.

## DEBUG_ANTI_GRAVITY

The existing fields remain:

| field | meaning |
| ---: | --- |
| debug[0] | raw throttle derivative x100 |
| debug[1] | filtered throttle derivative x100 |
| debug[2] | pitch-equivalent I gain multiplier x1000 |
| debug[3] | pitch P gain multiplier x1000 |

The active bench build adds:

| field | meaning |
| ---: | --- |
| debug[4] | Anti-Gravity rate-PID application active (0/1) |
| debug[5] | gain-scaled filtered derivative x100 |
| debug[6] | additive I accelerator x1000 |

A useful first validation is to confirm that `debug[4]` changes only during
an Anti-Gravity transient, that debug gain values return toward their neutral
state after the transient, and that the ESC/servo drivers remain unattached.
