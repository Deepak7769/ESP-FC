#pragma once

// -----------------------------------------------------------------------------
// Assisted-mode V2 compile-time feature selection.
//
// The legacy *_ACTIVE_TEST flags are retained for safe-bench regression builds.
// ESPFC_ASSISTED_V2_ACTIVE is the production-capable umbrella switch.
//
// Any motor-driving assisted-mode build must also define
// ESPFC_ASSISTED_V2_OUTPUT_ACK explicitly. This makes the transition from
// non-actuating validation to physical actuator authority deliberate.
// -----------------------------------------------------------------------------

#if defined(ESPFC_ANGLE_V2_ACTIVE_TEST) && \
    !defined(ESPFC_SAFE_BENCH_BUILD)
#error "ESPFC_ANGLE_V2_ACTIVE_TEST requires ESPFC_SAFE_BENCH_BUILD"
#endif

#if defined(ESPFC_ALTHOLD_V2_ACTIVE_TEST) && \
    !defined(ESPFC_SAFE_BENCH_BUILD)
#error "ESPFC_ALTHOLD_V2_ACTIVE_TEST requires ESPFC_SAFE_BENCH_BUILD"
#endif

#if defined(ESPFC_LAND_V2_ACTIVE_TEST) && \
    !defined(ESPFC_SAFE_BENCH_BUILD)
#error "ESPFC_LAND_V2_ACTIVE_TEST requires ESPFC_SAFE_BENCH_BUILD"
#endif

#if defined(ESPFC_ASSISTED_V2_ACTIVE) && \
    !defined(ESPFC_SAFE_BENCH_BUILD) && \
    !defined(ESPFC_ASSISTED_V2_OUTPUT_ACK)
#error "Motor-driving Assisted V2 builds require ESPFC_ASSISTED_V2_OUTPUT_ACK"
#endif

#if defined(ESPFC_ASSISTED_V2_ACTIVE) || \
    defined(ESPFC_ANGLE_V2_ACTIVE_TEST)
#define ESPFC_ANGLE_V2_ACTIVE 1
#endif

#if defined(ESPFC_ASSISTED_V2_ACTIVE) || \
    defined(ESPFC_ALTHOLD_V2_ACTIVE_TEST)
#define ESPFC_ALTHOLD_V2_ACTIVE 1
#endif

#if defined(ESPFC_ASSISTED_V2_ACTIVE) || \
    defined(ESPFC_LAND_V2_ACTIVE_TEST)
#define ESPFC_LAND_V2_ACTIVE 1
#endif

#if defined(ESPFC_LAND_V2_ACTIVE) && \
    !defined(ESPFC_ALTHOLD_V2_ACTIVE)
#error "LAND V2 requires AltHold V2"
#endif

#if defined(ESPFC_LAND_V2_ACTIVE) && \
    !defined(ESPFC_ANGLE_V2_ACTIVE)
#error "LAND V2 requires Angle V2"
#endif

// Channel index carrying a spring-centered vertical-stick command for AltHold.
// 3 == the normal throttle channel and preserves existing bench/test behavior.
// A production radio integration can override this with an unused AUX channel
// carrying the raw centered stick (the current project uses AUX3 / index 6).
#ifndef ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL
#define ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL 3
#endif

#if ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL < 0 || \
    ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL > 15
#error "ESPFC_ALTHOLD_V2_CENTERED_STICK_CHANNEL must be in range 0..15"
#endif
