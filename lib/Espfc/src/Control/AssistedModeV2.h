#pragma once

// -----------------------------------------------------------------------------
// Assisted-mode V2 feature selection.
//
// ANGLE V2 is now the only Angle controller and is always compiled.
// AltHold V2 and LAND V2 remain feature-gated until their production
// activation is completed.
//
// ESPFC_ASSISTED_V2_ACTIVE enables the production AltHold/LAND path.
// A motor-driving AltHold/LAND build must explicitly acknowledge actuator
// authority through ESPFC_ASSISTED_V2_OUTPUT_ACK.
// -----------------------------------------------------------------------------



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
