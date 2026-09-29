#pragma once

#include "Model.h"

namespace Espfc::Control {

class Controller;

#if defined(ESPFC_ADVANCED_MODES_ACTIVE_TEST) && !defined(ESPFC_SAFE_BENCH_BUILD)
#error "ESPFC_ADVANCED_MODES_ACTIVE_TEST requires ESPFC_SAFE_BENCH_BUILD"
#endif

#if defined(ESPFC_ADVANCED_MODES_ACTIVE_TEST)
void applyAdvancedModesBench(Model& model, const Controller& controller);
#endif

} // namespace Espfc::Control
