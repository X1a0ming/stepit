#include <stepit/debugging/dummy_control.h>

namespace stepit {
STEPIT_REGISTER_CTRLINPUT(dummy, kMinPriority, []() { return std::make_unique<DummyControl>(); });
}  // namespace stepit
