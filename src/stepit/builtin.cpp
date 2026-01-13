#include <stepit/publisher.h>
#include <stepit/spin.h>

namespace stepit {
STEPIT_REGISTER_PUBLISHER(dummy, kMinPriority, Publisher::makeDerived<Publisher>);
STEPIT_REGISTER_SPIN(catch_sigint_spin, kMinPriority + 1, Spin::makeDerived<WaitForSigInt>);
}  // namespace stepit
