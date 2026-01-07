#include <unistd.h>
#include <csignal>

#include <stepit/spin.h>

namespace stepit {
volatile std::sig_atomic_t CatchSigIntSpin::sigint_received_ = 0;

int spin() {
  STEPIT_REGISTER_SPIN(catch_sigint_spin, kMinPriority + 1, [] { return std::make_unique<CatchSigIntSpin>(); });
  return SpinReg::make("")->spin();
}
}  // namespace stepit
