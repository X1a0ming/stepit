#include <unistd.h>
#include <csignal>

#include <stepit/spin.h>

namespace stepit {
volatile std::sig_atomic_t WaitForSigInt::sigint_received_ = 0;

int spin() { return Spin::make("")->spin(); }
}  // namespace stepit
