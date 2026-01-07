#include <stepit/policy.h>
#include <stepit/publisher.h>
#include <stepit/registry.h>
#include <stepit/robot.h>

namespace stepit {
template class RegistrySingleton<ControlInput>;
template class RegistrySingleton<Policy, const RobotSpec &, const std::string &>;
template class RegistrySingleton<Publisher>;
template class RegistrySingleton<RobotApi>;
}  // namespace stepit
