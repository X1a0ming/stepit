#include <stepit/control_input.h>
#include <stepit/policy_neuro/subscriber_action.h>

namespace stepit {
namespace neuro_policy {
// clang-format off
const std::map<std::string, SubscriberAction> kSubscriberActionMap = {
    {"EnableSubscriber",  SubscriberAction::kEnableSubscriber},
    {"DisableSubscriber", SubscriberAction::kDisableSubscriber},
    {"SwitchSubscriber", SubscriberAction::kSwitchSubscriber},
};
// clang-format on
}  // namespace neuro_policy
}  // namespace stepit
