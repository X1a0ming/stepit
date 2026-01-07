#include <stepit/policy_neuro/roll_pitch_source.h>

namespace stepit {
namespace neuro_policy {
RollPitchSource::RollPitchSource(const PolicySpec &, const std::string &) {
  roll_pitch_id_ = registerProvision("roll_pitch", 2);
}

bool RollPitchSource::update(const LowState &low_state, ControlRequests &requests, FieldMap &result) {
  result[roll_pitch_id_] = Arr2f{low_state.imu.rpy[0], low_state.imu.rpy[1]};
  return true;
}

STEPIT_REGISTER_FIELD_SOURCE(roll_pitch, kDefPriority, FieldSource::make<RollPitchSource>);
STEPIT_REGISTER_SOURCE_OF_FIELD(roll_pitch, kDefPriority, FieldSource::make<RollPitchSource>);
}  // namespace neuro_policy
}  // namespace stepit
