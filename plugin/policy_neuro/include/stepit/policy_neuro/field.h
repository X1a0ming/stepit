#ifndef STEPIT_NEURO_POLICY_FIELD_H_
#define STEPIT_NEURO_POLICY_FIELD_H_

#include <map>
#include <mutex>
#include <vector>

#include <stepit/control_input.h>
#include <stepit/policy.h>
#include <stepit/registry.h>
#include <stepit/utils.h>

namespace stepit {
namespace neuro_policy {
using FieldId    = std::size_t;
using FieldMap   = std::map<FieldId, ArrXf>;
using FieldIdVec = std::vector<FieldId>;

class FieldSource {
 public:
  using Ptr = std::unique_ptr<FieldSource>;
  using Reg = RegistrySingleton<FieldSource, const PolicySpec &, const std::string &>;

  FieldSource()          = default;
  virtual ~FieldSource() = default;
  virtual void initFieldProperties() {}

  virtual bool reset() { return true; }
  virtual bool update(const LowState &low_state, ControlRequests &requests, FieldMap &result) = 0;
  virtual void postUpdate(const FieldMap &field_map) {}
  virtual void exit() {}

  const std::set<FieldId> &requirements() const { return requirements_; }
  const std::set<FieldId> &provisions() const { return provisions_; }

  template <typename Derived>
  static Ptr make(const PolicySpec &policy_spec, const std::string &home_dir) {
    return std::make_unique<Derived>(policy_spec, home_dir);
  }

 protected:
  FieldId registerRequirement(const std::string &field_name);
  FieldId registerProvision(const std::string &field_name, std::uint32_t size);

  std::set<FieldId> requirements_, provisions_;
};

using FieldSourcePtr = FieldSource::Ptr;
using FieldSourceReg = FieldSource::Reg;

// Singleton registry to manage observations
class FieldManager {
 public:
  FieldManager(const FieldManager &)            = delete;
  FieldManager &operator=(const FieldManager &) = delete;
  static FieldManager &instance();

  using SourceRegistry = Registry<FieldSource, const PolicySpec &, const std::string &>;
  auto registerSource(const std::string &name, int priority, SourceRegistry::Factory factory)
      -> SourceRegistry::Registration;
  auto makeSource(const std::string &name, const PolicySpec &policy_spec, const std::string &home_dir)
      -> FieldSourcePtr;

  FieldId registerField(const std::string &name, std::uint32_t size);
  FieldId getFieldId(const std::string &name);
  const std::string &getFieldName(FieldId id) const;
  std::uint32_t getFieldSize(FieldId id) const;
  void setFieldSize(FieldId id, std::uint32_t size);

 private:
  FieldManager() = default;
  mutable std::recursive_mutex mutex_;
  SourceRegistry source_registry_;
  std::map<std::string, FieldId> name_to_id_;
  std::vector<std::string> id_to_name_;
  std::vector<std::uint32_t> id_to_size_;
  FieldId next_id_{};
};

// Helper accessors
inline FieldManager &fieldManager() { return FieldManager::instance(); }
inline FieldId registerField(const std::string &name, std::uint32_t size) {
  return fieldManager().registerField(name, size);
}
inline FieldId getFieldId(const std::string &name) { return fieldManager().getFieldId(name); }
inline const std::string &getFieldName(FieldId id) { return fieldManager().getFieldName(id); }
inline std::uint32_t getFieldSize(FieldId id) { return fieldManager().getFieldSize(id); }
inline void setFieldSize(FieldId id, std::uint32_t size) { fieldManager().setFieldSize(id, size); }
inline FieldSourcePtr makeSourceOfField(const std::string &field_name, const PolicySpec &policy_spec,
                                        const std::string &home_dir) {
  return fieldManager().makeSource(field_name, policy_spec, home_dir);
}

void parseFieldIds(const YAML::Node &node, FieldIdVec &result);
void assembleFields(const FieldMap &field_map, const FieldIdVec &field_ids, rArrXf result);
void splitFields(cArrXf data, const FieldIdVec &field_ids, FieldMap &result);
}  // namespace neuro_policy

extern template class RegistrySingleton<neuro_policy::FieldSource, const PolicySpec &, const std::string &>;
}  // namespace stepit

#define STEPIT_REGISTER_FIELD_SOURCE(name, priority, factory)                                                   \
  static auto _field_source_class_##name##_registration = ::stepit::neuro_policy::FieldSourceReg::Registration( \
      #name, priority, factory)
#define STEPIT_REGISTER_SOURCE_OF_FIELD(field_name, priority, factory)                           \
  static auto _field_##field_name##_source_registration = ::stepit::neuro_policy::fieldManager() \
                                                              .registerSource(#field_name, priority, factory)

#endif  // STEPIT_NEURO_POLICY_FIELD_H_
