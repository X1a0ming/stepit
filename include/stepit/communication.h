#ifndef STEPIT_COMMUNICATION_H_
#define STEPIT_COMMUNICATION_H_

#include <atomic>
#include <mutex>
#include <thread>

#include <stepit/publisher.h>

namespace stepit {
int setThreadCPU(pid_t pid, long cpuid);
int setThreadRT(pthread_t thread);

class Communication {
 public:
  explicit Communication(const std::string &robot_type);
  virtual ~Communication() { stopCommunicationThread(); }

  const RobotApiPtr &api() const { return api_; }
  const RobotSpec &spec() const { return api_->getSpec(); }
  std::size_t dof() const { return dof_; }
  LowState getLowState();

  void startCommunicationThread();
  void stopCommunicationThread();

 private:
  void communicationMainLoop();
  void communicationEvent();
  void setDampedMode();

  RobotApiPtr api_;
  std::thread comm_thread_;
  std::atomic<pid_t> comm_tid_{-1};

  std::atomic<bool> communicating_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> frozen_{false};

  std::size_t dof_;
  std::size_t comm_freq_;
  std::atomic<std::size_t> comm_tick_{0};

 protected:
  bool isCommunicating() const { return communicating_; }
  bool isConnected() const { return connected_; }
  bool isFrozen() const { return frozen_; }
  bool isActive() const { return active_; }
  std::size_t getCommunicationFreq() const { return comm_freq_; }
  std::size_t getCommunicationTick() const { return comm_tick_; }
  void waitForCommunicationTick(std::size_t tick) const;
  void setActive(bool enabled = true) { active_ = enabled; }
  void setFrozen(bool enabled = true) { frozen_ = enabled; }

  std::mutex comm_mtx_;  // for low_cmd_msg_ & low_state_msg_
  LowState low_state_msg_;
  LowCmd low_cmd_msg_;
};
}  // namespace stepit

#endif  // STEPIT_COMMUNICATION_H_
