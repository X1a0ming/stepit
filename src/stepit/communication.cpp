#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <cmath>

#include <stepit/communication.h>
#include <stepit/logging.h>

namespace stepit {
/**
 * @brief Sets the CPU affinity for a specific thread or process.
 *
 * This function attempts to pin the execution of the thread or process identified by `pid`
 * to the specific CPU core identified by `cpuid`.
 *
 * @param pid The process ID (or thread ID) to set affinity for. If 0, the calling thread is used.
 * @param cpuid The logical ID of the CPU core to bind the thread to.
 *
 * @return The CPU ID actually set if successful, or -1 if the operation failed or the
 *         requested CPU ID was invalid.
 */
int setThreadCPU(pid_t pid, long cpuid) {
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(cpuid, &mask);
  int status = sched_setaffinity(pid, sizeof(mask), &mask);
  if (status) {
    STEPIT_WARN("Failed to set thread affinity (error code: {}).", status);
    return -1;
  }
  sched_getaffinity(pid, sizeof(mask), &mask);
  long num_cpus = sysconf(_SC_NPROCESSORS_CONF);
  if (cpuid >= num_cpus) {
    STEPIT_WARN("Invalid CPU ID '{}'.", cpuid);
    return -1;
  }
  for (int i{}; i < num_cpus; ++i) {
    if (CPU_ISSET(i, &mask)) return i;
  }
  return -1;
}

/**
 * @brief Sets the scheduling policy of a specific thread to real-time with maximum priority.
 *
 * This function attempts to change the scheduling policy of the provided thread to SCHED_RR
 * and sets its priority to the maximum value allowed for that policy.
 *
 * @param thread The POSIX thread identifier (pthread_t) of the thread to configure.
 * @return int The new scheduling priority of the thread on success, or -1 if setting the parameters failed.
 */
int setThreadRT(pthread_t thread) {
  int policy = SCHED_RR;
  sched_param param{sched_get_priority_max(policy)};
  int status = pthread_setschedparam(thread, policy, &param);
  if (status) {
    STEPIT_WARN("Failed to set thread priority (error code: {}).", status);
    return -1;
  }
  pthread_getschedparam(thread, &policy, &param);
  return param.sched_priority;
}

Communication::Communication(const std::string &robot_type)
    : api_(RobotApi::make(robot_type)),
      dof_{api_->getDoF()},
      comm_freq_{api_->getCommFreq()},
      low_state_msg_(api_->getDoF(), api_->getNumLegs()),
      low_cmd_msg_(api_->getDoF()) {
  STEPIT_ASSERT_EQ(spec().dof, dof(), "Ambiguous number of joints.");
  STEPIT_ASSERT_EQ(spec().foot_names.size(), api_->getNumLegs(), "Ambiguous number of legs.");
  api_->getControl(true);
}

LowState Communication::getLowState() {
  std::lock_guard<std::mutex> lock(comm_mtx_);
  return low_state_msg_;
}

void Communication::startCommunicationThread() {
  if (comm_thread_.joinable()) {  // thread has already started
    if (communicating_) communicating_ = false;
    comm_thread_.join();
  }

  active_      = false;
  comm_thread_ = std::thread([this] { communicationMainLoop(); });

  while (comm_tid_ < 0) std::this_thread::sleep_for(USec(10));
  long comm_cpuid{-1};
  if (getenv("STEPIT_COMM_CPUID", comm_cpuid)) {
    setThreadCPU(comm_tid_, comm_cpuid);
  }
  int priority = setThreadRT(comm_thread_.native_handle());
  if (priority > 0) STEPIT_LOG("Set communication thread priority to {}.", priority);
}

void Communication::stopCommunicationThread() {
  communicating_ = false;
  connected_     = false;
  if (comm_thread_.joinable()) {
    comm_thread_.join();
    STEPIT_LOG("Disconnected from Robot.");
  }
  comm_tid_ = -1;
}

void Communication::communicationMainLoop() {
  comm_tid_      = gettid();
  communicating_ = true;
  Rate rate(comm_freq_);
  bool printed1 = false, printed2 = false;

  while (communicating_) {
    communicationEvent();
    if (not printed1 and connected_) {
      STEPIT_LOG("Robot connected.");
      printed1 = printed2 = true;
    }
    if (not printed2 and not connected_) {
      STEPIT_CRIT("Robot not connected.");
      printed2 = true;
    }
    rate.sleep();
  }
}

void Communication::communicationEvent() {
  api_->recv();
  {
    std::lock_guard<std::mutex> _(comm_mtx_);
    api_->getRecv(low_state_msg_);
    connected_ = low_state_msg_.tick != 0;
    if (frozen_) active_ = false;
    if (not active_) setDampedMode();
    api_->setSend(low_cmd_msg_);
  }
  if (active_ or not spec().auto_damped_mode) api_->send();
  {
    std::lock_guard<std::mutex> _(comm_mtx_);
    publisher::publishLowLevel(spec(), low_state_msg_, low_cmd_msg_);
  }
  comm_tick_.store(comm_tick_ + 1, std::memory_order_release);
}

void Communication::waitForCommunicationTick(std::size_t tick) const {
  while (comm_tick_.load(std::memory_order_acquire) < tick) {
    std::this_thread::sleep_for(USec(10));
  }
}

void Communication::setDampedMode() {
  for (auto &motor_cmd : low_cmd_msg_) {
    motor_cmd.q   = 0.;
    motor_cmd.dq  = 0.;
    motor_cmd.tor = 0.;
    motor_cmd.Kp  = 0.;
    motor_cmd.Kd  = spec().kd_damped_mode;
  }
}
}  // namespace stepit
