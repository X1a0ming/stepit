#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <stepit/logging.h>
#include <stepit/utils.h>

namespace stepit {
std::string getTimeStampStr() {
  auto now  = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm broken_down{};
  localtime_r(&time, &broken_down);

  std::ostringstream oss;
  oss << std::put_time(&broken_down, "%F %T");

  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}

LoggingModule &LoggingModule::withoutTimestamp() {
  next_timestamp_ = false;
  return *this;
}

LoggingModule &LoggingModule::setVerbosity(VerbosityLevel level) {
  verbosity_ = level;
  return *this;
}

LoggingModule &LoggingModule::setNextVerbosity(VerbosityLevel level) {
  next_verbosity_ = level;
  return *this;
}

LoggingModule &LoggingModule::setNextTextStyle(const char *style) {
  next_text_style_ = style;
  return *this;
}

void LoggingModule::clearStyle() {
  next_timestamp_  = true;
  next_verbosity_  = kInfo;
  next_text_style_ = "";
}

LoggingModule &LoggingModule::instance() {
  static LoggingModule *logging_module{nullptr};
  if (logging_module == nullptr) {
    logging_module = new LoggingModule();
    long verbosity{};
    if (getenv("STEPIT_VERBOSITY", verbosity)) {
      logging_module->setVerbosity(static_cast<VerbosityLevel>(verbosity));
    }
  }
  return *logging_module;
}

void LoggingModule::logImpl(const std::string &info) {
  if (info.empty()) {
    std::cout << std::endl;
  } else {
    if (next_timestamp_) {
      std::cout << "[" << getTimeStampStr() << "] ";
    }
    if (next_text_style_.empty()) {
      std::cout << info << std::endl;
    } else {
      std::cout << next_text_style_ << info << llu::kClear << std::endl;
    }
  }

  clearStyle();
}
}  // namespace stepit
