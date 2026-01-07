#ifndef STEPIT_CSV_PUBLISHER_H_
#define STEPIT_CSV_PUBLISHER_H_

#include <fstream>

#include <stepit/publisher.h>

namespace stepit {
class CsvPublisher : public Publisher {
 public:
  CsvPublisher();
  ~CsvPublisher() override;
  void publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) override;

 private:
  std::ofstream file_;
  bool header_written_{false};
};
}  // namespace stepit

#endif  // STEPIT_CSV_PUBLISHER_H_
