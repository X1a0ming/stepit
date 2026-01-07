#include <stepit/joystick/keymap.h>

namespace stepit {
namespace joystick {
Keymap::Keymap(const YAML::Node &config) {
  yml::setTo(config, "A", A);
  yml::setTo(config, "B", B);
  yml::setTo(config, "X", X);
  yml::setTo(config, "Y", Y);
  yml::setTo(config, "LB", LB);
  yml::setTo(config, "RB", RB);
  yml::setTo(config, "Select", Select);
  yml::setTo(config, "Start", Start);
  yml::setTo(config, "LAS", LAS);
  yml::setTo(config, "RAS", RAS);
  yml::setTo(config, "Up", Up);
  yml::setTo(config, "Down", Down);
  yml::setTo(config, "Left", Left);
  yml::setTo(config, "Right", Right);

  yml::setTo(config, "las_x", las_x);
  yml::setTo(config, "las_y", las_y);
  yml::setTo(config, "ras_x", ras_x);
  yml::setTo(config, "ras_y", ras_y);
  yml::setTo(config, "lt", lt);
  yml::setTo(config, "rt", rt);
}
}  // namespace joystick
}  // namespace stepit
