#ifndef STEPIT_USB_JOYSTICK_H_
#define STEPIT_USB_JOYSTICK_H_

#include <libgamepad.hpp>

#include <stepit/joystick/joystick.h>
#include <stepit/joystick/keymap.h>

namespace stepit {
namespace joystick {
class UsbJoystick final : public Joystick {
 public:
  explicit UsbJoystick();
  ~UsbJoystick() override;
  bool connected() const override { return connected_; }
  void getState(State &state) override;

 private:
  void connectHandler(std::shared_ptr<gamepad::device> dev);
  void disconnectHandler(std::shared_ptr<gamepad::device> dev);
  void axisHandler(std::shared_ptr<gamepad::device> dev);
  void buttonHandler(std::shared_ptr<gamepad::device> dev);

  long id_{-1}, rid_{-1};

  std::mutex mutex_;
  Slots slots_;
  Keymap keymap_;
  std::shared_ptr<gamepad::hook> hook_{nullptr};
  std::atomic<bool> connected_{false};
  TimePoint stamp_;
};
}  // namespace joystick
}  // namespace stepit

#endif  // STEPIT_USB_JOYSTICK_H_
