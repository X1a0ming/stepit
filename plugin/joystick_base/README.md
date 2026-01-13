# joystick_base

StepIt plugin providing joystick interface and control.

Provided interfaces:

- `stepit::joystick::Joystick`

Provided factories:

- `stepit::ControlInput`: `joystick`

Key bindings:
- `LAS`[^1] + `RAS`[^2]: Set the agent to the frozen mode, disabling all controls.
- `LT`[^3] + `X`: Resume the agent from the frozen mode.
- `LT` + `A`: Stand up from lying position, or lie down from standing position.
- `LT` + `B`: Enable the locomotion policy, or switch to standing if already enabled.
- `LT` + `Y`: Cycle the current active locomotion policy.

[^1]: `LAS` refers to pressing the left analog stick.

[^2]: `RAS` refers to pressing the right analog stick.

[^3]: `LT` refers to the left trigger (or `L2`).
