# PicoStep

PicoStep is an Arduino library for controlling stepper motors with RP2040-based boards using an H-bridge. Tested on an original Raspberry Pi Pico and an original Raspberry Pi Pico W, both with a L298N H-bridge.

This library depends on https://github.com/khoih-prog/RP2040_PWM/ for hardware pwm control.

## Installation

You can find it in the Arduino IDE library manager, or on their website: https://reference.arduino.cc/reference/en/libraries/picostep/

Alternatively, one can include the source files directly in a project within the limitations of the GPL-3.0 license.

## Supported features:

- Continuous rotation
- Bidirectional speed control
- Smooth, configurable acceleration
- Freewheeling (arm/disarm)
- Hardware PWM (using the [RP2040_PWM](https://github.com/khoih-prog/RP2040_PWM/) library)
- Automatic microstepping (the crossover point to full stepping is configurable)
- Beep (using low frequency PWM to make noise while stationary)

## Wishlist features:

- Positional control
- Step counting / odometer (also in continuous rotation mode)
- Alternative acceleration curves (currently hardcoded to be linear)
