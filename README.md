# Haptic BLDC Library
### Standalone Arduino library for smartknob-like devices and general digitally controlled knob user interfaces.

A simple, standalone library for implementing basic haptic-controlled knobs as user-interfaces.
Based on "Drive by wire" interfaces and @scottbez1 SmartKnob project.

## Features 
* Easy to setup (minimal use just requires information about motor, driver IC, and encoder)
* Fully configurable (all PID controllers, haptic parameters and configuration are user-tunable during operation)
* High efficiency due to use of FOC for motor control (convenient to use with devices plugged into USB or battery)
* Wide variety of hardware support due to underlying SimpleFOC abstraction.

## Example code
```cpp
#include SimpleFOC.h
#include HapticBLDC.h

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, EN);
MagneticEncoderSPI encoder = MagneticEncoderSPI();
HapticInterface haptic = HapticInterface(&motor);

void setup(){
    encoder.init();
    driver.init();

    motor.linkSensor(&encoder);
    motor.linkDriver(&driver);
    motor.init();
    motor.initFOC();

    haptic.init();
    haptic.haptic_config->detent_num = 6; //6 positions for knob

}

void loop(){
    haptic.haptic_loop();
}
```