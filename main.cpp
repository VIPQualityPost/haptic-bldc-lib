#include <Arduino.h>
#include <SimpleFOC.h>
#include "encoders/smoothing/SmoothingSensor.h"
#include <haptic.h>



BLDCMotor motor = BLDCMotor(blcd_pp, bldc_pr, bldc_KV, bldc_Li);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_U, PIN_V, PIN_W, PIN_EN_U, PIN_EN_V, PIN_EN_W);
MagneticSensorMT6701SSI encoder(PIN_MT_CSN);
SmoothingSensor smooth = SmoothingSensor(encoder, motor);
HapticInterface haptic = HapticInterface(&motor);

void setup(){

  Serial.begin(115200);
  SPIClass* spi = new SPIClass(FSPI);
  spi->begin(PIN_MT_CLOCK, PIN_MT_DATA, -1, PIN_MT_CSN);
  encoder.init(spi);
  
  driver.voltage_power_supply = driver_supply;
  driver.voltage_limit = driver_voltage_limit;
  driver.init();
  motor.linkSensor(&smooth);
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.current_limit = 1.22;
  motor.init();
  motor.initFOC();
  haptic.init();
  delay(1500);
}

void loop(){
haptic.haptic_loop();
}