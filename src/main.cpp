// This sketch is for the RP2040 and ILI9341 TFT display.
// Other processors may work if they have sufficient RAM for
// a full screen displayAngleHistory (240 x 320 x 2 = 153,600 bytes).

// In this example 2 sprites are used to create DMA toggle
// buffers. Each sprite is half the screen size, this allows
// graphics to be rendered in one sprite at the same time
// as the other sprite is being sent to the screen.

// RP2040 typically runs at 45-48 fps

// Created by Bodmer 20/04/2021 as an example for:
// https://github.com/Bodmer/TFT_eSPI
#include <Arduino.h>
#include "./my_functions.h"
#include "Adafruit_GC9A01A.h"
#include "Wire.h"
#include <Adafruit_INA219.h>
#include <SimpleFOC.h>
#include "haptic.h"
#include <cmath>
#include <TFT_eSPI.h>
#include <random>
#include <iostream>
#include <string>
#include <cust_haptic.h>
#include "../../lib/Arduino-FOC-drivers/src/encoders/smoothing/SmoothingSensor.h"


/*-----------------------------------------------
	Position Sensor
-----------------------------------------------*/
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);
TwoWire positionSensorI2C(1);
#define pos_SDA_PIN 17
#define pos_SCL_PIN 16


/*-----------------------------------------------
	Motor Driver
-----------------------------------------------*/
#define UL 14 //red motor wire
#define UH 13 //red motor wire  - phase C
#define VL 12 //Blue motor wire
#define VH 21 //Blue motor wire -   pahse B
#define WL 47 //yellow motor wire
#define WH 48 //yellow motor wire - phase A
BLDCDriver6PWM driver = BLDCDriver6PWM(WH, WL, VH, VL, UH, UL);
//sparkfun motor
BLDCMotor motor = BLDCMotor(7);


SmoothingSensor smooth = SmoothingSensor(encoder, motor);
Cust_HapticInterface haptic = Cust_HapticInterface(&motor);

void setup(){
  	Serial.begin(115200);
	positionSensorI2C.begin(pos_SDA_PIN, pos_SCL_PIN);
	encoder.init(&positionSensorI2C);

	driver.voltage_power_supply = 5;
	driver.voltage_limit = 5;
	driver.init();
	motor.linkSensor(&smooth);
	motor.linkDriver(&driver);
	motor.controller = MotionControlType::torque;
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motor.current_limit = 1.22;
	motor.init();
	motor.initFOC();
	Serial.println("Before haptic init");
	haptic.init();
	Serial.println("after haptic init");
	delay(1500);
}

void loop(){
	Serial.println("before correct pid");
	//haptic.haptic_loop(); // Adjust PID (Derivative Gain)
	haptic.correct_pid(); // Adjust PID (Derivative Gain)
    haptic.find_detent(); // Calculate attraction angle depending on configured distance position.
    haptic.haptic_target(); // PID Command
}
