
#include <Arduino.h>
#include "./my_functions.h"
#include "Adafruit_GC9A01A.h"
#include "Wire.h"
#include <Adafruit_INA219.h>
#include <SimpleFOC.h>
#include "haptic.h"
/*-----------------------------------------------
  LCD Display
-----------------------------------------------*/
#define MISO   -1
#define SCLK   4
#define MOSI   5
#define RES    6
#define DC     7
#define CS     15
Adafruit_GC9A01A tft(CS, DC, MOSI, SCLK, RES, MISO);


/*-----------------------------------------------
  Current Sensor
-----------------------------------------------*/
//the current sensor
#define ma_sensor_addr_red_C (0x40) //Red motor wirie - Phase C
#define ma_sensor_addr_yellow_A (0x41) //Yellow motor wire - Phase A
#define ma_sensor_addr_blue_B (0x44) //Blue motor wire  - phase B
TwoWire currentSenseI2C(0);
Adafruit_INA219 current_sense_yellow_A(ma_sensor_addr_yellow_A);
Adafruit_INA219 current_sense_blue_B(ma_sensor_addr_blue_B);
Adafruit_INA219 current_sense_red_C(ma_sensor_addr_red_C);
#define cur_SDA_PIN 2
#define cur_SCL_PIN 1

float current_mA = 0;

// current sensor
InlineCurrentSense current_sense = InlineCurrentSense(&current_sense_red_C, &current_sense_blue_B, &current_sense_yellow_A);


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


BLDCMotor motor = BLDCMotor(7);//, 7.8, 270);


/*-----------------------------------------------
  Position Sensor
-----------------------------------------------*/
MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
TwoWire positionSensorI2C(1);
#define pos_SDA_PIN 17
#define pos_SCL_PIN 16

PIDController P_haptic(0.7,0,0,100000,5);
float shaftVelocity = 60;
// attractor angle variable
float attract_angle = 0;
// distance between attraction points
float attractor_distance = (15 * 3.14159265359f)/180.0; // dimp each 45 degrees
float findAttractor(float current_angle){
  return round(current_angle/attractor_distance)*attractor_distance;
}
float h = 0;
float outerRadius = 0;
float innerRadius = 0;
uint16_t screenBuffer[240 * 240];
void setup() {
  Serial.begin(115200);
  positionSensorI2C.begin(pos_SDA_PIN, pos_SCL_PIN);
  as5600.init(&positionSensorI2C);
  motor.linkSensor(&as5600);
  
  driver.voltage_power_supply = 5;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 5;
  // limiting motor movements
  motor.phase_resistance = 7.8; // [Ohm]
  motor.current_limit = 1.2;   // [Amps] - if phase resistance defined
  motor.voltage_limit = 5;   // [V] - if phase resistance not defined
  motor.velocity_limit = 20; // [rad/s] cca 50rpm
  motor.torque_controller = TorqueControlType::voltage;//TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  
  motor.init();
  motor.initFOC();
  motor.target = 0;
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);

  
  tft.begin();
  tft.fillScreen(GC9A01A_BLACK);
  drawTicks(tft);
  h = tft.height();
  outerRadius = h/2;
  innerRadius = (h/2) * 0.5;
}
float previousShaftAngle = -1;
void loop() {
  motor.loopFOC();
  motor.move(P_haptic(attract_angle - motor.shaft_angle));
  attract_angle = findAttractor(motor.shaft_angle);
  //float radians = angle * M_PI / 180.0f;
  if (previousShaftAngle > -1) {
    float x1 = (innerRadius * cos(previousShaftAngle)) + outerRadius;
    float x2 = (outerRadius * cos(previousShaftAngle)) + outerRadius;
    float y1 = (innerRadius * sin(previousShaftAngle)) + outerRadius;
    float y2 = (outerRadius * sin(previousShaftAngle)) + outerRadius;

    tft.drawLine(x1, y1, x2, y2, GC9A01A_BLACK);  
  }
  previousShaftAngle = motor.shaft_angle;
  float a_x1 = (innerRadius * cos(motor.shaft_angle)) + outerRadius;
  float a_x2 = (outerRadius * cos(motor.shaft_angle)) + outerRadius;
  float a_y1 = (innerRadius * sin(motor.shaft_angle)) + outerRadius;
  float a_y2 = (outerRadius * sin(motor.shaft_angle)) + outerRadius;

  tft.drawLine(a_x1, a_y1, a_x2, a_y2, GC9A01A_WHITE);
}