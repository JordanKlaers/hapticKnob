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

/*-----------------------------------------------
	Display
-----------------------------------------------*/
#define DWIDTH  240
#define DHEIGHT 240
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite spr[4] = {TFT_eSprite(&tft), TFT_eSprite(&tft), TFT_eSprite(&tft), TFT_eSprite(&tft)};
uint16_t* sprPtr[1];

CircularBuffer displayAngleHistory;											//This displayAngleHistory stores the positions of the dot - by degree
CircularBuffer motorAngleHistory;
bool displayIsFluttering = false;										//variable to help stablize when the know is not spinning
bool motorIsFluttering = false;	
float motorShaftAngle = 0;


/*-----------------------------------------------
	Position Sensor
-----------------------------------------------*/
MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
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


/*-----------------------------------------------
	Haptic controller and things
-----------------------------------------------*/

PIDController P_haptic(0.8,0,0,100000,5);
float attract_angle = 0;
float attractor_distance = (17.1428571429 * 3.14159265359f)/180.0; // 21 attractors should be divisible by 7 and work with the poll count hopeuflly
float findAttractor(float current_angle){
	return round(current_angle/attractor_distance)*attractor_distance;
}


/*-----------------------------------------------
	Tasks
-----------------------------------------------*/
TaskHandle_t motorTaskHandle;
TaskHandle_t lcdTaskHandle;
void motorTask(void *pvParameters);
void lcdTask(void *pvParameters);
QueueHandle_t floatQueue;


void setup() {
	Serial.begin(115200);
	positionSensorI2C.begin(pos_SDA_PIN, pos_SCL_PIN);
	as5600.init(&positionSensorI2C);
	// Add some sample values
	displayAngleHistory.add(0.0f);
	displayAngleHistory.add(0.0f);
	displayAngleHistory.add(0.0f);
	motorAngleHistory.add(0.0f);
	motorAngleHistory.add(0.0f);
	motorAngleHistory.add(0.0f);

	motor.linkSensor(&as5600);
	driver.voltage_power_supply = 5;
	driver.init();
	motor.linkDriver(&driver);
	motor.voltage_sensor_align = 5;
	motor.phase_resistance = 7.8; // [Ohm]
	motor.current_limit = 1.2;   // [Amps] - if phase resistance defined
	motor.voltage_limit = 5;   // [V] - if phase resistance not defined
	motor.velocity_limit = 20; // [rad/s] cca 50rpm
	motor.torque_controller = TorqueControlType::voltage;//TorqueControlType::voltage;
	motor.controller = MotionControlType::torque;
	
	motor.init();
	motor.initFOC();
	_delay(1000);


	tft.init();
	tft.initDMA();
	tft.fillScreen(TFT_BLACK);

	// Create the 2 sprites, each is half the size of the screen
	sprPtr[0] = (uint16_t*)spr[0].createSprite(DWIDTH, DHEIGHT);
	spr[0].fillSprite(TFT_TRANSPARENT);
	
	//for the 2 ticks
	spr[1].createSprite(6, DHEIGHT / 2);
	spr[1].setPivot(3.5, DHEIGHT / 2);
	spr[1].fillSprite(TFT_TRANSPARENT);
	//for the dot(white)
	spr[2].createSprite(24, DHEIGHT / 2);
	spr[2].setPivot(12, DHEIGHT / 2);
	spr[2].fillSprite(TFT_TRANSPARENT);
	spr[2].fillCircle(12, 14, 10, GC9A01A_WHITE);
	//for text in the middle
	spr[3].createSprite(DWIDTH, DHEIGHT);
	spr[3].setPivot(DWIDTH / 2, DHEIGHT / 2);
	spr[3].fillSprite(TFT_TRANSPARENT);
	spr[3].setTextDatum(MC_DATUM);
	spr[3].setTextColor(GC9A01A_WHITE);
	spr[3].setTextSize(5);
	spr[3].setTextFont(0);

	tft.startWrite(); // TFT chip select held low permanently

	xTaskCreatePinnedToCore(motorTask, "MotorTask", 40000, NULL, 1, &motorTaskHandle, 1); // Task assigned to core 1
	xTaskCreatePinnedToCore(lcdTask, "LCDTask", 40000, NULL, 1, &lcdTaskHandle, 0); // Task assigned to core 1
	disableCore0WDT();
}

void loop() {
}

int endLimit = 0;
bool isLimiting = false;
uint32_t donk = 0;
volatile float sharedFloat;
void motorTask(void *pvParameters) {
	while (1) {
		motor.loopFOC();
		motorShaftAngle = motor.shaft_angle;
		motor.move(P_haptic(attract_angle - motorShaftAngle));
		attract_angle = findAttractor(motorShaftAngle);
		sharedFloat = motorShaftAngle;
		xTaskNotifyGive(lcdTaskHandle);
	}
}

int limit = 0;
bool shouldDraw = true;
int direction = 0;
void lcdTask(void *pvParameters) {
    float localCopy;
    while (1) {
		uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, 0);
		if (notificationValue > 0) {
			localCopy = sharedFloat;
			if (localCopy > 2 * M_PI) {
				localCopy = 2 * M_PI;
			}
			if (localCopy < 0) {
				localCopy = 0;
			}
		}

        float shaftDegree = round(localCopy * (360.0f / (2 * M_PI)));
		if (shaftDegree != displayAngleHistory.getLast() && (!displayIsFluttering || !isWithinThreshold(displayAngleHistory.getLast(), shaftDegree, 2))) {
			displayIsFluttering = displayAngleHistory.add(shaftDegree);
			spr[0].fillSprite(TFT_BLACK);
			spr[0].fillRect(0,0,DWIDTH, map(shaftDegree, 45, 315, 0, DHEIGHT), GC9A01A_PURPLE);
			spr[3].fillSprite(TFT_TRANSPARENT);
			int value = map(shaftDegree, 45, 315, 0, 100);
			value = (value > 100) ? 100 : (value < 0) ? 0 : value;
			spr[3].drawCentreString(String(value), DWIDTH / 2, (DHEIGHT / 2) - 40, 2);
			spr[3].pushRotated(&spr[0], 180, TFT_TRANSPARENT);
			if (shaftDegree < 44.5) {
				limit = -1;
				spr[0].drawSmoothArc((DHEIGHT/2), (DHEIGHT/2), (DHEIGHT/2) - 10, (DHEIGHT/2) - 14, shaftDegree + 180, 45 + 180, GC9A01A_WHITE, TFT_BLACK, false);
			}
			else if (shaftDegree > 315.5) {
				limit = 1;
				spr[0].drawSmoothArc((DHEIGHT/2), (DHEIGHT/2), (DHEIGHT/2) - 10, (DHEIGHT/2) - 14, 135, shaftDegree - 180, GC9A01A_WHITE, TFT_BLACK, false);
			}
			//draw and push the end stop ticks
			spr[1].drawSmoothRoundRect(2, 2, 1, 0, 2, 20, GC9A01A_WHITE, TFT_BLACK);
			spr[1].pushRotated(&spr[0], 45, TFT_TRANSPARENT);
			spr[1].pushRotated(&spr[0], 315, TFT_TRANSPARENT);
			//draw and push the dot
			spr[2].pushRotated(&spr[0], displayAngleHistory.getLast(), TFT_TRANSPARENT);
			tft.pushImageDMA(0, 0, 240, 240, sprPtr[0]);
		}
    }
}
