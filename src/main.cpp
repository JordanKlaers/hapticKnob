#include <Arduino.h>
#include "./my_functions.h"
#include "Adafruit_GC9A01A.h"
#include "Wire.h"
#include <Adafruit_INA219.h>
#include <SimpleFOC.h>
#include "haptic.h"
#include <cmath>

#include <TFT_eSPI.h>
// Width and height of sprite
#define TFT_WIDTH  240
#define TFT_HEIGHT 240


TFT_eSPI tft = TFT_eSPI();
TFT_eSprite background = TFT_eSprite(&tft);
TFT_eSprite dot = TFT_eSprite(&tft);



/*-----------------------------------------------
	LCD Display
-----------------------------------------------*/
#define MISO   -1
#define SCLK   4
#define MOSI   5
#define RES    6
#define DC     7
#define CS     15

float h = 0;
float outerRadius = 0;
float innerRadius = 0;
uint16_t screenBuffer[240 * 240];

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

PIDController P_haptic(0.4,0,0,100000,5);
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



std::array<float, 4> coordinates = {0.0f, 0.0f, 0.0f, 0.0f};
CircularBuffer buffer;
bool isFluttering = false;
float shaftAngle = 0;
float motorShaftAngle = 0;
bool isWithinThreshold(float value1, float value2, float threshold) {
		// Calculate the absolute difference between the two values
		float absDiff = std::abs(value1 - value2);
		// Handle the special case where the values are close to the range boundary
		if (absDiff > 180.0) {
				absDiff = 360.0 - absDiff;
		}
		// Compare the absolute difference against the threshold
		return absDiff < threshold;
}
float normalizeRadians(float radians) {
		float twoPi = 2 * M_PI;
		radians = fmod(radians, twoPi);
		if (radians < 0) {
				radians += twoPi;
		}
		return radians;
}

	void setup() {
		Serial.begin(115200);

		// Add some sample values
		buffer.add(0.0f);
		buffer.add(0.0f);
		buffer.add(0.0f);
		
		
		positionSensorI2C.begin(pos_SDA_PIN, pos_SCL_PIN);
		as5600.init(&positionSensorI2C);

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



		/*------------------------------------------------------------
			Setting up TFT
				this uses a background sprint to draw/clear the dot
				the background is pushed into the display with as much transparency as possible

				X and Y coordinates for the dot position are associated to the degree of the sensor
				values are saved in an object that can manage fluttering vaules as well
		------------------------------------------------------------*/
		tft.begin();
		tft.setRotation(1);
		tft.fillScreen(GC9A01A_BLACK);
		
		//start with black background -> push it to display -> then reset to transparent so subsequent pushes are faster
		background.createSprite(240, 240);
		background.setPivot(TFT_WIDTH / 2, TFT_HEIGHT / 2);
		background.fillSprite(GC9A01A_BLACK);
		background.pushSprite(0,0);
		background.fillSprite(TFT_TRANSPARENT);
		//dot only needs to be big enough for the circle - the pivot point is the center of the screen
		dot.createSprite(21, 21);
		dot.setPivot(10, (TFT_HEIGHT / 2) - 10);
		dot.fillSprite(TFT_TRANSPARENT);


		/*
			The degrees are saved into cache if the cricle position is drawn using the x and y coordinates

			if this is commented out, then the alternative approach is used, drawing the dots using the pushRotated method of the TFT_eSPI library
		*/

		/*
		h = tft.getViewportHeight();
		outerRadius = h/2;
		innerRadius = (h/2) * 0.25;

		for (float i = 0; i < 361; i++) {
			//save points x and y for the circle position into memory for faster access
			addToCache(
				i,
				((innerRadius * cos(i * M_PI / 180.0f)) + outerRadius),
				((innerRadius * sin(i * M_PI / 180.0f)) + outerRadius),
				((outerRadius - 20) * cos(i * M_PI / 180.0f)) + outerRadius,
				((outerRadius - 20) * sin(i * M_PI / 180.0f)) + outerRadius
			);
		}
		*/
		xTaskCreatePinnedToCore(motorTask, "MotorTask", 10000, NULL, 1, NULL, 0); // Task assigned to core 0
		xTaskCreatePinnedToCore(lcdTask, "LCDTask", 10000, NULL, 1, NULL, 1); // Task assigned to core 1
	}

	void loop() {
	}

	void motorTask(void *pvParameters) {
		while (1) {
			motor.loopFOC();
			motorShaftAngle = motor.shaft_angle;
			motor.move(P_haptic(attract_angle - motorShaftAngle));
			attract_angle = findAttractor(motorShaftAngle);
		}
	}

	void lcdTask(void *pvParameters) {
		/*
			This task reads the motor angle and draws the dot at the corresponding position
			it draws the dot into a sprite, rotated by the screen. That sprite is then pushed into a sprite representing the whole screen
			then all of that is pushed at once into view.

			As many pixels as possible are drawn transparent to speed up the drawing process
		*/
		while (1) {
			shaftAngle = normalizeRadians(as5600.getSensorAngle());
			float shaftDegree = round(shaftAngle * (360.0f / (2 * M_PI)));
			//compare the current degree with the last degree saved to buff
			if (shaftDegree != buffer.getLast()) {
				/*
					if its not fluttering, we can add the new value
					fluttering is when the last saved degree (index 0) and the third saved degree (index 2) are the same while the second is different
					for example [1,2,1] would be considered fluttering
				*/
				if (!isFluttering) {
					//+/- 1 and not fluttering
					isFluttering = buffer.add(shaftDegree);
					//erase the old circle by drawing it black to match the background
					dot.fillSmoothCircle(10, 10, 10, GC9A01A_BLACK, GC9A01A_BLACK);
					dot.pushRotated(&background, buffer.getSecondLast(), TFT_TRANSPARENT);
					//draw the circle in the new degree
					dot.fillSmoothCircle(10, 10, 10, GC9A01A_WHITE, GC9A01A_BLACK);
					dot.pushRotated(&background, buffer.getLast(), TFT_TRANSPARENT);
				}
				//if its fluttering, then we only add new value if its +/- 2
				else if (!isWithinThreshold(shaftDegree, buffer.getLast(), 2)) {
					isFluttering = buffer.add(shaftDegree);
					dot.fillSmoothCircle(10, 10, 10, GC9A01A_BLACK, GC9A01A_BLACK);
					dot.pushRotated(&background, buffer.getSecondLast(), TFT_TRANSPARENT);
					dot.fillSmoothCircle(10, 10, 10, GC9A01A_WHITE, GC9A01A_BLACK);
					dot.pushRotated(&background, buffer.getLast(), TFT_TRANSPARENT);
				}
				background.pushSprite(0,0,TFT_TRANSPARENT);
			}
		}
	}