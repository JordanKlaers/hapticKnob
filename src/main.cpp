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

std::array<float, 4> coordinates = {0.0f, 0.0f, 0.0f, 0.0f};
CircularBuffer buffer;
bool isFluttering = false;
float shaftAngle = 0;
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
}

void loop() {
	shaftAngle = normalizeRadians(as5600.getSensorAngle());
	float shaftDegree = round(shaftAngle * (360.0f / (2 * M_PI)));
	//reading a new degree
	if (shaftDegree != buffer.getLast()) {
		//if its not fluttering, we can add the new value
		if (!isFluttering) {
			//+/- 1 and not fluttering
			isFluttering = buffer.add(shaftDegree);
			dot.fillSmoothCircle(10, 10, 10, GC9A01A_BLACK, GC9A01A_BLACK);
			dot.pushRotated(&background, buffer.getSecondLast(), TFT_TRANSPARENT);
			dot.fillSmoothCircle(10, 10, 10, GC9A01A_WHITE, GC9A01A_BLACK);
			dot.pushRotated(&background, buffer.getLast(), TFT_TRANSPARENT);
		}
		//if its fluttering, then we only add new value if tis +/- 2
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
