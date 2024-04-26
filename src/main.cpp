#include <Arduino.h>
#include "./my_functions.h"
#include "Adafruit_GC9A01A.h"
#include "Wire.h"
#include <Adafruit_INA219.h>
#include <SimpleFOC.h>
#include "haptic.h"
#include <cmath>

#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

class CircularBuffer {
private:
		std::array<float, 3> buffer; // Fixed-size buffer to store the last two floats
		int writeIndex; // Index to write new values
public:
		CircularBuffer() : writeIndex(0) {} // Constructor initializes writeIndex to 0
		
		bool add(float value) {
				// Shift the existing values to make room for the new one - only if its not the same as the last value
				if (value != buffer[0]) {
					buffer[2] = buffer[1];
					buffer[1] = buffer[0];
					buffer[0] = value;
				}
				return buffer[0] == buffer[2] && buffer[1] != buffer[0];
		}
		
		float getLast() {
				return buffer[0];
		}
		float getSecondLast() {
				return buffer[1];
		}
};
/*-----------------------------------------------
	LCD Display
-----------------------------------------------*/
#define MISO   -1
#define SCLK   4
#define MOSI   5
#define RES    6
#define DC     7
#define CS     15
/*
SPIClass mySPI(FSPI);
Adafruit_GC9A01A tft(&mySPI, DC, CS, RES);
*/
float h = 0;
float outerRadius = 0;
float innerRadius = 0;
uint16_t screenBuffer[240 * 240];

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
BLDCMotor motor = BLDCMotor(7);//, 7.8, 270);
//other motor
//BLDCMotor motor = BLDCMotor(6);



/*-----------------------------------------------
	Current Sensor
-----------------------------------------------*/
//the current sensor
/*
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
// current sensor - this requires the modified simpleFoc library
InlineCurrentSense current_sense = InlineCurrentSense(&current_sense_red_C, &current_sense_blue_B, &current_sense_yellow_A);
*/



/*-----------------------------------------------
	Position Sensor
-----------------------------------------------*/
MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
TwoWire positionSensorI2C(1);
#define pos_SDA_PIN 17
#define pos_SCL_PIN 16

/*-----------------------------------------------
	Haptic controller and things
-----------------------------------------------*/

PIDController P_haptic(1.2,0,0,100000,5);
float shaftVelocity = 60;
// attractor angle variable
float attract_angle = 0;
// distance between attraction points
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

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }


void setup() {
	Serial.begin(115200);

	// Add some sample values
	buffer.add(0.0f);
	buffer.add(0.0f);
	buffer.add(0.0f);
	/*
	currentSenseI2C.begin(cur_SDA_PIN, cur_SCL_PIN); //SDA - SCL
	 // Initialize the INA219 sensor
	if (!current_sense_yellow_A.begin(&currentSenseI2C)) {
		Serial.println("Failed to initialize current_sense_1!");
		while (1); // Halt the program if initialization fails
	}
	if (!current_sense_blue_B.begin(&currentSenseI2C)) {
		Serial.println("Failed to initialize current_sense_2!");
		while (1); // Halt the program if initialization fails
	}
	if (!current_sense_red_C.begin(&currentSenseI2C)) {
		Serial.println("Failed to initialize current_sense_3!");
		while (1); // Halt the program if initialization fails
	}
	current_sense.init();
	*/
	
	positionSensorI2C.begin(pos_SDA_PIN, pos_SCL_PIN);
	as5600.init(&positionSensorI2C);
	motor.linkSensor(&as5600);
	
	driver.voltage_power_supply = 5;
	driver.init();
	motor.linkDriver(&driver);
	motor.voltage_sensor_align = 5;
	// limiting motor movements
	//sparkfun motor
	motor.phase_resistance = 7.8; // [Ohm]
	//other motor
	//motor.phase_resistance = 12.5;
	motor.current_limit = 1.2;   // [Amps] - if phase resistance defined
	motor.voltage_limit = 5;   // [V] - if phase resistance not defined
	motor.velocity_limit = 20; // [rad/s] cca 50rpm
	motor.torque_controller = TorqueControlType::voltage;//TorqueControlType::voltage;
	motor.controller = MotionControlType::torque;
	
	motor.init();
	motor.initFOC();
	motor.target = 0;
	_delay(1000);

	/*
	tft_donk.init();
	tft_donk.fillScreen(TFT_BLACK);
	*/
	tft.init();
  tft.setRotation(0);
  Serial.begin(115200); // For debug

	h = tft.getViewportHeight();
	outerRadius = h/2;
	innerRadius = (h/2) * 0.25;

	for (float i = 0; i < 361; i++) {
		
		addToCache(
			i,
			((innerRadius * cos(i * M_PI / 180.0f)) + outerRadius),
			((innerRadius * sin(i * M_PI / 180.0f)) + outerRadius),
			((outerRadius - 20) * cos(i * M_PI / 180.0f)) + outerRadius,
			((outerRadius - 20) * sin(i * M_PI / 180.0f)) + outerRadius
		);
	}

	//xTaskCreatePinnedToCore(motorTask, "MotorTask", 10000, NULL, 1, NULL, 0); // Task assigned to core 0
	//xTaskCreatePinnedToCore(lcdTask, "LCDTask", 10000, NULL, 1, NULL, 1); // Task assigned to core 1
}
//This one is for the lcd
float shaftAngle = 0;
//This one is for the motor
float currentShaftAngle = 0;

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
PhaseCurrent_s donk;

float normalizeRadians(float radians) {
		float twoPi = 2 * M_PI;
		radians = fmod(radians, twoPi);
		if (radians < 0) {
				radians += twoPi;
		}
		return radians;
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
			//erase the old
			coordinates = getFromCache(buffer.getSecondLast());
			tft.fillCircle(coordinates[2] - 5, coordinates[3] - 5, 10, GC9A01A_BLACK, false);
			//draw the new
			coordinates = getFromCache(buffer.getLast());
			tft.fillCircle(coordinates[2] - 5, coordinates[3] - 5, 10, GC9A01A_WHITE, true);
		}
		//if its fluttering, then we only add new value if tis +/- 2
		else if (!isWithinThreshold(shaftDegree, buffer.getLast(), 2)) {
			isFluttering = buffer.add(shaftDegree);
			//erase the old
			coordinates = getFromCache(buffer.getSecondLast());
			tft.fillCircle(coordinates[2] - 5, coordinates[3] - 5, 10, GC9A01A_BLACK, false);
			//draw the new
			coordinates = getFromCache(buffer.getLast());
			tft.fillCircle(coordinates[2] - 5, coordinates[3] - 5, 10, GC9A01A_WHITE, true);
		}
	}
}


void motorTask(void *pvParameters) {
	while (1) {
		motor.loopFOC();
		currentShaftAngle = motor.shaft_angle;
		motor.move(P_haptic(attract_angle - currentShaftAngle));
		attract_angle = findAttractor(currentShaftAngle);
	}
}
void lcdTask(void *pvParameters) {
	while (1) {
		shaftAngle = normalizeRadians(motor.shaft_angle);
		float shaftDegree = round(shaftAngle * (360.0f / (2 * M_PI)));
		//reading a new degree
		if (shaftDegree != buffer.getLast()) {
			//if its not fluttering, we can add the new value
			if (!isFluttering) {
				//+/- 1 and not fluttering
				isFluttering = buffer.add(shaftDegree);

				//erase the old
				coordinates = getFromCache(buffer.getSecondLast());
				tft.fillCircle(coordinates[2] - 5, coordinates[3] - 5, 10, GC9A01A_BLACK, false);
				//draw the new
				coordinates = getFromCache(buffer.getLast());
				tft.fillCircle(coordinates[2] - 5, coordinates[3] - 5, 10, GC9A01A_WHITE, true);
			}
			//if its fluttering, then we only add new value if tis +/- 2
			else if (!isWithinThreshold(shaftDegree, buffer.getLast(), 2)) {
				isFluttering = buffer.add(shaftDegree);

				//erase the old
				coordinates = getFromCache(buffer.getSecondLast());
				tft.fillCircle(coordinates[2] - 5, coordinates[3] - 5, 10, GC9A01A_BLACK, false);
				//draw the new
				coordinates = getFromCache(buffer.getLast());
				tft.fillCircle(coordinates[2] - 5, coordinates[3] - 5, 10, GC9A01A_WHITE, true);
			}
		}
	}
}

