#include "Adafruit_GC9A01A.h"
#include <Arduino.h>
#include <Adafruit_INA219.h>

#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

// Declare the function to be moved
void drawLineOnCircle(Adafruit_GC9A01A tft, float outerRadius, float innerRadius);
void drawTicks(Adafruit_GC9A01A tft);

void readCurrent(Adafruit_INA219 *currentSensor, int sensorNumber, float current_mA);

#endif