#include "Adafruit_GC9A01A.h"
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <map>
#include <array>

#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

// Declare the function to be moved
void drawLineOnCircle(Adafruit_GC9A01A tft, float outerRadius, float innerRadius);
void drawTicks(Adafruit_GC9A01A tft);

void readCurrent(Adafruit_INA219 *currentSensor, int sensorNumber, float current_mA);


// Define the map data structure
extern std::map<float, std::array<float, 4>> cacheMap;

// Function prototypes
void addToCache(float key, float value1, float value2, float value3, float value4);
std::array<float, 4> getFromCache(float key);

#endif