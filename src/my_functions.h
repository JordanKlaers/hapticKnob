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

bool isWithinThreshold(float value1, float value2, float threshold);

#endif