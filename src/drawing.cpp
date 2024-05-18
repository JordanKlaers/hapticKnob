#include "my_functions.h"
#include "Adafruit_GC9A01A.h"
#include <Arduino.h>
#include <map>
#include <array>

void drawLineOnCircle(Adafruit_GC9A01A tft, float outerRadius, float innerRadius) {
    // Loop through 24 points (360 degrees / 15 degrees)
    for (int angle = 0; angle < 360; angle += 17.1428571429) {
        // Convert angle from degrees to radians
        float radians = angle * M_PI / 180.0f;
        
        // Calculate the coordinates of the point on the circle's circumference
        float x1 = (innerRadius * cos(radians)) + outerRadius;
        float x2 = (outerRadius * cos(radians)) + outerRadius;
        float y1 = (innerRadius * sin(radians)) + outerRadius;
        float y2 = (outerRadius * sin(radians)) + outerRadius;

        tft.drawLine(x1, y1, x2, y2, GC9A01A_GREEN);
    }
}

void drawTicks(Adafruit_GC9A01A tft) {
  float w = tft.width();
  float h = tft.height();
  drawLineOnCircle(tft, (h/2), ((h/2) * 0.25));
}

// Define the map data structure
std::map<float, std::array<float, 4>> cacheMap;

// Function to add a value to the cache
void addToCache(float key, float value1, float value2, float value3, float value4) {
    // Create an array to hold the values
    std::array<float, 4> values = {value1, value2, value3, value4};

    // Add the key-value pair to the map
    cacheMap[key] = values;
}

// Function to retrieve values from the cache
std::array<float, 4> getFromCache(float key) {
    // Check if the key exists in the map
    auto it = cacheMap.find(key);
    if (it != cacheMap.end()) {
        // Return the array of values corresponding to the key
        return it->second;
    } else {
        // Return an empty array or handle the case where the key is not found
        // For simplicity, returning an array with all elements set to 0
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }
}


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