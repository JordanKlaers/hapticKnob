#include "my_functions.h"
#include "Adafruit_GC9A01A.h"
#include <Arduino.h>

void drawLineOnCircle(Adafruit_GC9A01A tft, float outerRadius, float innerRadius) {
    // Loop through 24 points (360 degrees / 15 degrees)
    for (int angle = 0; angle < 360; angle += 15) {
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