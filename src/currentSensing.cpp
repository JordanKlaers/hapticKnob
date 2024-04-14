#include "my_functions.h"
#include <Arduino.h>
#include <Adafruit_INA219.h>

void readCurrent(Adafruit_INA219 *currentSensor, int sensorNumber, float current_mA) {	
  //float shuntvoltage = 0;
  //float busvoltage = 0;
  //float loadvoltage = 0;
  //float power_mW = 0;
  //shuntvoltage = currentSensor.getShuntVoltage_mV();
  //busvoltage = currentSensor.getBusVoltage_V();
  current_mA = currentSensor->getCurrent_mA();
  //power_mW = currentSensor.getPower_mW();
  //loadvoltage = busvoltage + (shuntvoltage / 1000);
  if (current_mA > 10) {
	Serial.print("Sensor ");
	Serial.print(sensorNumber);
	Serial.print(": ");
	//Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
	//Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
	//Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
	Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
	//Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  }
  current_mA = 0;
}