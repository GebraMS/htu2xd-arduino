#include "GebraBit_HTU2XD.h"

GebraBit_HTU2XD HTU2XD;

void setup() {
    Wire.begin();           // Initialize the I2C bus
    Serial.begin(9600);     // Initialize serial communication for debugging

    GB_HTU2XD_initialize(&HTU2XD); // Initialize the HTU2XD sensor
    GB_HTU2XD_Configuration(&HTU2XD); // Configure the HTU2XD sensor
}

void loop() {
    GB_HTU2XD_Get_Data(&HTU2XD); // Read data from the sensor

    Serial.print("Temperature: ");
    Serial.print(HTU2XD.TEMPERATURE);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(HTU2XD.HUMIDITY);
    Serial.println(" %");

    delay(2000); // Delay between readings
}
