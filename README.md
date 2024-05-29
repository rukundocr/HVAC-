# HVAC-
hvac1.ino is a code for HVAC trainer kit developed for TVET 

# Temperature-Controlled Fan and Compressor System

This project controls a fan and a compressor based on the temperature sensed by a temperature sensor. The system uses an LCD to display the current temperature and activates the fan and compressor accordingly.

## Components

- Arduino board
- Temperature sensor (e.g., DS18B20)
- LCD (I2C)
- Fan
- Compressor
- Resistors, wires, and a breadboard

## Wiring

- Temperature Sensor:
  - Data pin to digital pin 19
  - Power and ground to 5V and GND respectively

- LCD:
  - SDA to A4
  - SCL to A5
  - Power and ground to 5V and GND respectively

- Fan:
  - Control pin to digital pin 17
  - Power and ground as per fan specifications

- Compressor:
  - Control pin to digital pin 16
  - Power and ground as per compressor specifications

## Code

This code reads the temperature from the sensor and controls the fan and compressor based on predefined temperature thresholds. The current temperature is displayed on an LCD.

```cpp
#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 19 // Temp sensor
#define COMPRESSOR_PIN 16
#define FAN_PIN 17
#define MIN_TEMPERATURE 10 // Minimum temperature set
#define MAX_TEMPERATURE 12 // Maximum temperature set

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(COMPRESSOR_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  
  sensors.begin();
  lcd.init();
  lcd.backlight();
  
  Serial.begin(115200);
  while (!Serial);
}

void loop() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  Serial.print(tempC);
  lcd.setCursor(0, 0);
  lcd.print("Temperature");
  lcd.setCursor(0, 1);
  lcd.print(tempC);

  if (tempC <= MIN_TEMPERATURE) {
    digitalWrite(COMPRESSOR_PIN, LOW);
    digitalWrite(FAN_PIN, HIGH);
  } else {
    digitalWrite(COMPRESSOR_PIN, HIGH);
    digitalWrite(FAN_PIN, HIGH);
  }

  Serial.print(tempC);
  delay(3000);
}
