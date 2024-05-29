# HVAC1.ino file code -
hvac1.ino is a code for HVAC trainer kit developed for TVET with no communication protocol 

# Temperature-Controlled Fan and Compressor System

This project controls a fan and a compressor based on the temperature sensed by a temperature sensor. The system uses an LCD to display the current temperature and activates the fan and compressor accordingly.

## Components

- ESP32 DEVKIT V1 board
- Temperature sensor (e.g., DS18B20)
- LCD 16x2  (I2C)
- Fan 220V 
- Compressor

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


# HVAC2.ino file code -
hvac2.ino added wifi and adafruit MQTT procol


### Overall Code Explanation

This project is designed to control an HVAC trainer kit, consisting of a fan and a compressor, based on temperature readings from a DS18B20 temperature sensor. The system displays the current temperature on an LCD and uses a predefined temperature threshold to control the fan and compressor. If the temperature falls below the minimum threshold, the compressor is turned off and the fan is turned on. If the temperature exceeds this threshold, both the fan and the compressor are turned on. Additionally, the project integrates with Adafruit IO, an IoT cloud platform, to enable remote monitoring and control. This integration is achieved through WiFi and MQTT protocols, allowing the system to publish temperature readings and subscribe to control feeds for the fan and compressor.

The code initializes the necessary hardware components, including the temperature sensor, LCD, fan, and compressor. It then connects to a WiFi network and attempts to connect to the Adafruit MQTT broker, logging the connection status to both the serial monitor and the LCD. The main loop of the program continuously reads the temperature, updates the LCD display, and controls the fan and compressor based on the temperature reading. It also publishes the temperature data to the Adafruit IO platform every 2 seconds and processes incoming MQTT messages to allow remote control of the fan and compressor via Adafruit IO feeds. This setup ensures real-time monitoring and control of the HVAC system both locally and remotely through the IoT platform.



#code 

#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define ONE_WIRE_BUS 19 // Temp sensor
#define COMPRESSOR_PIN 16
#define FAN_PIN 17
#define MIN_TEMPERATURE 10 // Minimum temperature set
#define MAX_TEMPERATURE 12 // Maximum temperature set

// WiFi credentials
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASS "your_wifi_password"

// Adafruit IO credentials
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "your_aio_username"
#define AIO_KEY "your_aio_key"

// Create an ESP8266 WiFiClient class to connect to the MQTT server
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup the MQTT feeds
Adafruit_MQTT_Subscribe compressor_feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/compressor");
Adafruit_MQTT_Subscribe fan_feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/fan");
Adafruit_MQTT_Publish temperature_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");

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

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print("WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
  lcd.setCursor(0, 0);
  lcd.print("WiFi connected!");

  // Setup MQTT subscriptions
  mqtt.subscribe(&compressor_feed);
  mqtt.subscribe(&fan_feed);

  // Attempt to connect to the MQTT broker
  connectToMQTT();
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

  // Send temperature data to Adafruit IO
  if (!temperature_feed.publish(tempC)) {
    Serial.println("Failed to publish temperature data");
  }

  // Ensure the connection to the MQTT server is alive
  if (!mqtt.connected()) {
    connectToMQTT();
  }
  mqtt.processPackets(10000);

  delay(2000); // Delay 2 seconds
}

// Connect to Adafruit MQTT broker
void connectToMQTT() {
  Serial.print("Connecting to MQTT... ");
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print("MQTT...");
  while (mqtt.connect() != 0) {
    Serial.println(mqtt.connectErrorString(mqtt.connect()));
    lcd.setCursor(0, 0);
    lcd.print("MQTT connect");
    lcd.setCursor(0, 1);
    lcd.print("failed...");
    mqtt.disconnect();
    delay(5000); // wait 5 seconds and try again
  }
  Serial.println("MQTT connected!");
  lcd.setCursor(0, 0);
  lcd.print("MQTT connected!");
}

// Handle messages from Adafruit IO
void handleMessage(Adafruit_MQTT_Subscribe *subscription) {
  if (subscription == &compressor_feed) {
    int value = atoi((char *)compressor_feed.lastread);
    digitalWrite(COMPRESSOR_PIN, value);
    Serial.print("Compressor: ");
    Serial.println(value);
  } else if (subscription == &fan_feed) {
    int value = atoi((char *)fan_feed.lastread);
    digitalWrite(FAN_PIN, value);
    Serial.print("Fan: ");
    Serial.println(value);
  }
}

