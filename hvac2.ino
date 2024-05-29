//HVAC2.ino file code 
//1.Add 3 feeds to control the fan, compressor, and temperature.
//2.Add options to connect to WiFi and log the connection status to the LCD and serial monitor.
//3.Add code to connect to the Adafruit MQTT broker and log the connection status to the LCD.
//4.Add code to send temperature to the temperature feed every 2 seconds.
//5.Add code to subscribe to compressor and fan feeds and control them via the Adafruit platform dashboard.


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
