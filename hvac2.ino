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
#define WIFI_SSID "claudewifi"
#define WIFI_PASS "12341234"

// Adafruit IO credentials
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "" //add your adafruit username
#define AIO_KEY "" // add your adafruit API Key 

// Custom character for degree symbol
byte degreeSymbol[8] = {
  0b00111,
  0b00101,
  0b00111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};




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

  // Create custom degree symbol
  lcd.createChar(0, degreeSymbol);

  // Display company name
  lcd.setCursor(0, 0);
  lcd.print("Hills ELECTRONICS");
  lcd.setCursor(0, 1);
  lcd.print("S LTD");
  delay(3000); // Wait for 3 seconds
  
  // Clear the display and show the next message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HVAC Trainer kit");
  delay(3000); // Wait for another 3 seconds

  // Clear the display before starting the loop
  lcd.clear();
  
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
  delay(2000); //
  lcd.clear(); 
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
  lcd.print("COIL Temp:");
  lcd.setCursor(10, 0);
  lcd.print(tempC,1);
  lcd.write(byte(0));  // Display the custom degree symbol
  lcd.print("C");

  if (tempC <= MIN_TEMPERATURE) {
    digitalWrite(COMPRESSOR_PIN, LOW);
    digitalWrite(FAN_PIN, HIGH);
    lcd.setCursor(0, 1); // Set cursor to the beginning of the second line
    lcd.print("Compressor: OFF ");
  } else {
    digitalWrite(COMPRESSOR_PIN, HIGH);
    digitalWrite(FAN_PIN, HIGH);
    lcd.setCursor(0, 1); // Set cursor to the beginning of the second line
    lcd.print("Compressor: ON");
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
  delay(2000); //
  lcd.clear(); 
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
