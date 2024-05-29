#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <BluetoothSerial.h>

#define ONE_WIRE_BUS 19 // Temp sensor
#define COMPRESSOR_PIN 16
#define FAN_PIN 17
#define MIN_TEMPERATURE 10 // Minimum temperature set
#define MAX_TEMPERATURE 12 // Maximum temperature set

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);
BluetoothSerial SerialBT;

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

unsigned long previousMillis = 0;
const long interval = 1000; // Interval to send temperature data (1 second)

void setup() {
  pinMode(COMPRESSOR_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  sensors.begin();
  lcd.init();
  lcd.backlight();

  // Create custom degree symbol
  lcd.createChar(0, degreeSymbol);

  Serial.begin(115200);
  SerialBT.begin("ESP32_HVAC"); // Bluetooth device name
  while (!Serial);

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
}

void loop() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  lcd.setCursor(0, 0);
  lcd.print("COIL Temp:");
  lcd.setCursor(10, 0);
  lcd.print(tempC, 1);
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

  // Handle Bluetooth commands
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();
    if (command == "COMPRESSOR_ON") {
      digitalWrite(COMPRESSOR_PIN, HIGH);
      lcd.setCursor(0, 1);
      lcd.print("Compressor: ON ");
    } else if (command == "COMPRESSOR_OFF") {
      digitalWrite(COMPRESSOR_PIN, LOW);
      lcd.setCursor(0, 1);
      lcd.print("Compressor: OFF");
    } else if (command == "FAN_ON") {
      digitalWrite(FAN_PIN, HIGH);
    } else if (command == "FAN_OFF") {
      digitalWrite(FAN_PIN, LOW);
    } else if (command == "ESP3") {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("HVAC BLUETOOTH");
      lcd.setCursor(0, 1);
      lcd.print("CONTROL");
      delay(4000); // Wait for 4 seconds
      lcd.clear();
    }
  }

  // Send temperature data every second
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    SerialBT.print("Temperature: ");
    SerialBT.print(tempC);
    SerialBT.println("C");
  }

  delay(100); // Short delay to avoid overwhelming the loop
}
