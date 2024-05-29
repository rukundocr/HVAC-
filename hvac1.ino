#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 19 // Temp sensor
#define COMPRESSOR_PIN 16
#define FAN_PIN 17
#define MIN_TEMPERATURE 10 // Minimum temperature set
#define MAX_TEMPERATURE 12 // Maximum temperature set
//Creating degree symbol 
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

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(COMPRESSOR_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  
  sensors.begin();
  lcd.init();
  lcd.backlight();
    // Create custom degree symbol
  lcd.createChar(0, degreeSymbol);
  Serial.begin(115200);
  while (!Serial);
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
  delay(3000);
}
