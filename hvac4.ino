//using remote control for hvac kit


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DIYables_IRcontroller.h>
#include <OneWire.h>
#include <DallasTemperature.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define IR_RECEIVER_PIN 4
#define ONE_WIRE_BUS 19
DIYables_IRcontroller_21 irController(IR_RECEIVER_PIN, 200);
int maxtemp = 10;
int mintemp = 5;
int compressor = 16;
int fan = 17;
float tempC;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
void setup() {
  Serial.begin(115200);
  irController.begin();
  lcd.init();
  lcd.backlight();
  
  pinMode(compressor, OUTPUT);
  pinMode(fan, OUTPUT);
}

void loop() {
sensors.requestTemperatures();
tempC = sensors.getTempCByIndex(0);
lcd.clear();
updateLCD();
 Key21 command = irController.getKey();
  if (command != Key21::NONE) {
    switch (command) {
      case Key21::KEY_VOL_PLUS:
        Serial.println("VOL+");
        changeMax1();
        break;
       case Key21::KEY_VOL_MINUS:
        Serial.println("VOL-");
        changeMin1();
        break;
        case Key21::KEY_CH_MINUS:
        Serial.println("CH-");
        changeMin2();
        break;
       case Key21::KEY_CH_PLUS:
        Serial.println("CH+");
        changeMax2();
        break;
         default:
        Serial.println("WARNING: undefined command:");
        lcd.setCursor(15, 0);
        lcd.print("x");
        delay(60);
        
        break;
        
    }

     if (tempC >= maxtemp){
        digitalWrite(compressor, HIGH);
        digitalWrite(fan, HIGH);                                          
       }
       
       if(tempC <= mintemp ){
            digitalWrite(compressor, LOW);
            digitalWrite(fan, HIGH);
     }
  }
}

void updateLCD() {
  // Clear the previous number and display the new number
  lcd.setCursor(0, 0);
  lcd.print("range:"); // Clear the previous number
  lcd.setCursor(8, 0);
  lcd.print(maxtemp);
  lcd.print("<->");
  lcd.print(mintemp);
  lcd.setCursor(1, 1);
  lcd.print("TEMP: ");
  lcd.print(tempC);
  
}

void changeMax1() {
    maxtemp++;
    updateLCD();
}

void changeMin1() {
    maxtemp--;
    updateLCD();
}

void changeMax2() {
    mintemp++;
    updateLCD();
}

void changeMin2() {
    mintemp--;
    updateLCD();
}
