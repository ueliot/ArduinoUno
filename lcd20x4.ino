/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-lcd-20x4
 */

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 20 column and 4 rows

void setup() {
  lcd.init(); // initialize the lcd
  lcd.backlight();

  lcd.setCursor(0, 0);            // move cursor the first row
  lcd.print("LCD 20x4");          // print message at the first row
  lcd.setCursor(0, 1);            // move cursor to the second row
  lcd.print("I2C Address: 0x27"); // print message at the second row
  lcd.setCursor(0, 2);            // move cursor to the third row
  lcd.print("DIYables");          // print message at the third row
  lcd.setCursor(0, 3);            // move cursor to the fourth row
  lcd.print("www.diyables.io");   // print message the fourth row
}

void loop() {
}
