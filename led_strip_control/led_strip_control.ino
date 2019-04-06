/**********************************************************************
* light_strip_control
* Date: 4/5/2019
* Author: Sam R.
* 
* Description: Program to demonstrate usage of the LedStrip library.
*   r - set to red
*   b - set to blue
*   g - set to green
**********************************************************************/

#include <LedStrip.h>

char readVal = 'u';
int brightness = 128;

//Define LED strip. Use pins 5, 6, and 7.
LedStrip ledStrip(5, 6, 7);


//=====setup==============================
void setup() {
  Serial.begin(9600);
  ledStrip.setBrightness(brightness);
  
  //initialize to a preset color
  ledStrip.setColorHEX(LedStrip::HotPink);
}

//=====loop==============================
void loop() {
  switch (readVal) {
  case 'r':
    ledStrip.setColorRGB(255, 0, 0);
    Serial.println("RED");
    break;
  case 'g':
    ledStrip.setColorRGB(0, 255, 0);
    Serial.println("GREEN");
    break;
  case 'b':
    ledStrip.setColorRGB(0, 0, 255);
    Serial.println("BLUE");
    break;
  case 'u':
    brightness += 25;
    ledStrip.setBrightness(brightness);
    Serial.println(brightness);
    break;
  case 'd':
    brightness -= 25;
    ledStrip.setBrightness(brightness);
    Serial.println(brightness);
    break;
  case 'h':
    ledStrip.setColorRGB(255, 255, 255);
    break;
  default:
    ledStrip.setColorRGB(255, 255, 255);
    Serial.println("WHITE");
    break;
}

ledStrip.printColor();
  
  //wait for a value, read it, and clear the extra
  while (Serial.available() <= 0) {
    delay(100);
  }
  readVal = Serial.read();
  while(Serial.available()){Serial.read();}
}
