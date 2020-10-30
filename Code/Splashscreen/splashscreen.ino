/**************************************************************************
 In MS paint save image as a 24bit bmp
 Install LCD Image Converter
 -prepare: Invert
 -image: Color R8, 24 bit, 
 **************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "logo.h"
//#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>


  // For the breakout board, you can use any 2 or 3 pins.
  // These pins will also work for the 1.8" TFT shield.
#define TFT_CS        4
#define TFT_RST        22 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         21
#define TFT_BACKLIGHT  26

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void setup(void) 
{
  Serial.begin(115200);
  delay(100);
 
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);

  tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
  delay(100);

  tft.invertDisplay(1); 
  tft.setRotation(3);

  Serial.println(F("Initialized"));

  tft.setCursor(0, 0); 
  tft.drawRGBBitmap(1, 2, logo, 160, 80);

 

  Serial.println("done");
  delay(1000);
}

void loop() 
{

}
