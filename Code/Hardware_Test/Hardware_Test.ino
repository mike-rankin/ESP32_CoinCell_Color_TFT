//Tools/Board: ESP32 Dev Board
//Search Sketch/Include Library/Manage Libraries:
//Install Adafruit_GFX
//Install Adafruit_ST7735
//Install Adafruit_Sensor
//Install Adafruit_VL53L0X
//Install ClosedCube_HDC1080
//Install Adafruit_LIS3DH


/**************************************************************************
To get rid of the static on the bottom right of the lcd
in Adafruit_ST7735_and_ST7789_Library modify the Adafruit_ST7735.cpp file
under void Adafruit_ST7735::initR(uint8_t options) to
 displayInit(Rcmd2green160x80);
    _colstart = 24;  //originally 24 ,26 works better
    _rowstart = 2;   //originally 0 ,4 works well

To get the Accel to work possibly modify Adafruit_LIS3DH.h and set the i2c address to 0x19   
    
// Touch Pads on the bottom
// Touch Pad1 = GPIO27, T7
// Touch Pad2 = GPIO12, T5 
// Touch Pad3 = GPIO15, T3
// Touch Pad4 = GPIO2,  T2
 **************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L0X.h"
#include "ClosedCube_HDC1080.h"
#include <Adafruit_LIS3DH.h>
#include <SPI.h>
#include <Wire.h>

#define BLUE_LED 9
#define RED_LED 5
#define GREEN_LED 10

#define TFT_CS         4
#define TFT_RST        22 
#define TFT_DC         21
#define ACCEL_PWR      32 
#define TFT_BACKLIGHT  26

int threshold = 100;
bool touch1detected = false;
bool touch2detected = false;
bool touch3detected = false;
bool touch4detected = false;


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
ClosedCube_HDC1080 hdc1080;

void setup(void) 
{
  Serial.begin(115200);
  delay(1000);
  Serial.print(F("Hardware Test"));

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(TFT_BACKLIGHT, OUTPUT);

  digitalWrite(RED_LED, HIGH);   //Keep off
  digitalWrite(GREEN_LED, HIGH); //Keep off
  digitalWrite(BLUE_LED, HIGH);  //Keep off
  
  digitalWrite(TFT_BACKLIGHT, HIGH);

  blinkRGB();

  Wire.begin(13,14);
  delay(1000);

  //Temp/Humidity related
  hdc1080.begin(0x40);

  //Accel related
  pinMode(ACCEL_PWR, OUTPUT);
  digitalWrite(ACCEL_PWR, HIGH);
  lis.setRange(LIS3DH_RANGE_4_G);
  
  if (! lis.begin(0x19)) 
  {   // change this to 0x19 for alternative i2c address
   Serial.println("Couldnt start");
   while (1);
  } 

 
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) 
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 

  tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
  tft.invertDisplay(1);  //Fixes black & white reversed
  tft.setRotation(3);

  Serial.println(F("Initialized"));

  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(2, 5);  //over,down
  tft.setTextColor(ST77XX_RED);  //Actually Blue
  tft.setTextSize(2);
  tft.print("Hardware Test");
  delay(1000);

  touchAttachInterrupt(T7, gotTouch1, threshold);
  touchAttachInterrupt(T5, gotTouch2, threshold);
  touchAttachInterrupt(T3, gotTouch3, threshold);
  touchAttachInterrupt(T2, gotTouch4, threshold);
 

  // large block of text
  //tft.fillScreen(ST77XX_BLACK);
  //testdrawtext("Hardware Test", ST77XX_WHITE);
  //delay(1000);

 // tft.drawImageDemo();
 // delay(1000);

}

void loop() 
{
  digitalWrite(TFT_BACKLIGHT, HIGH);  //High keeps backlight on
  //digitalWrite(BLUE_LED, LOW);  //Green Nope, Blue Nope
  //delay(1000);
  
  //tft.drawRoundRect(10, 70, 30, 5, 30,ST77XX_GREEN);
  //tft.drawRoundRect(50, 70, 30, 5, 30,ST77XX_GREEN);
  //tft.drawRoundRect(90, 70, 30, 5, 30,ST77XX_GREEN);
  //tft.drawRoundRect(130, 70, 30, 5, 30,ST77XX_GREEN);

  //tft.setTextWrap(false);
  //tft.fillScreen(ST77XX_BLACK);
  //tft.setCursor(5, 5);  //over,down
  //tft.setTextColor(ST77XX_RED);  //Actually Blue
  //tft.setTextSize(2);
  //tft.print("Hardware Test");


  
  digitalWrite(ACCEL_PWR, HIGH);
  lis.read();
  sensors_event_t event;
  lis.getEvent(&event);

  VL53L0X_RangingMeasurementData_t measure;
  //Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  //if (measure.RangeStatus != 4) 
  //{  // phase failures have incorrect data
   // Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  //} else 
  //{
   // Serial.println(" out of range ");
  //}

  tft.setTextWrap(false);

  delay(5);
  tft.fillRect(0, 25, 180, 70, ST77XX_BLACK);  //Much faster refresh, change color to see refreshed area
 
  tft.setCursor(5, 30);  //over,down
  tft.setTextColor(ST77XX_MAGENTA);  
  tft.setTextSize(1);
  tft.print("TOF: ");
  tft.print(measure.RangeMilliMeter); 

  tft.setCursor(5, 40);  
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print("Accel: ");
  tft.print(event.acceleration.x);
  tft.print(" ");
  tft.print(event.acceleration.y); 
  tft.print(" ");
  tft.print(event.acceleration.z); 

  tft.setCursor(5, 50);  
  tft.setTextColor(ST77XX_MAGENTA);  
  tft.print(hdc1080.readTemperature());
  tft.print((char)247);
  tft.print("C  ");
  tft.print(hdc1080.readHumidity()); 
  tft.print("%  ");
  
  //delay(10);
  //tft.fillRect(0, 25, 180, 70, ST77XX_BLACK);  //Much faster refresh, change color to see refreshed area
  
   while (touch1detected)
  {
    Serial.println("Touch 1 detected");
    tft.fillRoundRect(10, 70, 30, 5, 30,ST77XX_GREEN);  //drawRoundRect(x0, y0, w, h, radius, color)
    delay(150);
    touch1detected = false;
    //tft.fillRoundRect(10, 70, 30, 5, 30,ST77XX_BLACK);
  }
  while (touch2detected)
  {
    Serial.println("Touch 2 detected");
    tft.fillRoundRect(50, 70, 30, 5, 30,ST77XX_GREEN);
    delay(150);
    touch2detected = false;
    //tft.fillRoundRect(50, 70, 30, 5, 30,ST77XX_BLACK);
  }
  while (touch3detected)
  {
    Serial.println("Touch 3 detected");
    tft.fillRoundRect(90, 70, 30, 5, 30,ST77XX_GREEN);  //drawRoundRect(x0, y0, w, h, radius, color)
    delay(150);
    touch3detected = false;
    //tft.fillRoundRect(90, 70, 30, 5, 30,ST77XX_BLACK);
  }
  while (touch4detected)
  {
    Serial.println("Touch 4 detected");
    tft.fillRoundRect(130, 70, 30, 5, 30,ST77XX_GREEN);  //drawRoundRect(x0, y0, w, h, radius, color)
    delay(150);
    touch4detected = false;
    //tft.fillRoundRect(130, 70, 30, 5, 30,ST77XX_BLACK);
  }

  

}


void gotTouch1()
{
  touch1detected = true;
}

void gotTouch2() 
{
  touch2detected = true;
}

void gotTouch3() 
{
  touch3detected = true;
}

void gotTouch4() 
{
  touch4detected = true;
}

void blinkRGB(void)
{
  digitalWrite(RED_LED, LOW);   //led on
  delay(150);
  digitalWrite(RED_LED, HIGH);   //led off
  delay(150);
  digitalWrite(GREEN_LED, LOW);
  delay(150);
  digitalWrite(GREEN_LED, HIGH);
  delay(150);
  digitalWrite(BLUE_LED, LOW);
  delay(150);
  digitalWrite(BLUE_LED, HIGH);
  delay(150);
}
