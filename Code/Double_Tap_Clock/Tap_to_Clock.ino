// Basic demo for tap/doubletap and display clock

#include <Adafruit_ST7735.h>
#include <Wire.h>
#include <WiFi.h>  
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "time.h" 

#define TFT_CS         4
#define TFT_RST        22 
#define TFT_DC         21
#define TFT_BACKLIGHT  26


#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex

//RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

const char* ssid       = "SSID";
const char* password   = "PASSWORD";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -14400;
const int   daylightOffset_sec = 3600;

RTC_DATA_ATTR int bootCount = 0;

struct tm timeinfo;


// Adjust this number for the sensitivity of the 'click' force
// this strongly depend on the range! for 16G, try 5-10
// for 8G, try 10-20. for 4G try 20-40. for 2G try 40-80
#define CLICKTHRESHHOLD 80

void setup(void) 
{
  Serial.begin(115200);
  delay(1000);
  Serial.print(F("Hardware Test"));

  tft.initR(INITR_MINI160x80);  // Init ST7735S mini display
  tft.invertDisplay(1);  //Fixes black & white reversed
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  
  Wire.begin(13,14);
  delay(1000);
  
  Serial.println("Adafruit LIS3DH Tap Test!");

  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
      delay(500);
      Serial.print(".");
  }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);   //init and get the time


  
  if (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

  // 0 = turn off click detection & interrupt
  // 1 = single click only interrupt output
  // 2 = double click only interrupt output, detect single click
  // Adjust threshhold, higher numbers are less sensitive
  lis.setClick(2, CLICKTHRESHHOLD);
  delay(100);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //1 = High, 0 = Low

  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  digitalWrite(TFT_BACKLIGHT, HIGH);
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(30, 30);  //over,down
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(&timeinfo, "%H:%M:");
  tft.setTextSize(1);
  tft.print(&timeinfo, "%S");
  delay(3000);
  tft.fillScreen(ST77XX_BLACK);

  

  digitalWrite(TFT_BACKLIGHT, LOW);
  //Go to sleep now
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");

  
}


void loop() 
{
  /*
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  digitalWrite(TFT_BACKLIGHT, HIGH);
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(30, 30);  //over,down
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print(&timeinfo, "%H:%M:");
  tft.setTextSize(1);
  tft.print(&timeinfo, "%S");
  delay(3000);
  tft.fillScreen(ST77XX_BLACK);
  */
}
