/*

 Deep sleep sketch 

*/
#define BLUE_LED 9
#define RED_LED 5
#define GREEN_LED 10

#define TFT_BACKLIGHT  26

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
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

void setup()
{
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(TFT_BACKLIGHT, OUTPUT);

  digitalWrite(RED_LED, HIGH);   //Keep off
  digitalWrite(GREEN_LED, HIGH); //Keep off
  digitalWrite(BLUE_LED, HIGH);  //Keep off
  
  digitalWrite(TFT_BACKLIGHT, HIGH);

  blinkRGB();
  

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */

  //digitalWrite(26, LOW);         //Low turns off the backlight
  //digitalWrite(32, LOW);  //Low turns off the accel
  
  Serial.println("Going to sleep now");
  Serial.flush(); 
 
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void loop()
{
  
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
