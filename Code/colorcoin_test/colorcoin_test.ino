#include "Adafruit_VL53L0X.h"
#include <bb_spi_lcd.h>
#include <WiFi.h>
#include <Wire.h>
#include <BitBang_I2C.h>
#include <BLEDevice.h>
#include <esp32_gamepad.h>

BLEScan *pBLEScan;
BLEScanResults foundDevices;
static uint8_t txBuffer[4096];
SS_GAMEPAD gp;
BBI2C i2c;
SPILCD lcd;
uint8_t devAddr[6];
volatile bool bChanged, bConnected;

//#define TRY_LOGIN

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

Adafruit_VL53L0X vlx = Adafruit_VL53L0X();
 const char* ssid       = "MYROUTER";  
 const char* password   = "MYPASSWORD";  

const char *szNames[]  = {"Unknown","SSD1306","SH1106","VL53L0X","BMP180", "BMP280","BME280",
                "MPU-60x0", "MPU-9250", "MCP9808","LSM6DS3", "ADXL345", "ADS1115","MAX44009",
                "MAG3110", "CCS811", "HTS221", "LPS25H", "LSM9DS1","LM8330", "DS3231", "LIS3DH",
                "LIS3DSH","INA219","SHT3X","HDC1080"};

static uint8_t imu_addr, temp_addr;

// Touch Pad1 = GPIO27, T7
// Touch Pad2 = GPIO12, T5 
// Touch Pad3 = GPIO15, T3
// Touch Pad4 = GPIO2 , T2
#define BUTTON_THRESHOLD 98
uint8_t iButtons[4] = {T7,T5,T3,T2};
// Display size
#define WIDTH 160
#define HEIGHT 80

// 40x40 pattern bitmap
// Classic Mac bomb icon
uint8_t ucBombMask[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x04,0x21,0x00,0x00,
  0x00,0x00,0x00,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x00,
  0x00,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x86,0x38,0xa8,0x00,
  0x00,0x00,0x00,0x01,0x01,0xc0,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x80,0x00,
  0x00,0x00,0x00,0x01,0x00,0x24,0x40,0x00,0x00,0x00,0x00,0x0f,0xe0,0x40,0x20,0x00,
  0x00,0x00,0x00,0x0f,0xe0,0x84,0x10,0x00,0x00,0x00,0x00,0x0f,0xe0,0x00,0x00,0x00,
  0x00,0x00,0x00,0x3f,0xf8,0x04,0x00,0x00,0x00,0x00,0x00,0xff,0xfe,0x00,0x00,0x00,
  0x00,0x00,0x00,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0x00,0x00,0x00,
  0x00,0x00,0x01,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x03,0xdf,0xff,0x80,0x00,0x00,
  0x00,0x00,0x03,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0x03,0xbf,0xff,0x80,0x00,0x00,
  0x00,0x00,0x03,0xbf,0xff,0x80,0x00,0x00,0x00,0x00,0x03,0xbf,0xff,0x80,0x00,0x00,
  0x00,0x00,0x03,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0x03,0xdf,0xff,0x80,0x00,0x00,
  0x00,0x00,0x01,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x01,0xf7,0xff,0x00,0x00,0x00,
  0x00,0x00,0x00,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xfe,0x00,0x00,0x00,
  0x00,0x00,0x00,0x3f,0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xe0,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00};

void SS_Callback(int iEvent, SS_GAMEPAD *pGamepad)
{
  if (iEvent == EVENT_DISCONNECT)
     bConnected = false;
  else if (iEvent == EVENT_CONNECT)
     bConnected = true;
  else
  {
    bChanged = 1;
    memcpy(&gp, pGamepad, sizeof(SS_GAMEPAD));
  }
} /* SS_Callback() */

void LIS3DHInit(byte bAddr)
{
uint8_t uc[4];

   imu_addr = bAddr;
   uc[0] = 0x20; // CTRL_REG1
   uc[1] = 0x77; // Turn on the sensor with ODR = 400Hz normal mode.
   I2CWrite(&i2c, imu_addr, uc, 2);
// High res & BDU enabled
   uc[0] = 0x23; // CTRL_REG4
   uc[1] = 0x88;
   I2CWrite(&i2c, imu_addr, uc, 2);
   // DRDY on INT1
   uc[0] = 0x22; // CTRL_REG3
   uc[1] = 0x10;
   I2CWrite(&i2c, imu_addr, uc, 2);

 // enable adcs
   uc[0] = 0x1f; // TEMP_CFG_REG
   uc[1] = 0x80;
   I2CWrite(&i2c, imu_addr, uc, 2);

   return;
   
   uc[0] = 0x21; // CTRL_REG2
   uc[1] = 0x01; // High-pass filter (HPF) enabled with 0.2Hz cut-off frequency for INT1 (AOI1) interrupt generation only.
   I2CWrite(&i2c, imu_addr, uc, 2);
   uc[0] = 0x22; // CTRL_REG3
   uc[1] = 0x40; // ACC AOI1 interrupt signal is routed to INT1 pin.
   I2CWrite(&i2c, imu_addr, uc, 2);
   uc[0] = 0x23; // CTRL_REG4
   uc[1] = 0x88; // Full Scale = +/-2 g with BDU and HR bits enabled.
   I2CWrite(&i2c, imu_addr, uc, 2);
   uc[0] = 0x24; // CTRL_REG5
//   uc[1] = 0x00; // INT1 pin is not latched, no need to read INT1_SRC to clear the int
   uc[1] = 0x08; // INT1 pin is latched; need to read the INT1_SRC register to clear the interrupt signal.   
   I2CWrite(&i2c, imu_addr, uc, 2); 
   // configurations for wakeup and motionless detection
   uc[0] = 0x32; // INT1_THS
   uc[1] = 0x02; // Threshold (THS) = 2LSBs * 15.625mg/LSB = 31.25mg.
   I2CWrite(&i2c, imu_addr, uc, 2);
   uc[0] = 0x33; // INT1_DURATION
   uc[1] = 0x01; // Duration = 1LSBs * (1/10Hz) = 0.1s
   I2CWrite(&i2c, imu_addr, uc, 2);
   uc[0] = 0x30; // INT1_CFG
   uc[1] = 0x15; // Enable XLIE, YLIE and ZLIE (low events) interrupt generation
//   uc[1] = 0xaa; // Enable ZHIE, YHIE, XHIE (high events) interrupt generation
   I2CWrite(&i2c, imu_addr, uc, 2);
} /* LIS3DHInit() */

void LIS3DHReadAccel(int16_t *X, int16_t *Y, int16_t *Z)
{
int i;
uint8_t ucTemp[8];

  i = I2CReadRegister(&i2c, imu_addr, 0xa8, ucTemp, 6);
  if (i > 0)
  {
    *X = (ucTemp[1] << 8) + ucTemp[0];
    *Y = (ucTemp[3] << 8) + ucTemp[2];
    *Z = (ucTemp[5] << 8) + ucTemp[4];
  }
  
} /* LIST3DHReadAccel() */

void TryConnect(void)
{
int counter = 0; 

  spilcdFill(&lcd, 0,DRAW_TO_LCD);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && WiFi.status() != WL_CONNECT_FAILED) //if not connected to wifi
  {
   spilcdWriteString(&lcd, 0,0,(char *)"Connecting to wifi", 0xffff,0, FONT_8x8, DRAW_TO_LCD);
   spilcdWriteString(&lcd, counter*8,8,(char *)".", 0xffff, 0, FONT_8x8, DRAW_TO_LCD);
   delay(1500);
   counter++;

   if (counter > 20)
   {
    spilcdWriteString(&lcd, 0,16,(char *)"Failed", 0xf800, 0, FONT_8x8, DRAW_TO_LCD);
    break;
   }

   if (WiFi.status() == WL_CONNECTED)  //if it connects to wifi
   {
    spilcdWriteString(&lcd, 0,16,(char *)"Connected!", 0x6e0,0,FONT_8x8, DRAW_TO_LCD);
    IPAddress myIP = WiFi.softAPIP();
//    display.println(myIP);
//    display.print("RSSI:");
//    display.print(WiFi.RSSI());
    delay(1000);  
   }
 } // while
} /* TryConnect() */

#define SDA_PIN 13
#define SCL_PIN 14

void I2CDetect() {
uint8_t map[16];
char szTemp[32];
uint8_t i;
int iDevice, iCount;

  spilcdFill(&lcd, 0, DRAW_TO_LCD);
  iCount = 0;
  spilcdWriteString(&lcd, 0,0,(char *)"Starting I2C Scan...", 0xffff,0,FONT_8x8, DRAW_TO_LCD);
  I2CScan(&i2c, map); // get bitmap of connected I2C devices
  if (map[0] == 0xfe) // something is wrong with the I2C bus
  {
    spilcdWriteString(&lcd, 0,8,(char *)"I2C pins are not correct", 0xf800,0,FONT_8x8,DRAW_TO_LCD);
    spilcdWriteString(&lcd, 0,16,(char *)"or bad device; scan failed", 0xf800,0,FONT_8x8,DRAW_TO_LCD);
  }
  else
  {
    for (i=1; i<128; i++) // skip address 0 (general call address) since more than 1 device can respond
    {
      if (map[i>>3] & (1 << (i & 7))) // device found
      {
        iCount++;
        iDevice = I2CDiscoverDevice(&i2c, i);
        sprintf(szTemp, "Device at 0x%x: %s", i, szNames[iDevice]);
        spilcdWriteString(&lcd, 0,iCount*8,szTemp, 0xf81f,0,FONT_6x8,DRAW_TO_LCD);
      }
    } // for i
    sprintf(szTemp, "%d device(s) found", iCount);
    spilcdWriteString(&lcd, 0,(iCount+1)*8,szTemp,0x6e0,0,FONT_8x8,DRAW_TO_LCD);
  }
  spilcdWriteString(&lcd, 0,72,(char *)"Press any button to exit", 0xffe0,0,FONT_6x8, DRAW_TO_LCD);
  while (GetButtons() != 0) // wait until any pressed are released
  { };
  
  while (GetButtons() == 0) // wait until any are pressed
  { };
} /* I2CDetect() */

int GetButtons(void)
{
static int iOldState;
int iState, iPressed;
int i, j;
int iCounts[4] = {0};

  iState = 0;
  if (bConnected) // use the gamepad
  {
    if (gp.u16Buttons & 0x20) // down
      iState |= 1;
    if (gp.u16Buttons & 0x10) // up
      iState |= 2;
    if (gp.u16Buttons & 4) // (1) button
      iState |= 4;
    if (gp.u16Buttons & 8) // (2) button
      iState |= 8;
  }
  else // use the touch buttons
  {
    // remove the 'flicker' of the buttons
    for (j=0; j<5; j++)
    {
      for (i=0; i<4; i++)
      {
        if (touchRead(iButtons[i]) < BUTTON_THRESHOLD)
          iCounts[i]++;
      }
      delay(4);
    }
    for (i=0; i<4; i++)
      if (iCounts[i] == 5)
         iState |= (1<<i);
  }    
    // Test button bits for which ones changed from LOW to HIGH
    iPressed = (iState ^ iOldState) & iState; // tells us which ones just changed to 1
    iOldState = iState;
    iPressed = iPressed | (iState << 8); // prepare combined state
    return iPressed; 
} /* GetButtons() */

void ButtonTest(void)
{
int iCount = 0;
int iButts;
int i;
uint16_t usColor;
char szTemp[16];

  spilcdFill(&lcd, 0,DRAW_TO_LCD);
  spilcdWriteString(&lcd, 36,0,(char *)"Button Test", 0xffff,0,FONT_8x8, DRAW_TO_LCD);
  spilcdWriteString(&lcd, 8,72,(char *)"Press 2 buttons to exit", 0xffff,0,FONT_6x8, DRAW_TO_LCD);
  while (iCount < 2)
  {
    iButts = GetButtons() >> 8; // get the currently pressed bits
    iCount = 0;
    for (i=0; i<4; i++)
    {
      if (iButts & 1)
      {
        usColor = 0xf800; // red for pressed
        iCount++; 
      }
      else
      {
        usColor = 0x6e0; // green for not pressed
      }
      sprintf(szTemp, "T%d", i+1);
      spilcdWriteString(&lcd, i*40,24,szTemp,usColor,0,FONT_16x32,DRAW_TO_LCD);
      iButts >>= 1;
    } // for i
  }
} /* ButtonTest() */

void DrawMainMenu(int iMenuItem)
{
static int i = 0;
uint16_t iFG, iBG;
char *pGP;
uint8_t u8Temp[8*40]; // holds the rotated mask
static int x[4], y[4], dx[4], dy[4];
int j;
uint16_t pal[4] = {0xffe0,0xf81f,0x1f,0xffff};
  if (i == 0) // first time through
  {
    for (j=0; j<4; j++)
    {
      x[j] = random(WIDTH-40);
      y[j] = random(HEIGHT-40);
      dx[j] = (j & 1) ? -1:1;
      dy[j] = (j & 1) ? -1:1;
    }
  }
  iFG = 0xffff; iBG = 0x6e0;
  spilcdRectangle(&lcd, 0, 0, WIDTH, HEIGHT, 0x0000, 0xf800, 1, DRAW_TO_RAM);
  spilcdWriteString(&lcd, 44,0,(char *)"Main Menu", 0x6ff,-1,FONT_8x8, DRAW_TO_RAM);
  spilcdWriteString(&lcd, 0,16,(char *)"I2C Scan", (iMenuItem == 0) ? iFG:iBG,-1,FONT_8x8, DRAW_TO_RAM);
  spilcdWriteString(&lcd, 0,24,(char *)"WiFi Scan", (iMenuItem == 1) ? iFG:iBG,-1,FONT_8x8, DRAW_TO_RAM);
  spilcdWriteString(&lcd, 0,32,(char *)"BLE Scan", (iMenuItem == 2) ? iFG:iBG,-1,FONT_8x8, DRAW_TO_RAM);
  spilcdWriteString(&lcd, 0,40,(char *)"Button Test", (iMenuItem == 3) ? iFG:iBG,-1,FONT_8x8, DRAW_TO_RAM);
  spilcdWriteString(&lcd, 0,48,(char *)"Proximity Test", (iMenuItem == 4) ? iFG:iBG,-1,FONT_8x8, DRAW_TO_RAM);
  spilcdWriteString(&lcd, 0,56,(char *)"IMU Test", (iMenuItem == 5) ? iFG:iBG,-1,FONT_8x8, DRAW_TO_RAM);
  spilcdWriteString(&lcd, 0,64,(char *)"Temp/Humidity Test", (iMenuItem == 6) ? iFG:iBG,-1,FONT_8x8, DRAW_TO_RAM);
  if (bConnected)
     pGP = (char *)"Gamepad: disconnect";
  else
     pGP = (char *)"Gamepad: connect";
  spilcdWriteString(&lcd, 0,72,pGP, (iMenuItem == 7) ? iFG:iBG,-1,FONT_8x8, DRAW_TO_RAM);
  spilcdRotateBitmap(ucBombMask, u8Temp, 1, 40, 40, 8, 20, 20, i % 360);
  for (j=0; j<4; j++)
  {
    spilcdDrawPattern(&lcd, u8Temp, 8, x[j], y[j],40,40,pal[j],16);
    x[j] += dx[j]; y[j] += dy[j];
    if (x[j] <= 0 || x[j] >= WIDTH-40)
       dx[j] = -dx[j];
    if (y[j] <= 0 || y[j] >= HEIGHT-40)
       dy[j] = -dy[j];
  }
  spilcdShowBuffer(&lcd, 0,0,WIDTH,HEIGHT, DRAW_TO_LCD);
  i++; 
} /* DrawMainMenu() */

// Number of grains of sand - this is how many pixels in the first line of text
#define N_GRAINS     720
// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  uint16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
  uint16_t color; // pixel color
} grain[N_GRAINS];

void ResetGrains(int bRandom)
{
int i, j, x, y;
uint16_t *pBitmap = spilcdGetBuffer(&lcd);
uint16_t color, Pal[] = {0xf800,0xffff,0xffe0,0xf81f,0x1f,0x6e0,0x6ff,0xaaaa};

  spilcdFill(&lcd, 0, DRAW_TO_LCD | DRAW_TO_RAM);
  if (bRandom)
  {
    for(i=0; i<N_GRAINS; i++) {  // For each sand grain...
      do {
        grain[i].x = random(WIDTH  * 256); // Assign random position within
        grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
        // Check if corresponding pixel position is already occupied...
        for(j=0; (j<i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                           ((grain[i].y / 256) != (grain[j].y / 256))); j++);
      } while(j < i); // Keep retrying until a clear spot is found
      x = grain[i].x / 256; y = grain[i].y / 256;
      // because the display is rotated 90
      grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
      grain[i].color = Pal[random(7)];
      pBitmap[x + (y*WIDTH)] = grain[i].color; // Mark it
    }
  } // random
  else
  {
    spilcdWriteString(&lcd, 40,28,(char *)"C",0xf800,0,FONT_16x32,DRAW_TO_LCD | DRAW_TO_RAM);
    spilcdWriteString(&lcd, 56,28,(char *)"O",0x6e0,0,FONT_16x32,DRAW_TO_LCD | DRAW_TO_RAM);
    spilcdWriteString(&lcd, 72,28,(char *)"L",0x1f,0,FONT_16x32,DRAW_TO_LCD | DRAW_TO_RAM);
    spilcdWriteString(&lcd, 88,28,(char *)"O",0xf81f,0,FONT_16x32,DRAW_TO_LCD | DRAW_TO_RAM);
    spilcdWriteString(&lcd, 104,28,(char *)"R",0xffe0,0,FONT_16x32,DRAW_TO_LCD | DRAW_TO_RAM);
    i = 0;
    for (y=0; y<HEIGHT; y++)
    {
      for (x=0; x<WIDTH; x++)
      {
        color = pBitmap[x + (y*WIDTH)];
        if (color != 0) // pixel set?
        {
          grain[i].x = x*256; grain[i].y = y*256;
          grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
          grain[i].color = color;
          i++;
          if (i == N_GRAINS) return;
        }
      } // for x
    } // for y
//    Serial.println(i, DEC);
    
  }
  
} /* ResetGrains() */

void IMUTest(void)
{
int16_t x, y, z;
int32_t v2; // Velocity squared
int16_t ax, ay, az;
//signed int        oldidx, newidx;
signed int        newx, newy;
signed int        x1, y1, x2, y2;
int i;
uint16_t *pBitmap = spilcdGetBuffer(&lcd);
uint16_t u16Flags[5]; // divide the display into 16x16 blocks for quicker refresh
int iFrame = 0;
uint32_t frac, ysum, oldy;

  ResetGrains(0);
  delay(2000);
  memset(u16Flags,0,sizeof(u16Flags));
  while (1)
  {
    iFrame++;
    if ((iFrame & 7) == 0)
    {
      if (GetButtons())
       return;
    }
//    if (iFrame & 1) // update display if we didn't check the buttons
//    {
//      spilcdShowBuffer(&lcd, 0,0,WIDTH,HEIGHT, DRAW_TO_LCD);
//    }
    if (bConnected) // use the left analog stick to simulate gravity
    {
      ax = gp.iLJoyX / 4;
      ay = gp.iLJoyY / 4;
    }
    else // use the accelerometer
    {
      LIS3DHReadAccel(&x, &y, &z);
      ax = x / 512; // Transform accelerometer axes
      ay = -y / 512;      // to grain coordinate space
      az = abs(z) / 2048; // Random motion factor
      az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
      ax -= az;                         // Subtract motion factor from X, Y
      ay -= az;
    }
  // Apply 2D accelerometer vector to grain velocities...
  //
  // Theory of operation:
  // if the 2D vector of the new velocity is too big (sqrt is > 256), this means it might jump
  // over pixels. We want to limit the velocity to 1 pixel as a maximum.
  // To avoid using floating point math (sqrt + 2 multiplies + 2 divides)
  // Instead of normalizing the velocity to keep the same direction, we can trim the new
  // velocity to 5/8 of it's value. This is a reasonable approximation since the maximum
  // velocity impulse from the accelerometer is +/-64 (16384 / 256) and it gets added every frame
  //
  spilcdSetPosition(&lcd, 0, 0, WIDTH, HEIGHT, DRAW_TO_LCD);
  ysum = 0;
  oldy = 0xffff;
  frac = (HEIGHT * 65536) / N_GRAINS;
  // Update the position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)
  //
  // (x,y) to bytes mapping:
  // The SSD1306 has 8 rows of 128 bytes with the LSB of each byte at the top
  // In other words, bytes are oriented vertically with bit 0 as the top pixel
  // Part of my optimizations were writing the pixels into memory the same way they'll be
  // written to the display. This means calculating an offset and bit to test/set each pixel
  //
  for(i=0; i<N_GRAINS; i++) {
    int16_t vx, vy; // use local vars for more speed
    ysum += frac;
    if ((ysum >> 16) != oldy) // update the display
    {
      oldy = ysum >> 16;
      spilcdWriteDataBlock(&lcd, (uint8_t *)&pBitmap[oldy*WIDTH], WIDTH*2, DRAW_TO_LCD | DRAW_WITH_DMA);
    }
    vx = grain[i].vx + ax;// + random(5); // Add a little random impulse to each grain
    vy = grain[i].vy + ay;// + random(5);
    v2 = (int32_t)(vx*vx) + (int32_t)(vy*vy);
    if (v2 >= 400000) // too big, trim it
    {
      vx = (vx * 5)/8; // quick and dirty way to avoid doing a 'real' divide
      vy = (vy * 5)/8;
    }
    newx = grain[i].x + vx; // New position in grain space
    newy = grain[i].y + vy;
    if(newx > MAX_X) {               // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      vx /= -2;             // give a slight bounce off the wall
    } else if(newx < 0) {
      newx         = 0;
      vx /= -2;
    }
    if(newy > MAX_Y) {
      newy         = MAX_Y;
      vy /= -2;
    } else if(newy < 0) {
      newy         = 0;
      vy /= -2;
    }

    x1 = grain[i].x / 256; y1 = grain[i].y / 256; // old position
    x2 = newx / 256; y2 = newy / 256;
    if((x1 != x2 || y1 != y2) && // If grain is moving to a new pixel...
        (pBitmap[x2 + (y2*WIDTH)] != 0)) {       // but if that pixel is already occupied...
        // Try skidding along just one axis of motion if possible (start w/faster axis)
        if(abs(vx) > abs(vy)) { // X axis is faster
          y2 = grain[i].y / 256;
          if(pBitmap[x2 + (y2*WIDTH)] == 0) { // That pixel's free!  Take it!  But...
            newy         = grain[i].y; // Cancel Y motion
            vy = (vy /-2) + random(8);         // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            y2 = newy / 256; x2 = grain[i].x / 256;
            if(pBitmap[x2 + (y2*WIDTH)] == 0) { // Pixel is free, take it, but first...
              newx         = grain[i].x; // Cancel X motion
              vx = (vx /-2) + random(8);         // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              vx = (vx /-2) + random(8);         // Bounce X & Y velocity
              vy = (vy /-2) + random(8);
            }
          }
        } else { // Y axis is faster
          y2 = newy / 256; x2 = grain[i].x / 256;
          if(pBitmap[x2 + (y2*WIDTH)] == 0) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x; // Cancel X motion
            vx = (vx /-2) + random(8);        // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            y2 = grain[i].y / 256; x2 = newx / 256;
            if(pBitmap[x2 + (y2*WIDTH)] == 0) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              vy = (vy /-2) + random(8);        // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              vx = (vx /-2) + random(8);         // Bounce X & Y velocity
              vy = (vy /-2) + random(8);
            }
          }
        }
    }
    grain[i].x  = newx; // Update grain position
    grain[i].y  = newy; // possibly only a fractional change
    grain[i].vx = vx;
    grain[i].vy = vy;
    y2 = newy / 256; x2 = newx / 256;
    if (x1 != x2 || y1 != y2)
    {
      pBitmap[x1 + (y1*WIDTH)] = 0; // erase old pixel
      pBitmap[x2 + (y2*WIDTH)] = grain[i].color;  // Set new pixel
    }
  } // for i
  // display the last line since the fractional sum won't reach it
  spilcdWriteDataBlock(&lcd, (uint8_t *)&pBitmap[(HEIGHT-1)*WIDTH], WIDTH*2, DRAW_TO_LCD | DRAW_WITH_DMA);

  } // while (1)
} /* IMUTest() */

//
// Temperature and humidity test
//
void TempTest(void)
{
    char szTemp[32];
    uint8_t ucTemp[4];
    uint32_t T, H;
    temp_addr = 0x40;
    spilcdFill(&lcd, 0,DRAW_TO_LCD);
    spilcdWriteString(&lcd, 0,72,(char *)"Press any button to exit", 0xffe0,0,FONT_6x8, DRAW_TO_LCD);
    // configure it
    ucTemp[0] = 0x02; // config register
    ucTemp[1] = 0x10; // temp+humidity, 14-bit resolution
    ucTemp[2] = 0x00;
    I2CWrite(&i2c, temp_addr, ucTemp, 3);
    delay(100);
    while (GetButtons() == 0)
    {
      ucTemp[0] = 0x00;
      I2CWrite(&i2c, temp_addr, ucTemp, 1); // read temp register
      delay(15); // wait for conversion we just triggered
      I2CRead(&i2c, temp_addr, ucTemp, 4); // read temp+humidity
      T = (ucTemp[0]<<8) + ucTemp[1];
      H = (ucTemp[2]*256) + ucTemp[3];
      // convert to percent
      H = (H * 100) >> 16;
      T = ((T * 1650) >> 16) - 400; // 10x temp
      T -= 84; // adjustment that seems to give a more accurate result
      sprintf(szTemp,"Humidity: %d%%", H); // display humidity value
      spilcdWriteString(&lcd, 0,0,szTemp,0xffff,0,FONT_8x8,DRAW_TO_LCD);
      sprintf(szTemp,"Air Temp = %d.%dC", (int)(T/10), (int)(T % 10)); // display temperature value
      spilcdWriteString(&lcd, 0,8,szTemp,0xffff,0,FONT_8x8, DRAW_TO_LCD);
      T = temprature_sens_read();
      if (T == 128) // invalid, must not be present
         spilcdWriteString(&lcd, 0,16,(char *)"CPU Temp = N/A", 0xffff,0,FONT_8x8,DRAW_TO_LCD);
      else
      {
        sprintf(szTemp,"CPU Temp = %dF", T); // display CPU temperature
        spilcdWriteString(&lcd, 0,16,szTemp,0xffff,0,FONT_8x8, DRAW_TO_LCD);
      }
    }
} /* TempTest() */

void TOFTest()
{
char szTemp[32];
VL53L0X_RangingMeasurementData_t measure;
int iFrame = 0;
int16_t butts = 0;

  spilcdFill(&lcd, 0,DRAW_TO_LCD); // fill to black
  spilcdWriteString(&lcd, 0,72,(char *)"Press any button to exit", 0xffe0,0,FONT_6x8, DRAW_TO_LCD);
  while (1)
  {
    iFrame++;
    if ((iFrame & 7) == 0) // check buttons every once in a while
       butts = GetButtons();
    if ((butts & 0xff) != 0) return; // exit at first button press
    vlx.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    sprintf(szTemp, "Dist = %03dmm", measure.RangeMilliMeter);
    spilcdWriteString(&lcd, 0,0,szTemp, 0xf81f, 0, FONT_16x32, DRAW_TO_LCD);
  }
} /* TOFTest() */

#define MAX_APS 8
void WiFiScan()
{
String ssidList[MAX_APS]; // top SSIDs found in the area
uint8_t ssidEncrypt[MAX_APS];
int ssidRSSI[MAX_APS];
int i, n;
char szTemp[64];

  spilcdFill(&lcd, 0, DRAW_TO_LCD);
  spilcdWriteString(&lcd, 0,0,(char *)"Scanning Wifi...", 0xffff,0,FONT_8x8, DRAW_TO_LCD);

  // Find nearby networks (up to 9)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // make sure we're not connected to a network
  delay(100);
  // workaround for a bug which doesn't power wifi unless we try to connect to something
//  WiFi.begin();
  // WiFi.scanNetworks will return the number of networks found
//  WiFi.scanDelete(); // delete the last scan result from memory
  n = WiFi.scanNetworks();
  sprintf(szTemp,"Found: %d", n);
  if (n > MAX_APS)
    strcat(szTemp, (char *)" (showing top 8)");
  if (n > MAX_APS) n = MAX_APS; // we only care about the N strongest
  spilcdFill(&lcd, 0,DRAW_TO_LCD);
  spilcdWriteString(&lcd, 0,0,szTemp, 0xffff,0, FONT_8x8, DRAW_TO_LCD);
  for (i = 0; i < n; ++i)
  {
     ssidList[i] = WiFi.SSID(i);
     ssidEncrypt[i] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
     ssidRSSI[i] = WiFi.RSSI(i);
     sprintf(szTemp, "%s, %c, %ddBm", ssidList[i].c_str(), (ssidEncrypt[i]) ? '*':' ',ssidRSSI[i]);
     spilcdWriteString(&lcd, 0,8+(i*8), szTemp, 0x6e0,0,FONT_6x8, DRAW_TO_LCD);
  } // for each ssid found
  spilcdWriteString(&lcd, 0,72,(char *)"Press any button to exit", 0xffe0,0,FONT_6x8, DRAW_TO_LCD);
  while (GetButtons() == 0)
  {
    delay(10);
  }
} /* WiFiScan() */

static BLEAddress *Server_BLE_Address;
static char Scanned_BLE_Name[10][32];
static char Scanned_BLE_Address[10][32];
static int iBLECount;

// Called for each device found during a BLE scan by the client
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice) {
//      Serial.printf("Scan Result: %s \n", advertisedDevice.toString().c_str());
//      if (strcmp(Band_BLE_Name.c_str(), advertisedDevice.getName().c_str()) == 0) { // this is what we want
        Server_BLE_Address = new BLEAddress(advertisedDevice.getAddress());
        strcpy(Scanned_BLE_Address[iBLECount], Server_BLE_Address->toString().c_str());
        strcpy(Scanned_BLE_Name[iBLECount], advertisedDevice.getName().c_str());
        delete Server_BLE_Address;
        iBLECount++;
//      }
    }
};

void BLEScan()
{
char szTemp[64];
int i;

  iBLECount = 0;
  spilcdFill(&lcd, 0, DRAW_TO_LCD);
  spilcdWriteString(&lcd, 0,0,(char *)"Scanning for BLE... ", 0xffff,0,FONT_8x8, DRAW_TO_LCD);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  foundDevices = pBLEScan->start(10); // scan for 10 seconds
  sprintf(szTemp, "%d device(s) found  ", iBLECount);
  spilcdWriteString(&lcd, 0,0,szTemp, 0xffff,0,FONT_8x8, DRAW_TO_LCD);
  for (i=0; i<iBLECount; i++)
  {
    sprintf(szTemp, "%s,%s", Scanned_BLE_Address[i], Scanned_BLE_Name[i]);
    spilcdWriteString(&lcd, 0,(i+1)*8,szTemp, 0x6e0,0,FONT_6x8, DRAW_TO_LCD);
  }
  pBLEScan->stop(); // stop scanning
  spilcdWriteString(&lcd, 0,72,(char *)"Press any button to exit", 0xffe0,0,FONT_6x8, DRAW_TO_LCD);
  while (GetButtons() == 0)
  {
    delay(10);
  }
} /* BLEScan() */

// Connect or disconnect the gamepad
void Gamepad(void)
{
  if (bConnected) // disconnect
  {
    SS_Disconnect();
  }
  else // connect it
  {
    spilcdFill(&lcd, 0, DRAW_TO_LCD | DRAW_TO_RAM);
    spilcdWriteString(&lcd, 0,0,(char *)"Starting discovery...", 0xffff, 0, FONT_8x8, DRAW_TO_LCD);
    if (SS_Scan(6, devAddr))
    {    
      spilcdWriteString(&lcd, 0,16,(char *)"SS found!", 0x6e0, 0, FONT_16x16, DRAW_TO_LCD);
      if (SS_Connect(devAddr))
      {
        spilcdWriteString(&lcd, 0,32,(char *)"Connected!", 0x6e0, 0, FONT_16x16, DRAW_TO_LCD);
      }
      else
        spilcdWriteString(&lcd, 0,32,(char *)"Not Connected", 0xf800, 0, FONT_16x16, DRAW_TO_LCD);
    }
    else
    {
      spilcdWriteString(&lcd, 0,16,(char *)"Not found", 0xf800, 0, FONT_16x16, DRAW_TO_LCD);
    }    
    delay(2000);
  }
} /* Gamepad() */

void MainMenu(void)
{
int iButts, iMenuItem = 0;
int iFrame = 0;
  iButts = 0;
  while (1)
  {
     DrawMainMenu(iMenuItem);
     if (iFrame & 1)
       iButts = GetButtons();
     if (iButts & 1 && iMenuItem < 7) // first button = down
     {
        iMenuItem++;
     }
     else if (iButts & 2 && iMenuItem > 0) // second button = up
     {
        iMenuItem--;
     }
     else if (iButts & 4) // action
     {
      switch (iMenuItem)
      {
         case 0:
          I2CDetect();
          break;
         case 1:
           WiFiScan();
           break;
         case 2:
           BLEScan();
           break;
         case 3:
           ButtonTest();
           break;
         case 4:
          TOFTest();
          break;
         case 5:
          IMUTest();
          break;
         case 6:
          TempTest();
          break;
         case 7:
          Gamepad();
          break;
       }
     }
     iButts = 0; // don't let it double press on even frames
     iFrame++;
  }
} /* MainMenu() */

void gotTouch1()
{
//  iButtons[0]++;
}
void gotTouch2()
{
//  iButtons[1]++;
}
void gotTouch3()
{
//  iButtons[2]++;
}
void gotTouch4()
{
//  iButtons[3]++;
}

void setup() {
  Serial.begin(115200);
  i2c.iSDA = -1;
  i2c.iSCL = -1;
  i2c.bWire = 1;
  I2CInit(&i2c, 400000L);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50); // need this small delay or the vlx.begin will hang
  SS_Init();
  SS_RegisterCallback(SS_Callback);
  bConnected = false;
  LIS3DHInit(0x19);
  vlx.begin();
  spilcdSetTXBuffer(txBuffer, 4096);
  spilcdInit(&lcd, LCD_ST7735S_B, FLAGS_INVERT | FLAGS_SWAP_RB, 32000000, 4, 21, 22, 26, -1, 23, 18); // Mike's coin cell pin numbering
  spilcdSetOrientation(&lcd, LCD_ORIENTATION_90);
  spilcdAllocBackbuffer(&lcd);
//  touchAttachInterrupt(T7, gotTouch1, threshold);
//  touchAttachInterrupt(T5, gotTouch2, threshold);
//  touchAttachInterrupt(T3, gotTouch3, threshold);
//  touchAttachInterrupt(T2, gotTouch4, threshold);
} /* setup() */

void loop() {
  // put your main code here, to run repeatedly:
int i, j, x, y;

#define WIDTH 160
#define HEIGHT 80

MainMenu();
  I2CDetect();
//TryConnect();
while (1) {};

static int iColorOffset = 0;
static int iColorDelta = 1;
static int yLogo = 0;
static int yDelta = 1;
int dx, dy;

x = y = 0;
dx = dy = 1;
i = 0; // frame
while (1)
{
uint16_t r, g, b, usColor1, usColor2;
      // color offset goes between 0 and 0x20
      // use to transition between 2 starting and 2 ending colors for the gradient
      r = iColorOffset;
      g = 1-((1*iColorOffset)>>5);
      b = 0x1f - iColorOffset;
      usColor1 = (r<<11) | (g<<5) | b; // start
      r = 0x1f - iColorOffset;
      g = 0x32 - (((0x17)*iColorOffset)>>5);
      b = iColorOffset;
      usColor2 = (r<<11) | (g<<5) | b; // end
      iColorOffset += iColorDelta; // increase green
      if (iColorOffset < 0)
         {
         iColorOffset = 0;
         iColorDelta = -iColorDelta;
         }
      if (iColorOffset >= 0x1f)
         {
         iColorOffset = 0x1f;
         iColorDelta = -iColorDelta;
         }
      yLogo += yDelta;
      if (yLogo == 0 || yLogo == 155)
         yDelta = -yDelta;

    spilcdRectangle(&lcd, 0, 0, WIDTH, HEIGHT, usColor1, usColor2, 1, 0);
    spilcdWriteString(&lcd, x,y,(char *)"Mike's Magical", 0xffff,-1,FONT_8x8, DRAW_TO_RAM);
    spilcdWriteString(&lcd, x,y+8,(char *)"Color CoinCell!", 0xffff,-1,FONT_8x8, DRAW_TO_RAM);
//void spilcdDrawPattern(uint8_t *pPattern, int iSrcPitch, int iDestX, int iDestY, int iCX, int iCY, uint16_t usColor, int iTranslucency);
    spilcdDrawPattern(&lcd, ucBombMask, 8, x+16,y,40,40,0x6e0,((y>>1)&31)+1);
    spilcdDrawPattern(&lcd, ucBombMask, 8, x+48,y,40,40,0xf800,((y>>1)&31)+1);
    spilcdDrawPattern(&lcd, ucBombMask, 8, x+80,y,40,40,0x1f,((y>>1)&31)+1);
    {
      uint8_t u8Temp[8*40]; // holds the rotated mask
      spilcdRotateBitmap(ucBombMask, u8Temp, 1, 40, 40, 8, 20, 20, i % 360);
      j = i % 120;
      spilcdDrawPattern(&lcd, u8Temp, 8, j,0,40,40,0x6ff,16);
      spilcdDrawPattern(&lcd, u8Temp, 8, 120-j,0,40,40,0xf81f,16);
    }
//  spilcdDrawBMP((uint8_t *)ucLogoBMP, 8, yLogo, 0, 0, 0);
  spilcdShowBuffer(&lcd, 0,0,WIDTH,HEIGHT, DRAW_TO_LCD);
  if (i & 1) // change pos every other frame
  {
    x += dx; y += dy;
    if (x == 0) dx = -dx;
    else if (x >= 39) dx = -dx;
    if (y == 0) dy = -dy;
    else if (y > 63) dy = -dy;
  }
  i++;
} // while (1)

} // loop
