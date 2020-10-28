# ESP32_CoinCell_Color_TFT

![Top](https://user-images.githubusercontent.com/4991664/97198353-75da4b00-178d-11eb-99a8-423e4afa3d99.png)
![Bottom](https://user-images.githubusercontent.com/4991664/97198461-930f1980-178d-11eb-9ccb-2a1128894bd0.png)
![kit](https://user-images.githubusercontent.com/4991664/97461608-ab616e80-191c-11eb-923d-81e30eafeb2a.jpg)

Based off of the CoinCell Board but with more sensors and a 80x160 ST7735 color tft lcd

This ESP32 internet of things dev board with has an accelerometer, 80x160 pixel 0.96" color TFT LCD display, RGB led, temperature/humidity sensor, laser range sensor and LiPo battery protection. The very bottom edge has four capacitive touch sensors for menu navigation and is powered by a rechargeable LIR2450 coin cell, external LiPo battery or micro usb cable. When a battery is connected and you plug in the micro usb cable, it charges the battery. It was not made for any specific purpose and was really more of a design challenge to try and make it as small as possible with plenty of sensors. The design files and parts list are provided on my Github site if you would like to assemble your own. Each board is hand assembled by myself using a soldering iron and hot air wand. Hand assembly is complicated. Components except for the sensors and display are placed, tested and then the board is washed, sensors are placed, tested and cleaned again, then the display is soldered into place and tested.

As expected, battery using a rechargeable 2450 coincell is very, very poor so I've added a connector for a larger LiPo battery. Current draw awake with the ESP awake, the tft, accelerometer, temp/humidity and range sensor on is around 70mA and only lasts around 10 minutes. Wifi current draw is significantly more. In sleep mode current draw is around 220uA and waking every 10 minutes to grab sensor data, use wifi and turn on the display it on the screen lasts around 12 hours or so. Using a larger external LiPo battery is highly recommended.

Hardware design instead of software is my strength so at this state the only a few simple Arduino sketches are available that test out the hardware.

Hardware on this dev board are:
-temperature/humidity sensor (HDC1080DMBT)
-Laser range sensor (VL53L0CXV0DH)
-accelerometer (LIS3DHTR)
-LiPo protection (DW01A)
-80x160 TFT LCD (ER-TFT0.96-1)
-3.3V LDO (HT7833)
-LiPo charger (SL4054ST25P)
-USB interface (CP2104N)

If  you decice to purchase this project from Tindie XXXXXX you get an assembled and tested board with coin cell holder not soldered in place along with a microJST cable to use with an external battery.



