#include "icons/lowBattery200x200.h"

#include <GxEPD2_BW.h>

#define MAX_DISPLAY_BUFFER_SIZE 800
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS=*/ 15, /*DC=*/ 27, /*RST=26*/ 26, /*BUSY=*/ 25));

#define BAUDRATE 9600

void setupDisplay()
{
  display.init(BAUDRATE); // uses standard SPI pins, e.g. SCK(18), MISO(19), MOSI(23), SS(5)
  SPI.end(); // release standard SPI pins
  SPI.begin(13, 12, 14, 15); // map and init SPI pins SCK(13), MISO(12), MOSI(14), SS(15)
  display.setRotation(1); // Display is installed upside down
  Serial.println("Display ready");
}

void setup()
{
  Serial.begin(BAUDRATE);
  
  setupDisplay();
  drawLowBattery();
}

void drawLowBattery()
{
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.drawInvertedBitmap(0, 0, low_battery, display.epd2.WIDTH, display.epd2.HEIGHT, GxEPD_BLACK);
  }
  while (display.nextPage());
}

void loop()
{

}
