#include "icons/lowBattery200x200.h"

#include <GxEPD2_BW.h>

#define MAX_DISPLAY_BUFFER_SIZE 800
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS=*/ 15, /*DC=*/ 27, /*RST=26*/ 26, /*BUSY=*/ 25));

#include "DHT.h"
DHT dht(GPIO_NUM_5, DHT22);

const int CO2_RX_PIN = GPIO_NUM_16;
const int CO2_TX_PIN = GPIO_NUM_17;
const int CO2_POWER_PIN = GPIO_NUM_26;
const int CO2_POWER_PIN_FREQ = 5000;
const int CO2_POWER_PIN_CH = 0;
const int CO2_POWER_PIN_CH_RES = 8;

HardwareSerial co2Serial(1);

byte co2Response[7];

const int READ_ERROR = -1;

void co2_write_request()
{
  byte co2ReadRequest[8] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};

  while (co2Serial.available()) // Flush whatever we have before writing
    co2Serial.read();
  co2Serial.write(co2ReadRequest, 8);
}

int co2_read_response()
{
  unsigned long startTime = millis();
  int waitMs = 500;
  int waitBytes = 7;
  
  while (co2Serial.available() < waitBytes) //Wait full response
  {
    unsigned long timeout = (unsigned long)((long)millis() - (long)startTime);
    if(waitMs < timeout) {       
      return READ_ERROR;
    }
  }

  for (int i = 0; i < 7; i++)
  {
    co2Response[i] = co2Serial.read();
  }

  return waitBytes;
}

int co2_get_measurement()
{
  co2_write_request();
  if (co2_read_response() != READ_ERROR)
  {
    Serial.println("CO2 Response: ");
    for (int i = 0; i < 7; i++) {
      Serial.print(co2Response[i], HEX);
      Serial.print(" ");    
    }
    Serial.println();
      
    int high = co2Response[3];
    int low = co2Response[4];
  
    return high * 256 + low;   
  } else {
    Serial.println("CO2 Read error!");
    return READ_ERROR;
  }
}

void draw_low_battery()
{
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.drawInvertedBitmap(0, 0, low_battery, display.epd2.WIDTH, display.epd2.HEIGHT, GxEPD_BLACK);
  }
  while (display.nextPage());
}

float get_humidity()
{
  float humidity = dht.readHumidity();
  Serial.println("Humidity: " + String(humidity));
  return humidity;
}

float get_temperature()
{
  float temperature = dht.readTemperature();
  Serial.println("Temperature: " + String(temperature));
  return temperature;
}

void co2_power_on()
{
  ledcWrite(CO2_POWER_PIN_CH, 255);
  delay(50);
  Serial.println("CO2 Power ON");
}

void co2_power_off()
{
  ledcWrite(CO2_POWER_PIN_CH, 0);
  delay(50);
  Serial.println("CO2 Power OFF");
}

void setup()
{
  Serial.begin(9600);

  // Display
  display.init(9600); // uses standard SPI pins, e.g. SCK(18), MISO(19), MOSI(23), SS(5)
  SPI.end(); // release standard SPI pins
  SPI.begin(13, 12, 14, 15); // map and init SPI pins SCK(13), MISO(12), MOSI(14), SS(15)
  display.setRotation(1); // Display is installed upside down
  Serial.println("Display ready");
  
  draw_low_battery();

  // DHT
  dht.begin();

  // CO2
  ledcSetup(CO2_POWER_PIN_CH, CO2_POWER_PIN_FREQ, CO2_POWER_PIN_CH_RES);
  ledcAttachPin(CO2_POWER_PIN, CO2_POWER_PIN_CH);
  co2Serial.begin(9600, SERIAL_8N1, CO2_RX_PIN, CO2_TX_PIN);
}

void loop()
{
  delay(50);
  get_humidity();
  delay(50);
  get_temperature();
  
  co2_power_on();
  // 10 second warmup
  delay(10000);
  int co2 = co2_get_measurement();
  Serial.println("CO2 measurement: " + String(co2));  
  co2_power_off();
  
  delay(5000);
}
