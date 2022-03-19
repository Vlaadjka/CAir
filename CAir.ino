#include "icons/lowBattery200x200.h"

#include <GxEPD2_BW.h>
#include "fonts/BebasNeueRegular60pt7b.h"
#include "fonts/BebasNeueRegular22pt7b.h"
#include "fonts/BebasNeueRegular14pt7b.h"
#include "icons/humidity20x30.h"

#include "driver/rtc_io.h"

#define MAX_DISPLAY_BUFFER_SIZE 800
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(/*CS=*/ 15, /*DC=*/ 27, /*RST=26*/ 26, /*BUSY=*/ 25));

#include "DHT.h"
DHT dht(GPIO_NUM_5, DHT22);

const int CO2_RX_PIN = GPIO_NUM_16;
const int CO2_TX_PIN = GPIO_NUM_17;
const gpio_num_t CO2_POWER_PIN = GPIO_NUM_26;
const int CO2_POWER_PIN_FREQ = 5000;
const int CO2_POWER_PIN_CH = 0;
const int CO2_POWER_PIN_CH_RES = 8;

const int VOLTAGE_PIN = GPIO_NUM_35;
const float VOLTAGE_DIVIDER_CONSTANT = 0.001755;
const float VOLTAGE_LOW = 3.5;

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
  rtc_gpio_hold_dis(CO2_POWER_PIN);
  ledcWrite(CO2_POWER_PIN_CH, 255);
  delay(50);
  rtc_gpio_hold_en(CO2_POWER_PIN);
  Serial.println("CO2 Power ON");
  co2Serial.begin(9600, SERIAL_8N1, CO2_RX_PIN, CO2_TX_PIN);
}

void co2_power_off()
{
  rtc_gpio_hold_dis(CO2_POWER_PIN);
  ledcWrite(CO2_POWER_PIN_CH, 0);
  delay(50);
  rtc_gpio_hold_en(CO2_POWER_PIN);
  Serial.println("CO2 Power OFF");
}

void draw_data(int co2, float temp, float humidity, float voltage)
{ 
  char str_co2[5], str_temp[5], str_humidity[3], str_voltage[5];
  char co2Text[5], tempText[7], humidityText[3], voltageText[6];

  String(co2).toCharArray(str_co2, sizeof(str_co2));
  String(temp).toCharArray(str_temp, sizeof(str_temp));
  String(humidity).toCharArray(str_humidity, sizeof(str_humidity));
  String(voltage).toCharArray(str_voltage, sizeof(str_voltage));

  snprintf(co2Text, sizeof(co2Text), "%s", str_co2);
  snprintf(tempText, sizeof(tempText), "%s C", str_temp);
  snprintf(humidityText, sizeof(humidityText), "%s", str_humidity);
  snprintf(voltageText, sizeof(voltageText), "%sV", str_voltage);

  uint16_t x, y, w, h, tw, th, hw, hh;
  int16_t xc, yc, xt, yt, xh, yh;

  display.setFont(&BebasNeueRegular60pt7b);
  display.getTextBounds(co2Text, 0, 0, &xc, &yc, &w, &h);
  x = (display.width() - w) / 2 - xc;
  y = 70 + h; 

  display.setFont(&BebasNeueRegular22pt7b);
  display.getTextBounds(tempText, 0, 0, &xt, &yt, &tw, &th);
  display.getTextBounds(humidityText, 0, 0, &xh, &yh, &hw, &hh);

  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

    display.setTextColor(GxEPD_BLACK);
    display.setFont(&BebasNeueRegular22pt7b);
    display.setCursor(0, th + 8);
    display.print(tempText);
    display.drawCircle(62,13,4,GxEPD_BLACK);
    display.drawCircle(62,13,3,GxEPD_BLACK);
    display.drawCircle(62,13,2,GxEPD_BLACK);

    display.setCursor(170 - hw, hh + 8);
    display.print(humidityText);
    display.drawInvertedBitmap(175, 9, humidityBitmap, 20, 30, GxEPD_BLACK);

    display.setFont(&BebasNeueRegular14pt7b);
    display.setCursor(145, 200);
    display.print(voltageText);

    if (co2 > 1400)
    {
      display.setTextColor(GxEPD_WHITE);
      display.fillRect(0, 60, 200, 25 + h, GxEPD_BLACK);
    }

    display.setFont(&BebasNeueRegular60pt7b);
    display.setCursor(x, y);
    display.print(co2Text);
  }
  while (display.nextPage());
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

  // DHT
  dht.begin();

  // CO2 Power pin setup
  ledcSetup(CO2_POWER_PIN_CH, CO2_POWER_PIN_FREQ, CO2_POWER_PIN_CH_RES);
  ledcAttachPin(CO2_POWER_PIN, CO2_POWER_PIN_CH);
}

void deep_sleep(int seconds)
{
  esp_sleep_enable_timer_wakeup(seconds * 1000000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  Serial.println("Going to deep sleep for " + String(seconds) + " seconds"); 
  esp_deep_sleep_start();
}

void loop()
{
  int votagePinValue = analogRead(VOLTAGE_PIN);
  float voltage = votagePinValue * VOLTAGE_DIVIDER_CONSTANT;
  Serial.println("Voltage: " + String(voltage));

  if (voltage <= VOLTAGE_LOW)
  {
    draw_low_battery();
    deep_sleep(300);
  }

  delay(50);
  float humidity = get_humidity();
  delay(50);
  float temp = get_temperature();
  
  co2_power_on();
  // 10 second warmup
  delay(10000);
  int co2 = co2_get_measurement();
  co2_power_off();
  Serial.println("CO2 measurement: " + String(co2));

  draw_data(co2, temp, humidity, voltage);

  deep_sleep(30);
}
