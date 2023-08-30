/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

/*
    Please note the long strings of data sent mean the *RTS* pin is
    required with UART to slow down data sent to the Bluefruit LE!
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

const int PULSE_INPUT = A1;
const int PULSE_BLINK = LED_BUILTIN;
const int PULSE_FADE = 5;
const int THRESHOLD = 550;

PulseSensorPlayground pulseSensor;

// Create the bluefruit object, either software serial...uncomment these lines
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */

int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

// Define constants for the TFT display
// For the breakout board, you can use any 2 or 3 pins.
// These pins will also work for the 1.8" TFT shield.
#define TFT_CS        10
#define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         8

// Create an instance of the Adafruit ST7789 display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// 每个水滴的结构体
struct Drop {
  int x;
  int y;
  int r;
  int v;
};

// 水滴的数量
const int NUM_DROPS = 10;

// 水滴的数组
Drop drops[NUM_DROPS];

void setup(void)
{
  while (!Serial); // required for Flora & Micro
  delay(500);

  boolean success;

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Heart Rate Monitor (HRM) Example"));
  Serial.println(F("---------------------------------------------------"));

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ) {
    error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit HRM': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Bluefruit HRM")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    error(F("Could not add HRM service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
  if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
  if (! success) {
    error(F("Could not add BSL characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();

  //Serial.begin(115200);
  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);
  pulseSensor.setThreshold(THRESHOLD);

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our particular Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try PulseSensor_BPM_Alternative.ino,
       which doesn't use interrupts.
    */
    for (;;) {
      // Flash the led to show things didn't work.
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }

  tft.init(240, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // 初始化每个水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    drops[i].x = random(tft.width());
    drops[i].y = -random(tft.height());
    drops[i].r = random(5, 15);
    drops[i].v = random(5, 15);
  }

  waterdrops(10000);

  flower(10000);

  ghost(10000);

  leaf(10000);

}

void  waterdrops(unsigned long runtime) {
  unsigned long endtime = millis() + runtime;
  tft.fillScreen(ST77XX_BLACK);
  // 不停地模拟每个水滴的下落
  // while (millis() < endtime) {
  // 绘制每个水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    tft.fillCircle(drops[i].x, drops[i].y, drops[i].r, ST77XX_BLUE);
    tft.fillTriangle(drops[i].x - drops[i].r, drops[i].y - drops[i].r / 2, drops[i].x, drops[i].y - drops[i].r * 2, drops[i].x + drops[i].r, drops[i].y - drops[i].r / 2, ST77XX_BLUE);
  }

  // 延迟一段时间
  delay(10);

  // 抹去上一个位置的水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    tft.fillCircle(drops[i].x, drops[i].y, drops[i].r, ST77XX_BLACK);
    tft.fillTriangle(drops[i].x - drops[i].r, drops[i].y - drops[i].r / 2, drops[i].x, drops[i].y - drops[i].r * 2, drops[i].x + drops[i].r, drops[i].y - drops[i].r / 2, ST77XX_BLACK);
  }

  // 计算每个水滴的下一个位置
  for (int i = 0; i < NUM_DROPS; i++) {
    drops[i].y += drops[i].v;

    // 如果水滴到达了屏幕底部，将其重置到顶部
    if (drops[i].y > tft.height() + drops[i].r) {
      drops[i].y = -drops[i].r;
      drops[i].x = random(tft.width());
      drops[i].r = random(5, 15);
      drops[i].v = random(5, 15);
    }
  }
  // }
}

void flower(unsigned long runtime) {
  unsigned long endtime = millis() + runtime;
  tft.fillScreen(ST77XX_BLACK);
  // 不停地模拟每个水滴的下落
  // while (millis() < endtime) {
  // 绘制每个水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    // 画花瓣
    tft.fillCircle(drops[i].x - drops[i].r, drops[i].y, drops[i].r, ST77XX_YELLOW);
    tft.fillCircle(drops[i].x + drops[i].r, drops[i].y, drops[i].r, ST77XX_YELLOW);
    tft.fillCircle(drops[i].x, drops[i].y - drops[i].r, drops[i].r, ST77XX_YELLOW);
    tft.fillCircle(drops[i].x, drops[i].y + drops[i].r, drops[i].r, ST77XX_YELLOW);

    // 画花心
    tft.fillCircle(drops[i].x, drops[i].y, drops[i].r / 2, ST77XX_RED);
  }

  // 延迟一段时间
  delay(10);

  // 抹去上一个位置的水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    // 画花瓣
    tft.fillCircle(drops[i].x - drops[i].r, drops[i].y, drops[i].r, ST77XX_BLACK);
    tft.fillCircle(drops[i].x + drops[i].r, drops[i].y, drops[i].r, ST77XX_BLACK);
    tft.fillCircle(drops[i].x, drops[i].y - drops[i].r, drops[i].r, ST77XX_BLACK);
    tft.fillCircle(drops[i].x, drops[i].y + drops[i].r, drops[i].r, ST77XX_BLACK);

    // 画花心
    tft.fillCircle(drops[i].x, drops[i].y, drops[i].r / 2, ST77XX_BLACK);
  }

  // 计算每个水滴的下一个位置
  for (int i = 0; i < NUM_DROPS; i++) {
    drops[i].y += drops[i].v;

    // 如果水滴到达了屏幕底部，将其重置到顶部
    if (drops[i].y > tft.height() + drops[i].r) {
      drops[i].y = -drops[i].r;
      drops[i].x = random(tft.width());
      drops[i].r = random(5, 15);
      drops[i].v = random(5, 15);
    }
  }
  // }
}

void ghost(unsigned long runtime) {
  unsigned long endtime = millis() + runtime;
  tft.fillScreen(ST77XX_BLACK);
  // 不停地模拟每个水滴的下落
  // while (millis() < endtime) {
  // 绘制每个水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    tft.fillCircle(drops[i].x, drops[i].y, drops[i].r, ST77XX_RED);
    tft.fillTriangle(drops[i].x - drops[i].r, drops[i].y + drops[i].r * 3, drops[i].x, drops[i].y, drops[i].x + drops[i].r, drops[i].y + drops[i].r * 3, ST77XX_RED);
    tft.fillCircle(drops[i].x - drops[i].r, drops[i].y + drops[i].r * 1.5, drops[i].r / 2, ST77XX_RED);
    tft.fillCircle(drops[i].x + drops[i].r, drops[i].y + drops[i].r * 1.5, drops[i].r / 2, ST77XX_RED);
    tft.fillCircle(drops[i].x - drops[i].r / 2, drops[i].y + drops[i].r * 3, drops[i].r / 2, ST77XX_RED);
    tft.fillCircle(drops[i].x + drops[i].r / 2, drops[i].y + drops[i].r * 3, drops[i].r / 2, ST77XX_RED);
    tft.fillCircle(drops[i].x - drops[i].r / 2, drops[i].y, drops[i].r / 2, ST77XX_WHITE);
    tft.fillCircle(drops[i].x + drops[i].r / 2, drops[i].y, drops[i].r / 2, ST77XX_WHITE);
  }

  // 延迟一段时间
  delay(10);

  // 抹去上一个位置的水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    tft.fillCircle(drops[i].x, drops[i].y, drops[i].r, ST77XX_BLACK);
    tft.fillTriangle(drops[i].x - drops[i].r, drops[i].y + drops[i].r * 3, drops[i].x, drops[i].y, drops[i].x + drops[i].r, drops[i].y + drops[i].r * 3, ST77XX_BLACK);
    tft.fillCircle(drops[i].x - drops[i].r, drops[i].y + drops[i].r * 1.5, drops[i].r / 2, ST77XX_BLACK);
    tft.fillCircle(drops[i].x + drops[i].r, drops[i].y + drops[i].r * 1.5, drops[i].r / 2, ST77XX_BLACK);
    tft.fillCircle(drops[i].x - drops[i].r / 2, drops[i].y + drops[i].r * 3, drops[i].r / 2, ST77XX_BLACK);
    tft.fillCircle(drops[i].x + drops[i].r / 2, drops[i].y + drops[i].r * 3, drops[i].r / 2, ST77XX_BLACK);
    tft.fillCircle(drops[i].x - drops[i].r / 2, drops[i].y, drops[i].r / 2, ST77XX_BLACK);
    tft.fillCircle(drops[i].x + drops[i].r / 2, drops[i].y, drops[i].r / 2, ST77XX_BLACK);
  }

  // 计算每个水滴的下一个位置
  for (int i = 0; i < NUM_DROPS; i++) {
    drops[i].y += drops[i].v;

    // 如果水滴到达了屏幕底部，将其重置到顶部
    if (drops[i].y > tft.height() + drops[i].r) {
      drops[i].y = -drops[i].r;
      drops[i].x = random(tft.width());
      drops[i].r = random(5, 15);
      drops[i].v = random(5, 15);
    }
  }
}

void leaf(unsigned long runtime) {
  unsigned long endtime = millis() + runtime;
  tft.fillScreen(ST77XX_BLACK);
  // 不停地模拟每个水滴的下落
  // while (millis() < endtime) {
  // 绘制每个水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    tft.fillCircle(drops[i].x, drops[i].y, drops[i].r, ST77XX_GREEN);
    tft.fillTriangle(drops[i].x - drops[i].r, drops[i].y - drops[i].r / 2, drops[i].x, drops[i].y - drops[i].r * 2, drops[i].x + drops[i].r, drops[i].y - drops[i].r / 2, ST77XX_GREEN);
    tft.fillTriangle(drops[i].x - drops[i].r, drops[i].y + drops[i].r / 2, drops[i].x, drops[i].y + drops[i].r * 2, drops[i].x + drops[i].r, drops[i].y + drops[i].r / 2, ST77XX_GREEN);
  }

  // 延迟一段时间
  delay(0);

  // 抹去上一个位置的水滴
  for (int i = 0; i < NUM_DROPS; i++) {
    tft.fillCircle(drops[i].x, drops[i].y, drops[i].r, ST77XX_BLACK);
    tft.fillTriangle(drops[i].x - drops[i].r, drops[i].y - drops[i].r / 2, drops[i].x, drops[i].y - drops[i].r * 2, drops[i].x + drops[i].r, drops[i].y - drops[i].r / 2, ST77XX_BLACK);
    tft.fillTriangle(drops[i].x - drops[i].r, drops[i].y + drops[i].r / 2, drops[i].x, drops[i].y + drops[i].r * 2, drops[i].x + drops[i].r, drops[i].y + drops[i].r / 2, ST77XX_BLACK);
  }

  // 计算每个水滴的下一个位置
  for (int i = 0; i < NUM_DROPS; i++) {
    drops[i].y += drops[i].v;

    // 如果水滴到达了屏幕底部，将其重置到顶部
    if (drops[i].y > tft.height() + drops[i].r) {
      drops[i].y = -drops[i].r;
      drops[i].x = random(tft.width());
      drops[i].r = random(5, 15);
      drops[i].v = random(5, 15);
    }
  }
  // }
}

/** Send randomized heart rate data continuously **/
void loop(void) {

  /*
       Wait a bit.
       We don't output every sample, because our baud rate
       won't support that much I/O.
  */
  delay(20);

  // write the latest sample to Serial.
  //pulseSensor.outputSample();

  /*
     If a beat has happened since we last checked,
     write the per-beat information to Serial.
  */
  if (pulseSensor.sawStartOfBeat()) {
    //pulseSensor.outputBeat();

    int heart_rate = pulseSensor.getBeatsPerMinute();

    Serial.print(F("Updating HRM value to "));
    Serial.print(heart_rate);
    Serial.println(F(" BPM"));

    /* Command is sent when \n (\r) or println is called */
    /* AT+GATTCHAR=CharacteristicID,value */
    ble.print( F("AT+GATTCHAR=") );
    ble.print( hrmMeasureCharId );
    ble.print( F(",00-") );
    ble.println(heart_rate, HEX);

    /* Check if command executed OK */
    if ( !ble.waitForOK() )
    {
      Serial.println(F("Failed to get response!"));
    }

    if (128 >= heart_rate && heart_rate >= 110) {
      waterdrops(10000);
    } else if (95 >= heart_rate) {
      flower(10000);
    } else if (heart_rate >= 129) {
      ghost(10000);
    } else if (109 >= heart_rate && heart_rate >= 96) {
      leaf(10000);
    }

  }

}
