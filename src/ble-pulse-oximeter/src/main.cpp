#include <Arduino.h>

#include <TFT_eSPI.h>

#include <OneButton.h>
#include <Wire.h>

#include "pcf8563.h"
#include "MPU9250.h"
#include "MAX30105.h"

#include "image.h"

#define LED_PIN             33
#define VBUS_PIN            37
#define CHARGE_PIN          32
#define TOUCH_POWER_PIN     25
#define BATTERY_PIN         35

#define RTC_PIN             34

#define BUTTON_PIN          38

#define HEART_RATE_SDA_PIN  15
#define HEART_RATE_SCL_PIN  13

#define  MPU_INT            4
#define  MPU_I2C_SDA_PIN    21
#define  MPU_I2C_SCL_PIN    22

TFT_eSPI tft = TFT_eSPI();
bool range_error = 0;

OneButton button(BUTTON_PIN, true);

PCF8563_Class rtc;
bool rtcIrq = false;
uint8_t hh, mm, ss;

MPU9250 mpu;
bool freefallDetected = false;
int freefallBlinkCount = 0;

MAX30105 max30102;
bool found_max30102_device = false;

void checkMpuSettings(void);

void clickButton(void) {
  tft.setTextColor(TFT_BLUE); // Red 
  tft.println("555");
}

void doubleClickButton(void) {
  tft.setTextColor(TFT_RED);  // Blue
  tft.println("555");
}

bool initializeRTC(void) {
  tft.setTextColor(TFT_RED);  // Blue
  tft.println("Checking PCF8563...");

  Wire.beginTransmission(PCF8563_SLAVE_ADDRESS);

  if (Wire.endTransmission() != 0) {
    return false;
  }

  rtc.begin(Wire);

  pinMode(RTC_PIN, INPUT_PULLUP);

  attachInterrupt(RTC_PIN, [] {
    rtcIrq = true;
  }, FALLING);

  rtc.disableAlarm();
  rtc.setDateTime(2019, 4, 7, 9, 5, 58);
  rtc.setAlarmByMinutes(6);
  rtc.enableAlarm();

  while (true)
  {
    Serial.println(rtc.formatDateTime());

    if (rtcIrq) {
      rtcIrq = false;
      detachInterrupt(RTC_PIN);
      rtc.resetAlarm();
      break;
    }
    delay(500);
  }

  // Check if the RTC clock matches, if not, use compile time
  rtc.check();
  
  RTC_Date datetime = rtc.getDateTime();
  hh = datetime.hour;
  mm = datetime.minute;
  ss = datetime.second;

  return true;
}

bool initializeMPU(void) {
  tft.setTextColor(TFT_RED);  // Blue
  tft.println("Checking MPU9250...");

  if (!mpu.begin(MPU9250_SCALE_2000DPS, MPU9250_RANGE_16G)) {
    return false;
  }

  mpu.calibrateGyro();
  mpu.setThreshold(3);
  mpu.setAccelPowerOnDelay(MPU9250_DELAY_3MS);
  mpu.setIntFreeFallEnabled(true);
  mpu.setIntZeroMotionEnabled(true);
  mpu.setIntMotionEnabled(true);
  mpu.setDHPFMode(MPU9250_DHPF_5HZ);
  mpu.setFreeFallDetectionThreshold(17);
  mpu.setFreeFallDetectionDuration(2);

  checkMpuSettings();

  pinMode(MPU_INT, INPUT);
  attachInterrupt(MPU_INT, [] {
    freefallBlinkCount = 0;
    freefallDetected = true;
  }, CHANGE);

  return true;
}

void checkMpuSettings(void) {
  Serial.println();

  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:          ");
  switch (mpu.getClockSource()) {
    case MPU9250_CLOCK_KEEP_RESET: Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU9250_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU9250_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU9250_CLOCK_PLL_ZGYRO: Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU9250_CLOCK_PLL_YGYRO: Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU9250_CLOCK_PLL_XGYRO: Serial.println("PLL with X axis gyroscope reference"); break;
  case MPU9250_CLOCK_INTERNAL_8MHZ: Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Accelerometer:         ");
  switch (mpu.getRange()) {
    case MPU9250_RANGE_16G: Serial.println("+/- 16 g"); break;
    case MPU9250_RANGE_8G: Serial.println("+/- 8 g"); break;
    case MPU9250_RANGE_4G: Serial.println("+/- 4 g"); break;
    case MPU9250_RANGE_2G: Serial.println("+/- 2 g"); break;
  }

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

  Serial.println();
}

bool initializeMAX(void) {
  tft.fillScreen(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_RED);  // Blue
  tft.println("Checking MAX30102...");

  if (!max30102.begin(Wire1, 400000)) {
    return false;
  }
  
  max30102.setup();
  max30102.setPulseAmplitudeRed(0x0A);
  max30102.setPulseAmplitudeGreen(0);

  found_max30102_device = true;

  return true;
}

void drawTopBar(void) {
  
}

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
{
    if (percentage == 0) {
        tft.fillRoundRect(x0, y0, w, h, 3, TFT_BLACK);
    }
    uint8_t margin = 2;
    uint16_t barHeight = h - 2 * margin;
    uint16_t barWidth = w - 2 * margin;
    tft.drawRoundRect(x0, y0, w, h, 3, frameColor);
    tft.fillRect(x0 + margin, y0 + margin, barWidth * percentage / 100.0, barHeight, barColor);
}

void clearScreen(void) {
  tft.fillScreen(TFT_WHITE);
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Initialize ST7735 0.96 LCD TFT
  tft.init();
  tft.setRotation(3);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_WHITE);
  tft.setTextFont(2);
  tft.setCursor(0, 0);

  // Initialize Button
  button.attachClick(clickButton);
  button.attachDoubleClick(doubleClickButton);

  pinMode(LED_PIN, OUTPUT);

  // Initialize MPU9250 (9-Axis) I2C
  Wire.begin(MPU_I2C_SDA_PIN, MPU_I2C_SCL_PIN);

  // Initialize MAX30102 I2C
  Wire1.begin(HEART_RATE_SDA_PIN, HEART_RATE_SCL_PIN);

  if (!initializeRTC()) {
    // tft.fillScreen(TFT_WHITE);
    // tft.setCursor(0, 0);
    tft.setTextColor(TFT_BLUE);
    tft.println("Check PCF8563 Failed");
  } else {
    // tft.fillScreen(TFT_WHITE);
    // tft.setCursor(0, 0);
    tft.setTextColor(TFT_GREEN);
    tft.println("Check PCF8563 Passed");
  }

  delay(500);

  if (!initializeMPU()) {
    // tft.fillScreen(TFT_WHITE);
    // tft.setCursor(0, 0);
    tft.setTextColor(TFT_BLUE);
    tft.println("Check MPU9250 Failed");
  } else {
    // tft.fillScreen(TFT_WHITE);
    // tft.setCursor(0, 0);
    tft.setTextColor(TFT_GREEN);
    tft.println("Check MPU9250 Passed");
  }

  delay(500);

  if (!initializeMAX()) {
    tft.setTextColor(TFT_BLUE);
    tft.println("Check MAX30102 Failed");
  } else {
    tft.setTextColor(TFT_GREEN);
    tft.println("Check MAX30102 Passed");
  }

  delay(1000);

  clearScreen();
  
  drawProgressBar(10, 30, 120, 15, 50, TFT_WHITE, TFT_BLUE);

  setCpuFrequencyMhz(80);
}

void loop() {
  button.tick();
}