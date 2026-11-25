// Fast software I2C for the SSD1306 OLED (SCL = PA1, SDA = PA2).
//
// You can ignore this file unless the display misbehaves.
//
// Why it exists: the I2C code that comes with the U8g2 library is generic. It
// calls a function for every single line change and delay, which costs about
// 26 microseconds per bit. A full screen then takes ~240 ms and you can watch
// the picture being drawn. This file talks to the GPIO registers directly,
// which is about 6 times faster. It works the same electrically: a line is
// either pulled low, or released so the pull-up makes it high.
//
// Build options (#define before this file is included, or as build flags):
//   -DI2C_DELAY_LOOPS=1     slow the bus down if the display shows garbage
//   -DOLED_OPEN_DRAIN=1     even faster, but needs external pull-up resistors
//                           on SCL and SDA (most OLED modules have them)
#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

#ifndef OLED_OPEN_DRAIN
#define OLED_OPEN_DRAIN 0
#endif

#ifndef I2C_DELAY_LOOPS
#if OLED_OPEN_DRAIN
#define I2C_DELAY_LOOPS 1   // open-drain edges are so fast that SCL needs some width
#else
#define I2C_DELAY_LOOPS 0
#endif
#endif

// Pins: both are on port A. The numbers are the bit positions (PA1, PA2).
static const uint8_t OLED_SCL_BIT = 1;
static const uint8_t OLED_SDA_BIT = 2;

// ---- the two things a line can do: be pulled low, or be released ------------
#if OLED_OPEN_DRAIN
static inline void lineLow(uint8_t bit)     { GPIOA->BCR = 1u << bit; }
static inline void lineRelease(uint8_t bit) { GPIOA->BSHR = 1u << bit; }
static inline void setupPins() {
  for (uint8_t bit = OLED_SCL_BIT; bit <= OLED_SDA_BIT; ++bit) {
    GPIOA->BSHR = 1u << bit;  // start released
    GPIOA->CFGLR = (GPIOA->CFGLR & ~(0xFu << (bit * 4))) | (0x5u << (bit * 4));  // open-drain output
  }
}
#else
static inline void lineLow(uint8_t bit) {
  GPIOA->BCR = 1u << bit;
  GPIOA->CFGLR = (GPIOA->CFGLR & ~(0xFu << (bit * 4))) | (0x1u << (bit * 4));  // push-pull output
}
static inline void lineRelease(uint8_t bit) {
  GPIOA->CFGLR = (GPIOA->CFGLR & ~(0xFu << (bit * 4))) | (0x8u << (bit * 4));  // input with pull-up
  GPIOA->BSHR = 1u << bit;
}
static inline void setupPins() {}
#endif

static inline void shortPause() {
#if I2C_DELAY_LOOPS > 0
  for (volatile uint8_t i = 0; i < I2C_DELAY_LOOPS; ++i) {}
#endif
}

// SDA changes a lot. Remember its level so we skip register writes that would
// not change anything.
static bool sdaIsHigh = true;
static inline void setSda(bool high) {
  if (high == sdaIsHigh) return;
  sdaIsHigh = high;
  if (high) lineRelease(OLED_SDA_BIT); else lineLow(OLED_SDA_BIT);
}

// Send one byte, most significant bit first, then one extra clock for the
// acknowledge bit (we ignore the display's answer).
static void i2cWriteByte(uint8_t value) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    setSda(value & mask);
    lineRelease(OLED_SCL_BIT);
    shortPause();
    lineLow(OLED_SCL_BIT);
  }
  setSda(true);
  lineRelease(OLED_SCL_BIT);
  shortPause();
  lineLow(OLED_SCL_BIT);
}

// U8g2 calls this to start a transfer, send bytes and end a transfer.
static bool i2cStarted = false;
static uint8_t fastI2cByte(u8x8_t *u8x8, uint8_t message, uint8_t count, void *data) {
  switch (message) {
    case U8X8_MSG_BYTE_INIT:
      lineRelease(OLED_SCL_BIT);
      lineRelease(OLED_SDA_BIT);
      sdaIsHigh = true;
      i2cStarted = false;
      break;

    case U8X8_MSG_BYTE_SET_DC:
      break;

    case U8X8_MSG_BYTE_START_TRANSFER:
      if (i2cStarted) {          // a "repeated start": get both lines high first
        setSda(true);
        lineRelease(OLED_SCL_BIT);
        shortPause();
      }
      setSda(true);
      setSda(false);             // START: SDA falls while SCL is high
      shortPause();
      lineLow(OLED_SCL_BIT);
      i2cStarted = true;
      i2cWriteByte(u8x8_GetI2CAddress(u8x8));
      break;

    case U8X8_MSG_BYTE_SEND: {
      const uint8_t *bytes = (const uint8_t *)data;
      while (count--) i2cWriteByte(*bytes++);
      break;
    }

    case U8X8_MSG_BYTE_END_TRANSFER:
      setSda(false);
      shortPause();
      lineRelease(OLED_SCL_BIT);
      shortPause();
      setSda(true);              // STOP: SDA rises while SCL is high
      i2cStarted = false;
      break;

    default:
      return 0;
  }
  return 1;
}

// U8g2 also needs a function for pin setup and the few delays used while the
// display is being initialised.
static uint8_t fastI2cPinsAndDelays(u8x8_t *, uint8_t message, uint8_t arg, void *) {
  switch (message) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      pinMode(PA1, INPUT_PULLUP);   // also switches on the port A clock
      pinMode(PA2, INPUT_PULLUP);
      setupPins();
      lineRelease(OLED_SCL_BIT);
      lineRelease(OLED_SDA_BIT);
      break;
    case U8X8_MSG_DELAY_NANO:
    case U8X8_MSG_DELAY_100NANO:
    case U8X8_MSG_DELAY_I2C:
      break;
    case U8X8_MSG_DELAY_10MICRO: delayMicroseconds(10u * arg); break;
    case U8X8_MSG_DELAY_MILLI:   delay(arg); break;
    default: return 0;
  }
  return 1;
}

// Call once, before display.begin(), to switch the display to the fast driver.
static void useFastI2c(U8G2 &display) {
  display.getU8x8()->gpio_and_delay_cb = fastI2cPinsAndDelays;
  display.getU8x8()->byte_cb = fastI2cByte;
}
