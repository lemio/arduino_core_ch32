// Remembers the last choice (which option and which PPS voltage) in flash, so
// the next power-up starts where you left off.
//
// The choice lives in the very last 256-byte page of the 62 KB code flash. The
// firmware is about half that size, so the page is free. Re-flashing new code
// does not touch it. A flash page can only be erased ~10,000 times, so the
// sketch saves only after you stop pressing the button, and only if the value
// actually changed.
#pragma once
#include <Arduino.h>
#include <string.h>

static const uint32_t SAVED_CHOICE_ADDRESS = 0x08000000UL + 62UL * 1024 - 256;

struct SavedChoice {
  uint16_t magic;       // marks "something valid was saved here"
  uint8_t  isPps;       // 0 = fixed voltage option, 1 = PPS option
  uint8_t  checksum;    // catches a half-written or corrupted record
  uint16_t voltageMv;   // fixed voltage, or the PPS voltage when isPps == 1
  uint16_t ppsMv;       // last PPS voltage, remembered even if a fixed option is selected
  uint32_t unused[3];   // pads the record to 20 bytes (whole 4-byte flash words)
};

static const uint16_t SAVED_CHOICE_MAGIC = 0x5D70;

static uint8_t checksumOf(const SavedChoice &c) {
  return (uint8_t)(c.isPps ^ c.voltageMv ^ (c.voltageMv >> 8) ^ c.ppsMv ^ (c.ppsMv >> 8) ^ 0xA5);
}

// Returns false if nothing valid has been saved yet.
static bool loadSavedChoice(SavedChoice &c) {
  memcpy(&c, (const void *)SAVED_CHOICE_ADDRESS, sizeof(c));
  return c.magic == SAVED_CHOICE_MAGIC && c.checksum == checksumOf(c) && c.isPps < 2;
}

// Writes the choice to flash and reads it back to check. Returns false if the
// flash does not contain what we wrote (the caller can then try again later).
static bool storeSavedChoice(SavedChoice &c) {
  c.magic = SAVED_CHOICE_MAGIC;
  c.checksum = checksumOf(c);

  // Flash is written a whole page at a time: build the page in RAM first.
  uint32_t page[64];                       // 64 words = 256 bytes
  memset(page, 0xFF, sizeof(page));        // 0xFF is the "erased" value
  memcpy(page, &c, sizeof(c));

  for (int attempt = 0; attempt < 3; ++attempt) {
    FLASH_Unlock_Fast();
    FLASH_ErasePage_Fast(SAVED_CHOICE_ADDRESS);
    FLASH_BufReset();
    for (int i = 0; i < 64; ++i) FLASH_BufLoad(SAVED_CHOICE_ADDRESS + 4 * i, page[i]);
    FLASH_ProgramPage_Fast(SAVED_CHOICE_ADDRESS);
    FLASH_Lock_Fast();

    if (memcmp((const void *)SAVED_CHOICE_ADDRESS, page, sizeof(page)) == 0) return true;
  }
  return false;
}

static bool sameChoice(const SavedChoice &a, const SavedChoice &b) {
  return a.isPps == b.isPps && a.voltageMv == b.voltageMv && a.ppsMv == b.ppsMv;
}
