// =============================================================================
//  USB-C power supply controller: pick a voltage with one button.
//
//  The CH32X035 asks a USB-C (USB Power Delivery) charger for a voltage and
//  shows what the charger offers on a small OLED. A single button chooses.
//
//  Needs the "U8g2" library (Library Manager, by oliver).
//
//  Wiring
//    Button        PA3 -> GND            (uses the internal pull-up)
//    OLED SSD1306  SCL = PA1, SDA = PA2  (I2C address 0x78)
//    PC14 / PC15 are the USB-C CC lines and must not be used for anything else.
//
//  What you see
//    Top line   the voltage the charger is really giving + its status.
//               Loading dots appear while it is negotiating.
//    Below      one row per option the charger offers. The option you asked
//               for is highlighted (inverted). For a PPS option the highlight
//               is a bar: the longer the bar, the higher the voltage.
//
//  The button (it reacts the moment you press, the screen updates instantly)
//    Fixed option  (5 V, 9 V, ...)  press = go to the next option.
//    PPS option    (adjustable)     press = next level: the minimum (e.g.
//                  3.3 V), then 4 V, 5 V, 6 V ... up to the maximum. The press
//                  after the last level moves on to the next option.
//
//  The charger is only asked for the new voltage 0.4 s after your last press,
//  so you can click through several options without it renegotiating each time.
//
//  The last choice is stored in flash and restored at the next power-up if that
//  charger offers it; otherwise it starts at 5 V.
//
//  Optional settings (#define above the #includes, or as build flags):
//    -DBUTTON_PIN=PA4            use another pin for the button
//    -DMAX_REQUEST_CURRENT=2.0f  amps to ask for (default 1.0)
//    -DANIMATE_SELECTION=0       no sliding highlight animation
//    -DANIMATION_MS=250          length of the sliding animation
//    -DOLED_OPEN_DRAIN=1         faster display, needs external I2C pull-ups
//    -DI2C_DELAY_LOOPS=1         slower display bus if it shows garbage
// =============================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <USBPD.h>
#include <stdio.h>
#include <string.h>

#include "fast_i2c.h"                   // fast display bus (can be ignored)
#include "saved_choice.h"               // remembering the choice in flash


// =============================================================================
//  1. Settings
// =============================================================================

#ifndef BUTTON_PIN
#define BUTTON_PIN PA3
#endif

#ifndef MAX_REQUEST_CURRENT
#define MAX_REQUEST_CURRENT 1.0f        // amps. The charger may offer less.
#endif

#ifndef ANIMATE_SELECTION
#define ANIMATE_SELECTION 1             // 1 = highlight slides, 0 = it jumps
#endif

#ifndef ANIMATION_MS
#define ANIMATION_MS 160                // how long the slide takes
#endif

static const unsigned COMMIT_DELAY_MS      = 400;   // wait this long after the last press before asking the charger
static const unsigned PPS_UPDATE_MS        = 150;   // minimum time between two PPS requests
static const unsigned SAVE_DELAY_MS        = 1500;  // save to flash this long after the last change
static const unsigned NO_CHARGER_GRACE_MS  = 1500;  // a charger must be missing this long before we forget it
static const unsigned DOT_STEP_MS          = 300;   // speed of the loading dots
static const unsigned PPS_BUTTON_STEP_MV   = 1000;  // PPS goes up in whole volts
static const unsigned DEFAULT_VOLTAGE_MV   = 5000;  // used when nothing usable was saved

// Screen layout: the 128x64 screen is 8 text lines of 8 pixels.
static const uint8_t  OLED_I2C_ADDRESS = 0x78;
static const uint8_t  ROW_HEIGHT       = 8;
static const uint8_t  FIRST_ROW_TOP    = 8;     // line 0 is the top bar
static const uint8_t  MAX_ROWS         = 7;     // options that fit below the top bar


// =============================================================================
//  2. State (everything the program remembers)
// =============================================================================

U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, PA1, PA2, U8X8_PIN_NONE);

// What the user has chosen
static uint8_t  selectedIndex = 0;              // which option (0 = first row)
static unsigned ppsRequestMv  = DEFAULT_VOLTAGE_MV;  // chosen PPS voltage in millivolts

// Talking to the charger
static bool          chargerSeen       = false;  // true after capabilities arrived
static unsigned long noChargerSince    = 0;      // when capabilities disappeared (0 = present)
static bool          needRequest       = true;   // a new choice must be sent to the charger
static uint32_t      lastRequestKey    = 0;      // identifies what we asked for last (0 = nothing)
static unsigned long lastRequestTime   = 0;

// Waiting for the user to finish pressing
static bool          waitingForMorePresses = false;
static unsigned long lastPressTime         = 0;

// Saving to flash
static bool          choiceNeedsSaving = false;
static unsigned long lastChoiceChange  = 0;


// =============================================================================
//  3. Small helpers
// =============================================================================

// The library gives volts as floats; we calculate in whole millivolts.
static unsigned voltsToMv(float volts) { return (unsigned)(volts * 1000.0f + 0.5f); }

// Keep a voltage inside the range an option can deliver.
static unsigned clampToRange(const USBPDCapability &option, unsigned mv) {
  const unsigned lowest  = voltsToMv(option.minVoltage);
  const unsigned highest = voltsToMv(option.maxVoltage);
  if (mv < lowest)  return lowest;
  if (mv > highest) return highest;
  return mv;
}

// How many amps to ask for: our limit, or what the charger offers if less.
static float currentToRequest(const USBPDCapability &option) {
  return option.current < MAX_REQUEST_CURRENT ? option.current : MAX_REQUEST_CURRENT;
}

// The next PPS level above `currentMv`: the minimum, then whole volts, then the
// maximum. Returns 0 when there is no higher level.
static unsigned nextPpsLevel(const USBPDCapability &option, unsigned currentMv) {
  const unsigned lowest  = voltsToMv(option.minVoltage);
  const unsigned highest = voltsToMv(option.maxVoltage);
  if (currentMv < lowest) return lowest;
  const unsigned nextWholeVolt = (currentMv / PPS_BUTTON_STEP_MV + 1) * PPS_BUTTON_STEP_MV;
  if (nextWholeVolt <= highest) return nextWholeVolt;
  return currentMv < highest ? highest : 0;
}


// =============================================================================
//  4. The button
//
//  The button is read by an interrupt, so even a very short tap is counted
//  while the main loop is busy drawing the screen. The interrupt only counts
//  presses; the main loop decides what a press means.
// =============================================================================

static volatile uint8_t       pressCount    = 0;      // presses not yet handled
static volatile bool          buttonIsDown  = false;
static volatile unsigned long lastPressAt   = 0;
static volatile unsigned long lastReleaseAt = 0;

// Runs on every edge of the button pin. It looks at the pin level (not at
// which edge it was), so a missed or noisy edge can never confuse it.
// Debounce: a press only counts if the previous press was over 30 ms ago and
// the button was released over 20 ms ago.
static void buttonInterrupt() {
  const unsigned long now = millis();

  if (digitalRead(BUTTON_PIN) != LOW) {          // released
    if (buttonIsDown) lastReleaseAt = now;
    buttonIsDown = false;
    return;
  }

  if (!buttonIsDown && now - lastPressAt >= 30 && now - lastReleaseAt >= 20) {
    lastPressAt = now;
    if (pressCount < 8) ++pressCount;
  }
  buttonIsDown = true;
}

// Returns how many presses happened since the last call.
static uint8_t takeButtonPresses() {
  // Safety net: if a release was missed, notice it here.
  if (buttonIsDown && digitalRead(BUTTON_PIN) != LOW) buttonIsDown = false;

  noInterrupts();                 // read and reset the counter without being interrupted
  const uint8_t presses = pressCount;
  pressCount = 0;
  interrupts();
  return presses;
}


// =============================================================================
//  5. Choosing, requesting and remembering
// =============================================================================

// Called when a charger connects: choose the option that was used last time,
// or 5 V if this charger does not offer it.
static void restoreSavedSelection(uint8_t optionCount) {
  SavedChoice saved;
  const bool haveSaved = loadSavedChoice(saved);
  USBPDCapability option;
  int pick = -1;
  unsigned ppsMv = haveSaved ? saved.ppsMv : DEFAULT_VOLTAGE_MV;

  if (haveSaved && saved.isPps) {
    // Find a PPS option whose range contains the saved voltage.
    for (uint8_t i = 0; i < optionCount && pick < 0; ++i) {
      if (USBPD.getCapability(i, option) && option.programmable &&
          saved.voltageMv >= voltsToMv(option.minVoltage) &&
          saved.voltageMv <= voltsToMv(option.maxVoltage)) {
        pick = i;
        ppsMv = saved.voltageMv;
      }
    }
  } else if (haveSaved) {
    // Find the fixed option with the saved voltage.
    for (uint8_t i = 0; i < optionCount && pick < 0; ++i) {
      if (USBPD.getCapability(i, option) && !option.programmable &&
          voltsToMv(option.maxVoltage) == saved.voltageMv)
        pick = i;
    }
  }

  if (pick < 0) {
    // Nothing usable saved: use the fixed 5 V option.
    for (uint8_t i = 0; i < optionCount && pick < 0; ++i) {
      if (USBPD.getCapability(i, option) && !option.programmable &&
          voltsToMv(option.maxVoltage) == DEFAULT_VOLTAGE_MV)
        pick = i;
    }
  }
  if (pick < 0) pick = 0;

  selectedIndex = pick;
  ppsRequestMv = ppsMv;
  if (USBPD.getCapability(selectedIndex, option) && option.programmable)
    ppsRequestMv = clampToRange(option, ppsRequestMv);

  needRequest = true;
  choiceNeedsSaving = false;      // we just loaded it, nothing new to save
}

// Remember that the choice changed: it must be sent to the charger, and saved.
static void markChoiceChanged() {
  needRequest = true;
  choiceNeedsSaving = true;
  lastChoiceChange = millis();
}

// Ask the charger for the selected option (unless we already asked for it).
static void sendRequestToCharger(const USBPDCapability &option) {
  // A "key" identifies a request: fixed options by their voltage, PPS options by
  // their voltage with a marker bit. If the charger is already giving (or is
  // busy negotiating) exactly this, asking again would only renegotiate.
  const uint32_t key = option.programmable ? (0x10000u | ppsRequestMv)
                                           : voltsToMv(option.maxVoltage);
  const USBPDStatus status = USBPD.getStatus();
  needRequest = false;
  if (key == lastRequestKey && (status == USBPD_READY || status == USBPD_NEGOTIATING)) return;

  if (option.programmable)
    USBPD.requestProgrammableVoltage(selectedIndex, ppsRequestMv / 1000.0f, currentToRequest(option));
  else
    USBPD.requestCapability(selectedIndex, currentToRequest(option));

  lastRequestKey = key;
  lastRequestTime = millis();
}

// Save the choice to flash once it has been stable for a while. While a change
// has not been saved yet a small square is shown in the top right corner: do
// not unplug until it is gone.
static void saveChoiceWhenStable(const USBPDCapability &option) {
  if (!choiceNeedsSaving || millis() - lastChoiceChange < SAVE_DELAY_MS) return;
  if (USBPD.getStatus() != USBPD_READY) return;   // writing flash briefly stalls the CPU: not during negotiation

  SavedChoice now;
  memset(&now, 0xFF, sizeof(now));
  now.isPps     = option.programmable ? 1 : 0;
  now.voltageMv = option.programmable ? ppsRequestMv : voltsToMv(option.maxVoltage);
  now.ppsMv     = ppsRequestMv;

  SavedChoice before;
  if (loadSavedChoice(before) && sameChoice(before, now)) {   // unchanged: spare the flash
    choiceNeedsSaving = false;
    return;
  }

  if (storeSavedChoice(now)) choiceNeedsSaving = false;
  else lastChoiceChange = millis();                           // failed: try again in a while
}


// =============================================================================
//  6. The sliding highlight animation
//
//  The highlight (inverted box) is described by its top edge `y` and its
//  `width`. When the target changes we glide from the old values to the new
//  ones with a smooth "ease in, ease out" curve.
// =============================================================================

static bool animationHasStarted = false;   // false until we know where the highlight starts
static bool animationRunning    = false;   // true while frames must keep being drawn
static int  animFromY, animFromWidth, animToY, animToWidth;
static int  animCurrentY, animCurrentWidth;
static unsigned long animStartTime = 0;

// Smooth start and stop. Input and output are 0..1000.
static int easeInOut(int t) {
  return (int)((long)t * t / 1000 * (3000 - 2 * t) / 1000);
}

// Given where the highlight should end up, return where to draw it right now.
static void highlightPosition(int targetY, int targetWidth, int &drawY, int &drawWidth) {
#if ANIMATE_SELECTION
  const unsigned long now = millis();

  if (!animationHasStarted) {                      // first time: no animation, just appear
    animationHasStarted = true;
    animFromY = animToY = animCurrentY = targetY;
    animFromWidth = animToWidth = animCurrentWidth = targetWidth;
    animationRunning = false;
  } else if (targetY != animToY || targetWidth != animToWidth) {
    animFromY = animCurrentY;                      // new target: glide from where we are now
    animFromWidth = animCurrentWidth;
    animToY = targetY;
    animToWidth = targetWidth;
    animStartTime = now;
    animationRunning = true;
  }

  if (animationRunning) {
    const unsigned long elapsed = now - animStartTime;
    if (elapsed >= ANIMATION_MS) {
      animCurrentY = animToY;
      animCurrentWidth = animToWidth;
      animationRunning = false;                    // this is the last frame
    } else {
      const int progress = easeInOut((int)(elapsed * 1000 / ANIMATION_MS));
      animCurrentY     = animFromY     + (animToY     - animFromY)     * progress / 1000;
      animCurrentWidth = animFromWidth + (animToWidth - animFromWidth) * progress / 1000;
    }
  }
  drawY = animCurrentY;
  drawWidth = animCurrentWidth;
#else
  drawY = targetY;
  drawWidth = targetWidth;
#endif
}


// =============================================================================
//  7. Drawing the screen
// =============================================================================

// ---- Sending only what changed ----------------------------------------------
// Sending the whole 1 KB picture over I2C is slow. We keep a copy of what the
// OLED currently shows and send only the 8x8-pixel tiles that differ.

static uint8_t onScreen[128 * 8];       // copy of what the OLED shows
static bool    onScreenIsKnown = false;

static void sendChangesToScreen() {
  uint8_t *picture = display.getBufferPtr();

  if (!onScreenIsKnown) {               // first picture: send everything
    display.sendBuffer();
    memcpy(onScreen, picture, sizeof(onScreen));
    onScreenIsKnown = true;
    return;
  }

  // The screen is 8 "pages" (rows of 8 pixels) of 16 tiles (8 columns each).
  for (uint8_t page = 0; page < 8; ++page) {
    const uint8_t *newPage = picture + page * 128;
    const uint8_t *oldPage = onScreen + page * 128;
    int runStart = -1;                  // first tile of a run of changed tiles

    for (uint8_t tile = 0; tile <= 16; ++tile) {
      const bool changed = tile < 16 && memcmp(newPage + tile * 8, oldPage + tile * 8, 8) != 0;
      if (changed && runStart < 0) runStart = tile;
      if (!changed && runStart >= 0) {  // a run ended: send it in one go
        display.updateDisplayArea(runStart, page, tile - runStart, 1);
        runStart = -1;
      }
    }
    memcpy(onScreen + page * 128, newPage, 128);
  }
}

// ---- Text for the top bar ---------------------------------------------------

static const char *statusText() {
  switch (USBPD.getStatus()) {
    case USBPD_READY:               return "ready";
    case USBPD_NEGOTIATING:         return "negotiating";
    case USBPD_VOLTAGE_UNAVAILABLE: return "V unavailable";
    case USBPD_CURRENT_UNAVAILABLE: return "A unavailable";
    case USBPD_REJECTED:            return "rejected";
    case USBPD_ERROR:               return "error";
    case USBPD_NO_SOURCE:
    case USBPD_DISCONNECTED:        return "no source";
    default:                        return "idle";
  }
}

// Number of loading dots to show (0..3). They run while we wait for a charger
// or while PD is negotiating, and the count changes every DOT_STEP_MS.
static uint8_t loadingDots(bool chargerPresent) {
  const bool busy = !chargerPresent || USBPD.getStatus() == USBPD_NEGOTIATING;
  return busy ? (millis() / DOT_STEP_MS) % 4 : 0;
}

// ---- The three parts of the picture -----------------------------------------

static void drawTopBar(bool chargerPresent) {
  char text[32];
  const uint8_t dots = loadingDots(chargerPresent);   // "%.*s" prints that many of the "..." characters

  if (USBPD.isReady()) {
    const unsigned mv = voltsToMv(USBPD.getVoltage());
    snprintf(text, sizeof(text), "%u.%02uV %s%.*s", mv / 1000, (mv % 1000) / 10, statusText(), dots, "...");
  } else {
    snprintf(text, sizeof(text), "--.--V %s%.*s",
             chargerPresent ? statusText() : "waiting for caps", dots, "...");
  }
  display.drawStr(0, 7, text);
  if (choiceNeedsSaving) display.drawBox(123, 2, 4, 4);   // "not saved yet" marker
}

// The inverted box behind the selected row. For a PPS option its width shows
// the voltage within the option's range.
static void drawHighlight() {
  USBPDCapability option;
  if (selectedIndex >= MAX_ROWS || !USBPD.getCapability(selectedIndex, option)) return;

  int targetWidth = 128;                                  // fixed option: the whole row
  const unsigned lowest  = voltsToMv(option.minVoltage);
  const unsigned highest = voltsToMv(option.maxVoltage);
  if (option.programmable && highest > lowest) {
    const unsigned long inRange = clampToRange(option, ppsRequestMv) - lowest;
    targetWidth = 1 + inRange * 127 / (highest - lowest);
  }

  int y, width;
  highlightPosition(FIRST_ROW_TOP + selectedIndex * ROW_HEIGHT, targetWidth, y, width);
  display.setDrawColor(1);
  display.drawBox(0, y, width, ROW_HEIGHT);
}

// The text of every option. It is drawn in XOR mode, so where it overlaps the
// highlight it turns dark on light, and elsewhere it stays light on dark.
static void drawOptionRows(uint8_t optionCount) {
  for (uint8_t i = 0; i < optionCount && i < MAX_ROWS; ++i) {
    USBPDCapability option;
    char text[32];
    char selectedVoltage[8] = "";                         // shown at the right edge of the selected PPS row
    const int textBaseline = FIRST_ROW_TOP + i * ROW_HEIGHT + 7;

    if (!USBPD.getCapability(i, option)) {
      snprintf(text, sizeof(text), "%u: unsupported", i + 1);
    } else {
      const unsigned minMv = voltsToMv(option.minVoltage);
      const unsigned maxMv = voltsToMv(option.maxVoltage);
      const unsigned mA    = voltsToMv(option.current);   // amps -> milliamps

      if (option.programmable) {
        snprintf(text, sizeof(text), "%u:%u.%02u-%u.%02uV %u.%02uA", i + 1,
                 minMv / 1000, (minMv % 1000) / 10, maxMv / 1000, (maxMv % 1000) / 10,
                 mA / 1000, (mA % 1000) / 10);
        if (i == selectedIndex)
          snprintf(selectedVoltage, sizeof(selectedVoltage), "%u.%uV",
                   ppsRequestMv / 1000, (ppsRequestMv % 1000) / 100);
      } else {
        snprintf(text, sizeof(text), "%u: %u.%02uV %u.%02uA", i + 1,
                 maxMv / 1000, (maxMv % 1000) / 10, mA / 1000, (mA % 1000) / 10);
      }
    }

    display.setDrawColor(2);                              // XOR
    display.drawStr(1, textBaseline, text);
    if (selectedVoltage[0])
      display.drawStr(128 - 5 * strlen(selectedVoltage) - 1, textBaseline, selectedVoltage);
    display.setDrawColor(1);
  }
}

// Draw one complete picture and send the changes to the OLED.
static void drawScreen(bool chargerPresent) {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tf);
  display.setFontMode(1);                                 // transparent text background

  drawTopBar(chargerPresent);

  if (!chargerPresent) {
    animationHasStarted = false;                          // next time the highlight just appears
    animationRunning = false;
  } else {
    drawHighlight();                                      // first, so the text goes on top
    drawOptionRows(USBPD.getCapabilityCount());
  }
  sendChangesToScreen();
}

// ---- Only draw when something changed ---------------------------------------

static uint32_t lastScreenFingerprint = 0;
static bool     screenEverDrawn = false;

// Mix a number into a running checksum (FNV hash).
static uint32_t mixIn(uint32_t hash, uint32_t value) { return (hash ^ value) * 16777619u; }

// A number that changes whenever anything shown on screen changes.
static uint32_t screenFingerprint(bool chargerPresent) {
  uint32_t h = 2166136261u;
  h = mixIn(h, chargerPresent);
  h = mixIn(h, loadingDots(chargerPresent));
  h = mixIn(h, USBPD.getStatus());
  h = mixIn(h, USBPD.isReady() ? voltsToMv(USBPD.getVoltage()) : 0);
  if (chargerPresent) {
    h = mixIn(h, USBPD.getCapabilityCount());
    h = mixIn(h, selectedIndex);
    h = mixIn(h, ppsRequestMv);
    h = mixIn(h, choiceNeedsSaving);
  }
  return h;
}

// Redraw if (and only if) something changed or an animation is running.
static void refreshScreen(bool chargerPresent) {
  const uint32_t fingerprint = screenFingerprint(chargerPresent);
  if (screenEverDrawn && fingerprint == lastScreenFingerprint && !animationRunning) return;

  screenEverDrawn = true;
  lastScreenFingerprint = fingerprint;
  drawScreen(chargerPresent);
}


// =============================================================================
//  8. setup() and loop()
// =============================================================================

void setup() {
  // PB12 powers the OLED on this board.
  pinMode(PB12, OUTPUT);
  digitalWrite(PB12, HIGH);

  // Button: call buttonInterrupt() whenever the pin changes.
  attachInterrupt(BUTTON_PIN, GPIO_Mode_IPU, buttonInterrupt, EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  // Display.
  useFastI2c(display);
  display.setI2CAddress(OLED_I2C_ADDRESS);
  display.begin();
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(0, 7, "Initializing...");
  display.sendBuffer();

  // Ask for 5 V right away. A charger expects an answer very soon after it
  // announces what it offers, and drawing a screen takes too long to wait for.
  USBPD.requestVoltage(5, 0.5f);
  lastRequestKey = DEFAULT_VOLTAGE_MV;
}

// Handle every press since the last pass: move to the next PPS level, or to
// the next option. `option` is updated to the newly selected option.
static void applyButtonPresses(uint8_t presses, uint8_t optionCount, USBPDCapability &option) {
  for (uint8_t p = 0; p < presses; ++p) {
    const unsigned nextLevel = option.programmable
        ? nextPpsLevel(option, clampToRange(option, ppsRequestMv)) : 0;

    if (nextLevel) {
      ppsRequestMv = nextLevel;                           // same PPS option, next level
    } else {
      for (uint8_t tries = 0; tries < optionCount; ++tries) {   // go to the next usable option
        selectedIndex = (selectedIndex + 1) % optionCount;
        if (USBPD.getCapability(selectedIndex, option)) break;
      }
      if (option.programmable) ppsRequestMv = voltsToMv(option.minVoltage);  // start at its lowest level
    }
  }
}

// One pass of the main loop while a charger is connected.
static void runWithCharger() {
  noChargerSince = 0;
  const uint8_t optionCount = USBPD.getCapabilityCount();

  if (!chargerSeen) {                                     // a charger has just connected
    chargerSeen = true;
    restoreSavedSelection(optionCount);
  }

  USBPDCapability option;
  if (selectedIndex >= optionCount || !USBPD.getCapability(selectedIndex, option)) {
    selectedIndex = 0;
    USBPD.getCapability(0, option);
  }

  // 1. Button: change the choice and show it immediately. The screen shows what
  //    was asked for, not (yet) what the charger gives.
  const uint8_t presses = takeButtonPresses();
  if (presses) {
    applyButtonPresses(presses, optionCount, option);
    waitingForMorePresses = true;
    lastPressTime = millis();
    markChoiceChanged();
    refreshScreen(true);
  }

  // 2. Charger: ask for the choice, but only after the user stopped pressing,
  //    so clicking through options does not renegotiate each time.
  if (waitingForMorePresses && millis() - lastPressTime >= COMMIT_DELAY_MS)
    waitingForMorePresses = false;

  const bool ppsMayBeUpdated = USBPD.getStatus() != USBPD_NEGOTIATING &&
                               millis() - lastRequestTime >= PPS_UPDATE_MS;
  if (needRequest && !waitingForMorePresses && (!option.programmable || ppsMayBeUpdated))
    sendRequestToCharger(option);

  // 3. Screen. The request comes first: the charger expects it promptly, while
  //    drawing takes a while.
  refreshScreen(true);

  // 4. Remember the choice for next time.
  saveChoiceWhenStable(option);
}

// One pass of the main loop while no charger is connected.
static void runWithoutCharger() {
  // While the voltage changes the charger can disappear for a moment. Only
  // treat it as really gone after a grace period; otherwise the choice would
  // be thrown away by a short glitch.
  if (noChargerSince == 0) noChargerSince = millis() ? millis() : 1;
  if (millis() - noChargerSince < NO_CHARGER_GRACE_MS) return;

  if (chargerSeen) lastRequestKey = 0;                    // it was connected and is now gone
  chargerSeen = false;                                    // the next charger gets a fresh start
  takeButtonPresses();                                    // nothing to select: ignore presses
  refreshScreen(false);
}

void loop() {
  if (USBPD.hasCapabilities()) runWithCharger();
  else                         runWithoutCharger();
}
