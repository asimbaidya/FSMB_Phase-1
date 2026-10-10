/*
 * =====================================================================
 *  Voltage Protector - Arduino Uno (Wokwi simulation)
 * =====================================================================
 *  PURPOSE
 *    Watches the (simulated) mains voltage and load current. If the voltage
 *    is too high / too low, or the current is too high, the relay opens and
 *    the appliance is disconnected. It reconnects by itself (or on a button
 *    press) once the supply has been healthy again for a set time.
 *
 *  INPUTS
 *    Potentiometer A0 : simulated voltage (0..300 V RMS)
 *    Potentiometer A1 : simulated current (0..15 A RMS)
 *    MODE   button    : open the settings menu / next page; after the last
 *                       page the settings are saved and the run screen returns
 *    UP     button    : increase the value on the current page (hold = repeat)
 *    DOWN   button    : decrease the value on the current page (hold = repeat)
 *    POWER  button    : switch the load ON / OFF; in a fault it restarts the
 *                       reconnect (line healthy) or switches OFF (line unsafe,
 *                       which also silences the buzzer)
 *
 *  OUTPUTS
 *    Relay (D2)       : connects the load
 *    Green LED        : load connected
 *    Yellow LED       : fault pending (blinking-in-NORMAL) or reconnect countdown
 *    Red LED          : tripped
 *    Buzzer           : continuous tone while tripped
 *    16x2 I2C LCD     : live readings, state and the settings menu
 *    USB serial       : status and settings for a computer (see below)
 *
 *  COMPUTER LINK (USB serial, 115200 baud, one command per line)
 *    GET                                    live readings and state as JSON
 *    GETCFG                                 current settings as JSON
 *    SET <OV|UV|OC|TD|RC|AUTO> <value>      change one setting (validated, saved)
 *    HELP                                   list the commands
 *    Units: OV/UV in volts, OC in amps, TD in ms, RC in seconds, AUTO 1 or 0.
 *    SET is refused while the settings menu is open on the device.
 *
 *  SETTINGS (edited on the LCD or over serial, stored in EEPROM)
 *    Over-voltage limit, under-voltage limit, over-current limit,
 *    trip delay, reconnect delay, automatic / manual restart.
 *    Edits are made on a copy and applied only when the menu is left, so a
 *    limit change never trips the load while you are still editing.
 *
 *  KNOWN LIMITS
 *    - RMS over one 20 ms cycle cannot see sub-cycle spikes. A real product
 *      needs a hardware clamp (MOV) and/or peak detection.
 *    - Trip timing resolution is the 100 ms measurement interval.
 *
 *  USING REAL SENSORS
 *    Only measureRms() is simulation-specific. Replace the simulated sample
 *    line with `analogRead(pin)` plus `delayMicroseconds(SAMPLE_GAP_US)`
 *    and calibrate COUNTS_PER_VOLT / COUNTS_PER_AMP against a multimeter.
 * =====================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

// =====================================================================
// 1. PIN MAP
// =====================================================================
const uint8_t PIN_VOLT_SENSE = A0;
const uint8_t PIN_CURR_SENSE = A1;

const uint8_t PIN_RELAY      = 2;    // HIGH = load connected
const uint8_t PIN_BTN_MODE   = 3;
const uint8_t PIN_BTN_UP     = 4;
const uint8_t PIN_BTN_DOWN   = 5;
const uint8_t PIN_BTN_POWER  = 6;
const uint8_t PIN_BUZZER     = 7;
const uint8_t PIN_LED_GREEN  = 8;
const uint8_t PIN_LED_YELLOW = 9;
const uint8_t PIN_LED_RED    = 10;
// A4 = SDA and A5 = SCL belong to the I2C LCD.

// =====================================================================
// 2. CONSTANTS
// =====================================================================

// ---- Sensor scaling ----
const float VOLT_FULL_SCALE = 300.0;   // volts at 100 % pot position
const float CURR_FULL_SCALE = 15.0;    // amps  at 100 % pot position

// ADC counts (RMS) per volt / per amp.
//   ZMPT101B  : ~1.1 counts per volt when trimmed for 300 V
//   ACS712-20A: 100 mV/A / 4.883 mV per count = 20.48 counts per amp
const float COUNTS_PER_VOLT = 1.10;
const float COUNTS_PER_AMP  = 20.48;

const int ADC_MIDPOINT       = 512;    // AC signal is centred on VCC/2
const int SAMPLES_PER_WINDOW = 100;    // one 50 Hz cycle (20 ms)

// ---- Protection behaviour ----
const float HYSTERESIS_VOLTS     = 5.0;   // must be this far inside the limits to reconnect
const float HYSTERESIS_CURRENT   = 0.90;  // current must fall below 90 % of the limit
const float HARD_OVERLOAD_RATIO  = 2.0;   // current above 2x the limit trips instantly
const float HARD_OVERVOLT_MARGIN = 20.0;  // voltage this far above OV trips instantly
const float MIN_BAND_VOLTS       = 30.0;  // keeps UV and OV far enough apart to reconnect

// ---- Allowed setting ranges ----
const float    LIMIT_OV_MIN           = 200.0;
const float    LIMIT_OV_MAX           = 300.0;
const float    LIMIT_UV_MIN           = 100.0;
const float    LIMIT_OC_MIN           = 1.0;
const float    LIMIT_OC_MAX           = 15.0;
const uint16_t LIMIT_DELAY_MAX_MS     = 5000;
const uint16_t LIMIT_RECONNECT_MIN_S  = 1;
const uint16_t LIMIT_RECONNECT_MAX_S  = 120;

// ---- Timing (milliseconds) ----
const unsigned long MEASURE_INTERVAL_MS    = 100;
const unsigned long DISPLAY_INTERVAL_MS    = 250;
const unsigned long MENU_TIMEOUT_MS        = 20000;
const unsigned long BUTTON_DEBOUNCE_MS     = 30;
const unsigned long BUTTON_REPEAT_AFTER_MS = 500;
const unsigned long BUTTON_REPEAT_EVERY_MS = 120;

// ---- Serial ----
const long SERIAL_BAUD     = 115200;
const int  RX_BUFFER_SIZE  = 40;       // longest accepted command line

// ---- Storage ----
const uint8_t SETTINGS_MAGIC = 0xA7;   // marks the EEPROM as written by this program
const int     EEPROM_ADDRESS = 0;

// =====================================================================
// 3. TYPES AND GLOBALS
// =====================================================================
enum State {
  STATE_OFF,        // switched off by the user
  STATE_NORMAL,     // load connected and being watched
  STATE_FAULT,      // tripped, load disconnected
  STATE_COOLDOWN    // waiting out the reconnect delay
};

enum FaultType {
  FAULT_NONE,
  FAULT_OV,         // over-voltage
  FAULT_UV,         // under-voltage
  FAULT_OC          // over-current
};

enum MenuPage {
  PAGE_RUN,         // normal run screen (menu closed)
  PAGE_OV,
  PAGE_UV,
  PAGE_OC,
  PAGE_TRIP_DELAY,
  PAGE_RECONNECT,
  PAGE_AUTO,
  PAGE_COUNT        // not a page: marks the end of the list
};

struct Settings {
  uint8_t  magic;
  float    overVoltageV;
  float    underVoltageV;
  float    overCurrentA;
  uint16_t tripDelayMs;       // a fault must last this long before tripping
  uint16_t reconnectDelayS;   // wait this long before reconnecting
  uint8_t  autoReconnect;     // 1 = automatic, 0 = wait for POWER button
};

struct Button {
  uint8_t       pin;
  bool          lastReading;
  bool          isDown;
  unsigned long lastChangeAt;
  unsigned long pressedAt;
  unsigned long lastRepeatAt;
};

LiquidCrystal_I2C lcd(0x27, 16, 2);

Settings settings;            // active settings used by the protection logic
Settings editCopy;            // working copy while the menu is open

float gVolts = 0;             // latest RMS readings
float gAmps  = 0;

State         state          = STATE_COOLDOWN;
FaultType     activeFault    = FAULT_NONE;   // reason for the current trip
FaultType     pendingFault   = FAULT_NONE;   // fault seen, waiting for trip delay
unsigned long pendingSince   = 0;
unsigned long cooldownEndsAt = 0;

MenuPage      menuPage    = PAGE_RUN;
unsigned long lastInputAt = 0;

Button btnMode  = { PIN_BTN_MODE,  false, false, 0, 0, 0 };
Button btnUp    = { PIN_BTN_UP,    false, false, 0, 0, 0 };
Button btnDown  = { PIN_BTN_DOWN,  false, false, 0, 0, 0 };
Button btnPower = { PIN_BTN_POWER, false, false, 0, 0, 0 };

unsigned long lastMeasureAt = 0;
unsigned long lastDisplayAt = 0;

char rxBuffer[RX_BUFFER_SIZE];   // command line being received
int  rxLength   = 0;
bool rxOverflow = false;         // line was too long: reject it

// =====================================================================
// 4. SETTINGS (EEPROM)
// =====================================================================

// True if every value is inside its allowed range.
bool settingsAreValid(const Settings &s) {
  return s.overVoltageV    >= LIMIT_OV_MIN && s.overVoltageV <= LIMIT_OV_MAX
      && s.underVoltageV   >= LIMIT_UV_MIN && s.underVoltageV <= s.overVoltageV - MIN_BAND_VOLTS
      && s.overCurrentA    >= LIMIT_OC_MIN && s.overCurrentA <= LIMIT_OC_MAX
      && s.tripDelayMs     <= LIMIT_DELAY_MAX_MS
      && s.reconnectDelayS >= LIMIT_RECONNECT_MIN_S
      && s.reconnectDelayS <= LIMIT_RECONNECT_MAX_S
      && s.autoReconnect   <= 1;
}

void loadDefaultSettings() {
  settings.magic           = SETTINGS_MAGIC;
  settings.overVoltageV    = 250.0;
  settings.underVoltageV   = 180.0;
  settings.overCurrentA    = 10.0;
  settings.tripDelayMs     = 200;
  settings.reconnectDelayS = 5;
  settings.autoReconnect   = 1;
}

void saveSettings() {
  EEPROM.put(EEPROM_ADDRESS, settings);   // put() only rewrites bytes that changed
}

// Reads the saved settings; falls back to defaults if the EEPROM is blank
// or holds invalid data.
void loadSettings() {
  EEPROM.get(EEPROM_ADDRESS, settings);
  if (settings.magic != SETTINGS_MAGIC || !settingsAreValid(settings)) {
    loadDefaultSettings();
    saveSettings();
  }
}

// =====================================================================
// 5. MEASUREMENT
// =====================================================================

// Returns the RMS value (in volts or amps) of the signal on `pin`.
//
// Welford's online algorithm gives mean and variance in one pass without an
// array (the Uno has only 2 KB of RAM) and stays accurate even though the
// signal sits on a large VCC/2 offset. RMS of an AC signal equals the
// standard deviation of its samples, so no separate offset removal is needed.
//
// The simulated signal is a sine wave whose amplitude comes from the pot,
// plus +-1 count of noise. It goes through the same RMS maths a real sensor
// would, so the Wokwi behaviour matches the real-hardware code path.
float measureRms(uint8_t pin, float fullScale, float countsPerUnit) {
  float simRms  = analogRead(pin) / 1023.0 * fullScale;   // pot -> RMS in real units
  float simPeak = simRms * countsPerUnit * 1.4142;        // peak in ADC counts

  float mean = 0.0;
  float m2   = 0.0;

  for (int n = 0; n < SAMPLES_PER_WINDOW; n++) {
    float angle  = 2.0 * PI * n / SAMPLES_PER_WINDOW;
    float sample = ADC_MIDPOINT + simPeak * sin(angle) + random(-1, 2);

    float delta = sample - mean;
    mean += delta / (n + 1);
    m2   += delta * (sample - mean);
  }

  float variance = m2 / SAMPLES_PER_WINDOW;
  if (variance < 0) variance = 0;
  return sqrt(variance) / countsPerUnit;
}

// =====================================================================
// 6. PROTECTION STATE MACHINE
//
//    COOLDOWN --(wait done, line healthy)-----------> NORMAL
//    COOLDOWN --(line unsafe again)-----------------> FAULT
//    NORMAL   --(fault lasts > trip delay, or hard)-> FAULT
//    FAULT    --(line healthy, auto mode)-----------> COOLDOWN
//    FAULT    --(POWER, line healthy)---------------> COOLDOWN
//    FAULT    --(POWER, line unsafe)----------------> OFF
//    NORMAL / COOLDOWN --(POWER)--------------------> OFF
//    OFF      --(POWER)-----------------------------> COOLDOWN
// =====================================================================
const char *faultName(FaultType f) {
  switch (f) {
    case FAULT_OV: return "OV";
    case FAULT_UV: return "UV";
    case FAULT_OC: return "OC";
    default:       return "NONE";
  }
}

const char *stateName(State s) {
  switch (s) {
    case STATE_OFF:    return "OFF";
    case STATE_NORMAL: return "NORMAL";
    case STATE_FAULT:  return "FAULT";
    default:           return "COOLDOWN";
  }
}

void setRelay(bool on) {
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
}

// Checks the readings against the limits.
//   withHysteresis = false : "is it faulty right now?"      (used to trip)
//   withHysteresis = true  : "is it safely healthy again?"  (used to reconnect)
// The stricter reconnect test stops the relay chattering on and off when the
// supply hovers right at a limit.
FaultType detectFault(bool withHysteresis) {
  float voltMargin = withHysteresis ? HYSTERESIS_VOLTS   : 0.0;
  float currFactor = withHysteresis ? HYSTERESIS_CURRENT : 1.0;

  if (gVolts > settings.overVoltageV  - voltMargin) return FAULT_OV;
  if (gVolts < settings.underVoltageV + voltMargin) return FAULT_UV;
  if (gAmps  > settings.overCurrentA  * currFactor) return FAULT_OC;
  return FAULT_NONE;
}

// Disconnects the load and raises the alarm.
void tripRelay(FaultType fault) {
  setRelay(false);                    // disconnect first, bookkeeping after
  state        = STATE_FAULT;
  activeFault  = fault;
  pendingFault = FAULT_NONE;
  tone(PIN_BUZZER, 2000);
}

void startCooldown(unsigned long now) {
  state          = STATE_COOLDOWN;
  cooldownEndsAt = now + (unsigned long)settings.reconnectDelayS * 1000UL;
  noTone(PIN_BUZZER);
}

void connectLoad() {
  setRelay(true);
  state        = STATE_NORMAL;
  activeFault  = FAULT_NONE;
  pendingFault = FAULT_NONE;
  noTone(PIN_BUZZER);
}

void switchOff() {
  setRelay(false);
  state        = STATE_OFF;
  activeFault  = FAULT_NONE;
  pendingFault = FAULT_NONE;
  noTone(PIN_BUZZER);
}

// Load is connected: trip when a limit stays exceeded for the trip delay,
// or immediately for a hard overload / large over-voltage surge.
//
// The delay timer starts at the first faulty reading and only resets when the
// line is fully back inside the limits. If the fault type changes meanwhile
// (e.g. OV becomes OC) the timer keeps running, so a signal bouncing between
// two faults can never keep postponing the trip.
void checkForTrip(unsigned long now) {
  FaultType fault = detectFault(false);

  if (fault == FAULT_NONE) {
    pendingFault = FAULT_NONE;
    return;
  }

  if (pendingFault == FAULT_NONE) {      // first faulty reading of this streak
    pendingSince = now;
  }
  pendingFault = fault;

  bool hardFault = gAmps  > settings.overCurrentA * HARD_OVERLOAD_RATIO
                || gVolts > settings.overVoltageV + HARD_OVERVOLT_MARGIN;
  bool delayElapsed = (now - pendingSince) >= settings.tripDelayMs;

  if (hardFault || delayElapsed) {
    tripRelay(fault);
  }
}

// Runs once per measurement.
void runProtection(unsigned long now) {
  switch (state) {
    case STATE_OFF:
      break;

    case STATE_NORMAL:
      checkForTrip(now);
      break;

    case STATE_FAULT:
      if (settings.autoReconnect && detectFault(true) == FAULT_NONE) {
        startCooldown(now);
      }
      break;

    case STATE_COOLDOWN: {
      FaultType problem = detectFault(true);
      if (problem != FAULT_NONE) {                 // went bad again while waiting
        state       = STATE_FAULT;
        activeFault = problem;
        tone(PIN_BUZZER, 2000);
      } else if ((long)(now - cooldownEndsAt) >= 0) {   // rollover-safe comparison
        connectLoad();
      }
      break;
    }
  }
}

// POWER button behaviour (see the state diagram above).
void powerButtonAction(unsigned long now) {
  switch (state) {
    case STATE_OFF:
      startCooldown(now);
      break;
    case STATE_NORMAL:
    case STATE_COOLDOWN:
      switchOff();
      break;
    case STATE_FAULT:
      if (detectFault(true) == FAULT_NONE) startCooldown(now);
      else                                 switchOff();    // silence the alarm
      break;
  }
}

// =====================================================================
// 7. BUTTONS AND SETTINGS MENU
// =====================================================================

// Debounced button read. Returns true once per press, and repeatedly while
// the button is held if `allowRepeat` is set.
bool buttonPressed(Button &b, bool allowRepeat, unsigned long now) {
  bool reading = (digitalRead(b.pin) == LOW);    // INPUT_PULLUP: pressed = LOW
  bool fired   = false;

  if (reading != b.lastReading) {                // reading changed: restart debounce
    b.lastReading  = reading;
    b.lastChangeAt = now;
  }

  if ((now - b.lastChangeAt) > BUTTON_DEBOUNCE_MS && reading != b.isDown) {
    b.isDown = reading;                          // stable long enough: accept it
    if (b.isDown) {
      fired          = true;
      b.pressedAt    = now;
      b.lastRepeatAt = now;
    }
  }

  if (allowRepeat && b.isDown
      && (now - b.pressedAt)    > BUTTON_REPEAT_AFTER_MS
      && (now - b.lastRepeatAt) > BUTTON_REPEAT_EVERY_MS) {
    fired          = true;
    b.lastRepeatAt = now;
  }
  return fired;
}

// UP / DOWN on a settings page. Edits the copy, never the live settings.
// `direction` is +1 or -1. Values are clamped to their allowed range, and
// the under-voltage limit is kept at least MIN_BAND_VOLTS below over-voltage.
void adjustSetting(int direction) {
  switch (menuPage) {
    case PAGE_OV:
      editCopy.overVoltageV = constrain(editCopy.overVoltageV + direction,
                                        LIMIT_OV_MIN, LIMIT_OV_MAX);
      if (editCopy.underVoltageV > editCopy.overVoltageV - MIN_BAND_VOLTS) {
        editCopy.underVoltageV = editCopy.overVoltageV - MIN_BAND_VOLTS;
      }
      break;

    case PAGE_UV:
      editCopy.underVoltageV = constrain(editCopy.underVoltageV + direction,
                                         LIMIT_UV_MIN, editCopy.overVoltageV - MIN_BAND_VOLTS);
      break;

    case PAGE_OC:
      editCopy.overCurrentA = constrain(editCopy.overCurrentA + 0.5 * direction,
                                        LIMIT_OC_MIN, LIMIT_OC_MAX);
      break;

    case PAGE_TRIP_DELAY:     // 100 ms steps = the measurement resolution
      editCopy.tripDelayMs = constrain((int)editCopy.tripDelayMs + 100 * direction,
                                       0, (int)LIMIT_DELAY_MAX_MS);
      break;

    case PAGE_RECONNECT:
      editCopy.reconnectDelayS = constrain((int)editCopy.reconnectDelayS + direction,
                                           (int)LIMIT_RECONNECT_MIN_S, (int)LIMIT_RECONNECT_MAX_S);
      break;

    case PAGE_AUTO:
      editCopy.autoReconnect = editCopy.autoReconnect ? 0 : 1;
      break;

    default:
      break;
  }
}

// Closes the menu. The edited copy becomes the active settings and is saved
// to EEPROM (only if something actually changed).
void leaveMenu() {
  menuPage = PAGE_RUN;
  if (settingsAreValid(editCopy) && memcmp(&editCopy, &settings, sizeof(Settings)) != 0) {
    settings = editCopy;
    saveSettings();
  }
}

// MODE button: open the menu, then step through the pages.
void goToNextPage() {
  if (menuPage == PAGE_RUN) editCopy = settings;   // entering the menu: fresh copy
  menuPage = (MenuPage)(menuPage + 1);
  if (menuPage >= PAGE_COUNT) leaveMenu();         // after the last page: save and exit
}

void handleButtons(unsigned long now) {
  bool modePressed  = buttonPressed(btnMode,  false, now);
  bool upPressed    = buttonPressed(btnUp,    true,  now);
  bool downPressed  = buttonPressed(btnDown,  true,  now);
  bool powerPressed = buttonPressed(btnPower, false, now);

  if (modePressed || upPressed || downPressed || powerPressed) lastInputAt = now;

  if (powerPressed) powerButtonAction(now);
  if (modePressed)  goToNextPage();

  if (menuPage != PAGE_RUN) {
    if (upPressed)   adjustSetting(+1);
    if (downPressed) adjustSetting(-1);
    if ((now - lastInputAt) > MENU_TIMEOUT_MS) leaveMenu();   // idle: save and exit
  }
}

// =====================================================================
// 8. DISPLAY AND INDICATORS
// =====================================================================

// Writes exactly 16 characters so old text is always overwritten.
void lcdPrintLine(uint8_t row, const char *text) {
  lcd.setCursor(0, row);
  int i = 0;
  while (text[i] != '\0' && i < 16) { lcd.write(text[i]); i++; }
  while (i < 16)                    { lcd.write(' ');     i++; }
}

// Run screen: line 0 = live readings, line 1 = state.
void buildRunScreen(unsigned long now, char *line0, char *line1) {
  char volts[12];
  char amps[12];
  dtostrf(gVolts, 5, 1, volts);
  dtostrf(gAmps,  4, 1, amps);
  snprintf(line0, 20, "V:%s I:%sA", volts, amps);

  switch (state) {
    case STATE_OFF:
      snprintf(line1, 20, "OFF - press PWR");
      break;
    case STATE_NORMAL:
      snprintf(line1, 20, "ON  Load powered");
      break;
    case STATE_FAULT:
      if (settings.autoReconnect) snprintf(line1, 20, "FAULT %s (auto)", faultName(activeFault));
      else                        snprintf(line1, 20, "FAULT %s: PWR=on", faultName(activeFault));
      break;
    case STATE_COOLDOWN: {
      long remainingMs = (long)(cooldownEndsAt - now);
      if (remainingMs < 0) remainingMs = 0;
      snprintf(line1, 20, "Reconnect in %us", (unsigned int)((remainingMs + 999) / 1000));
      break;
    }
  }
}

// Menu screen: shows the value from the edit copy, not the live settings.
void buildMenuScreen(char *line0, char *line1) {
  char number[12];

  switch (menuPage) {
    case PAGE_OV:
      snprintf(line0, 20, "1/6 Over-voltage");
      snprintf(line1, 20, "%d V   UP/DOWN", (int)editCopy.overVoltageV);
      break;
    case PAGE_UV:
      snprintf(line0, 20, "2/6 Under-volt.");
      snprintf(line1, 20, "%d V   UP/DOWN", (int)editCopy.underVoltageV);
      break;
    case PAGE_OC:
      dtostrf(editCopy.overCurrentA, 4, 1, number);
      snprintf(line0, 20, "3/6 Over-current");
      snprintf(line1, 20, "%s A  UP/DOWN", number);
      break;
    case PAGE_TRIP_DELAY:
      snprintf(line0, 20, "4/6 Trip delay");
      snprintf(line1, 20, "%u ms UP/DOWN", editCopy.tripDelayMs);
      break;
    case PAGE_RECONNECT:
      snprintf(line0, 20, "5/6 Reconnect");
      snprintf(line1, 20, "%u s   UP/DOWN", editCopy.reconnectDelayS);
      break;
    case PAGE_AUTO:
      snprintf(line0, 20, "6/6 Restart mode");
      snprintf(line1, 20, "%s UP/DOWN", editCopy.autoReconnect ? "AUTO  " : "MANUAL");
      break;
    default:
      line0[0] = '\0';
      line1[0] = '\0';
      break;
  }
}

void updateDisplay(unsigned long now) {
  char line0[20];
  char line1[20];

  if (menuPage == PAGE_RUN) buildRunScreen(now, line0, line1);
  else                      buildMenuScreen(line0, line1);

  lcdPrintLine(0, line0);
  lcdPrintLine(1, line1);
}

// LEDs follow the state. The buzzer is not handled here: it starts in
// tripRelay() / COOLDOWN->FAULT and stops in connectLoad(), switchOff()
// and startCooldown().
void updateIndicators(unsigned long now) {
  bool blinkOn = ((now / 250) % 2) == 1;

  bool green  = (state == STATE_NORMAL);
  bool red    = (state == STATE_FAULT);
  bool yellow = (state == STATE_COOLDOWN && blinkOn)                  // counting down
             || (state == STATE_NORMAL && pendingFault != FAULT_NONE); // fault pending

  digitalWrite(PIN_LED_GREEN,  green  ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,    red    ? HIGH : LOW);
}

// =====================================================================
// 9. COMPUTER LINK (USB serial)
// =====================================================================

// Live readings and state as one JSON line.
void printStatus(unsigned long now) {
  long remainingMs = 0;
  if (state == STATE_COOLDOWN) remainingMs = (long)(cooldownEndsAt - now);
  if (remainingMs < 0) remainingMs = 0;

  Serial.print("{\"v\":");              Serial.print(gVolts, 1);
  Serial.print(",\"i\":");              Serial.print(gAmps, 2);
  Serial.print(",\"relay\":");          Serial.print(digitalRead(PIN_RELAY));
  Serial.print(",\"state\":\"");        Serial.print(stateName(state));
  Serial.print("\",\"fault\":\"");      Serial.print(faultName(activeFault));
  Serial.print("\",\"reconnect_in\":"); Serial.print((remainingMs + 999) / 1000);
  Serial.println("}");
}

// Current settings as one JSON line.
void printSettings() {
  Serial.print("{\"ov\":");    Serial.print(settings.overVoltageV, 0);
  Serial.print(",\"uv\":");    Serial.print(settings.underVoltageV, 0);
  Serial.print(",\"oc\":");    Serial.print(settings.overCurrentA, 1);
  Serial.print(",\"td\":");    Serial.print(settings.tripDelayMs);
  Serial.print(",\"rc\":");    Serial.print(settings.reconnectDelayS);
  Serial.print(",\"auto\":");  Serial.print(settings.autoReconnect);
  Serial.println("}");
}

// Converts text to a number. The whole text must be a number, so "abc" or
// "12x" are rejected instead of silently becoming 0 or 12.
bool parseNumber(const char *text, float &result) {
  if (text == NULL) return false;
  char *end;
  double value = strtod(text, &end);
  if (end == text || *end != '\0') return false;
  result = (float)value;
  return true;
}

// Changes one setting. The change is made on a copy and only committed if
// the whole set of settings is still valid, exactly like the LCD menu.
bool applySetting(const char *key, float value) {
  if (value < 0 || value > 60000) return false;

  Settings candidate = settings;
  if      (strcmp(key, "OV")   == 0) candidate.overVoltageV    = value;
  else if (strcmp(key, "UV")   == 0) candidate.underVoltageV   = value;
  else if (strcmp(key, "OC")   == 0) candidate.overCurrentA    = value;
  else if (strcmp(key, "TD")   == 0) candidate.tripDelayMs     = (uint16_t)value;
  else if (strcmp(key, "RC")   == 0) candidate.reconnectDelayS = (uint16_t)value;
  else if (strcmp(key, "AUTO") == 0) candidate.autoReconnect   = (value != 0) ? 1 : 0;
  else return false;

  if (!settingsAreValid(candidate)) return false;
  settings = candidate;
  saveSettings();
  return true;
}

// Executes one complete command line (modified in place).
void handleCommand(char *line, unsigned long now) {
  for (char *p = line; *p != '\0'; p++) *p = (char)toupper(*p);   // not case sensitive

  char *command = strtok(line, " ");
  if (command == NULL) return;

  if (strcmp(command, "GET") == 0) {
    printStatus(now);

  } else if (strcmp(command, "GETCFG") == 0) {
    printSettings();

  } else if (strcmp(command, "SET") == 0) {
    char *key   = strtok(NULL, " ");
    char *text  = strtok(NULL, " ");
    float value;

    if (key == NULL || text == NULL) {
      Serial.println("ERR usage: SET <OV|UV|OC|TD|RC|AUTO> <value>");
    } else if (menuPage != PAGE_RUN) {
      Serial.println("ERR settings menu is open on the device");
    } else if (!parseNumber(text, value)) {
      Serial.println("ERR value is not a number");
    } else if (applySetting(key, value)) {
      Serial.println("OK");
    } else {
      Serial.println("ERR invalid key or value");
    }

  } else if (strcmp(command, "HELP") == 0) {
    Serial.println("GET | GETCFG | SET <OV|UV|OC|TD|RC|AUTO> <value>");

  } else {
    Serial.println("ERR unknown command, try HELP");
  }
}

// Collects characters until a line ends, then runs it. Over-long lines are
// rejected whole instead of being executed in truncated form.
void readSerialCommands(unsigned long now) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxOverflow) {
        Serial.println("ERR line too long");
      } else if (rxLength > 0) {
        rxBuffer[rxLength] = '\0';
        handleCommand(rxBuffer, now);
      }
      rxLength   = 0;
      rxOverflow = false;
    } else if (rxLength < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxLength++] = c;
    } else {
      rxOverflow = true;
    }
  }
}

// =====================================================================
// 10. SETUP AND MAIN LOOP
// =====================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Set the relay level before making the pin an output so it never pulses.
  digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_RELAY, OUTPUT);

  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_BTN_MODE,   INPUT_PULLUP);
  pinMode(PIN_BTN_UP,     INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN,   INPUT_PULLUP);
  pinMode(PIN_BTN_POWER,  INPUT_PULLUP);

  randomSeed(analogRead(A2));          // seed the simulated sensor noise (A2 is unconnected)

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcdPrintLine(0, "Voltage Protect");
  lcdPrintLine(1, "Starting...");

  loadSettings();

  unsigned long now = millis();
  lastInputAt = now;
  startCooldown(now);                  // wait before powering the load at start-up

  Serial.println("Voltage Protector ready. Type HELP.");
}

void loop() {
  unsigned long now = millis();

  readSerialCommands(now);

  if (now - lastMeasureAt >= MEASURE_INTERVAL_MS) {
    lastMeasureAt = now;
    gVolts = measureRms(PIN_VOLT_SENSE, VOLT_FULL_SCALE, COUNTS_PER_VOLT);
    gAmps  = measureRms(PIN_CURR_SENSE, CURR_FULL_SCALE, COUNTS_PER_AMP);
    runProtection(now);
  }

  handleButtons(now);
  updateIndicators(now);

  if (now - lastDisplayAt >= DISPLAY_INTERVAL_MS) {
    lastDisplayAt = now;
    updateDisplay(now);
  }
}

