/*
ESP32 sketch to replace MK5017 used in several HeathKit clocks
Requires shift register and level shifting interface which connects to the original board via chip socket
No modifications to the original clock except adding a 5v power supply module at the 120v input line
Heathkit GC-1092D, Heathkit GC-1005, Heathkit GC-1092A fully tested. GC-1094 not tested (yet)
Display updates are state driven and writing to display is handled by hardware timer

WIFI mode:
ESP Touch (also snooze or alarmoff switches) changes display from Time to Date, Date to display off, display off to on. 
GC-1092A/D - Original 555 touch hardware can be used, however each touch event must be at least 5 seconds apart.

Set timezone - https://github.com/ropg/ezTime This can handle DST changes automatically
Set Weekend and time of day blanking to suit.
Adjust Wifi settings if needed - Normal mode is setup via AP mode portal (192.168.4.1)
Clock model is set interactively if not found in EEPROM at startup. Toggle Hour/Month switch until the correct model is displayed, then toggle and reset Time Set.

MK5017 mode:
Switches and all behavior faithful to original hardware
RTC simulates battery backup of the GC-1092A/D (but works on all models) - Will consider time valid for configurable number of hours.
Switching modes will reset the model. Set model by pressing Hour/Month switch until the correct model is displayed, then press and release Time Set. 

Default mode is WIFI.
To switch mode hold Hour/Month switch at boot, Display will go from 888888 to ------, release and clock will restart into other mode.
Holding Hour/Month switch at power up will change between modes. Display will go from 888888 to ------ and reboot. 

OTA
Press Hour/Month and Minute/Day advance together to enter OTA mode. OTA mode uses AP mode with password and times out after 10 min.

Troubleshooting:
Holding Minute advance at power up will reset the Model, RTC data and disable sw_HOLD (60Hz detection)
Manual reboot - toggle Time Set 3 times in 5 seconds

Select ESP32-WROOM-DA Module in Arduino IDE
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <ezTime.h>
#include "Arduino.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESP32TimerInterrupt.h>
#include <RTClib.h>
#include "driver/rmt.h"
#include "driver/gpio.h"

// === Global variables and constants ===

// Time of day and day of week blanking - only for WIFI mode
// 7:30a - 6p M-F
bool timeBlankingEnabled = true;    // enable/disable time blanking
int ontime = 73000;                 // 24 hour time, no leading zero
int offtime = 180000;               // 24 hour time, no leading zero
bool weekendBlankingEnabled = true; // enable/disable weekend blanking
String weekendDay1 = "Saturday";    // days to blank display
String weekendDay2 = "Sunday";      // days to blank display
int showdatesec = 10;                // Seconds to show date when touched

// Timezone and NTP setup  ezTime https://github.com/ropg/ezTime
// POSIX definition optional, but prevents additional lookup traffic
#define LOCALTZ_POSIX "MST+7MDT,M3.2.0/2,M11.1.0/2" // MST timezone definition including DST On/Off
Timezone tz;
String ntpserver = "pool.ntp.org"; // NTP server
int ntpinterval = 500;                        // NTP update interval
ezDebugLevel_t ntpdebug = INFO;               // ezTime debug level
RTC_DS3231 rtc;

// Wifi stuff
// Wifi will revert to setup AP mode if no connection available
// OTA mode is triggered and runs in AP mode with timeout
String wifihostname = "Heathkit-Clock";        // Hostname of ESP device
String wifisetupssid = "Heathkit-Clock Setup"; // Setup AP mode SSID - no password
String wifiotassid = "Heathkit-OTA";           // OTA AP mode SSID
String wifiotapass = "update123";              // OTA AP mode password

// Pin definitions
const uint8_t pinData = 13;  // shift register data
const uint8_t pinClock = 14; // shift register clock
const uint8_t pinLatch = 15; // shift register latch
const uint8_t pinKA = 17;    // KA pin on socket (22)
const uint8_t pinKB = 16;    // KB pin on socket (21)
//Handbuilt board is backwards
//const uint8_t pinKA = 16;    // KA pin on socket (22)
//const uint8_t pinKB = 17;    // KB pin on socket (21)

const uint8_t pinTouch = 4;  // Touch input 
const uint8_t pinAM = 18;    // AM pin on socket (16)
#define RMT_PIN GPIO_NUM_19  // Tone pin on socket (20)
const int sw_HOLD_Pin = 23;  // 60Hz pin on socket (23)
bool Disable_swHOLD = false; // Disable HOLD function in case hardware issue
const int BatteryBackup = 2; // Hours to consider 'Battery backup' valid

// Should not need to change anything below here
#define RMT_CH RMT_CHANNEL_1
static bool toneActive = false;
#define CYCLES_PER_HALFSECOND 425
#define ITEMS_PER_CYCLE 7
#define TOTAL_ITEMS (CYCLES_PER_HALFSECOND * ITEMS_PER_CYCLE)
static rmt_item32_t pulse_items[TOTAL_ITEMS];

int datecounter = 0;
bool manualOverride = false;        // User override state (toggle ON/OFF)
bool overrideForBlank = false;      // true if override is forcing blank
bool touchoverride = false;         // Short-term override for date display

ESP32Timer ITimer1(1);
TaskHandle_t displayTaskHandle = nullptr;
const uint32_t T_INTERVAL = 525; // timer interval
const uint8_t NUM_DIGITS = 6;
volatile uint8_t curdigit = 0;
#define SEGS_OFF 0x00   //      all segments off for blanking
#define SEG_INVERT 0x00 //      use 0xff to invert SEG outputs, 0x00 for std SEG output - 00 for MK5717
#define DIG_INVERT 0x00 //      use 0xff to invert DIG outputs, 0x00 for std DIG output - 00 for MK5717
uint8_t displayBuffer[NUM_DIGITS] = {SEGS_OFF, SEGS_OFF, SEGS_OFF, SEGS_OFF, SEGS_OFF, SEGS_OFF};
// Segment pattern key
//    - a -
//   f    b
//   - g -
//   e    c
//   - d -

const uint8_t
    chars[] = //      Segment patterns for characters
    {
        // Dgfedcba
        0b00111111, // 0
        0b00000110, // 1
        0b01011011, // 2
        0b01001111, // 3
        0b01100110, // 4
        0b01101101, // 5
        0b01111101, // 6
        0b00000111, // 7
        0b01111111, // 8
        0b01101111, // 9
        0b01110111, // A  10
        0b01011111, // a  11
        0b00111001, // C  12
        0b01011000, // c  13
        0b01011110, // d  14
        0b01111001, // E  15
        0b01110001, // F  16
        0b00111000, // L  17
        0b01010100, // n  18
        0b00111111, // O  19
        0b01011100, // o  20
        0b01110011, // P  21
        0b01010000, // r  22
        0b01101101, // S  23
        0b01111000, // t  24
        0b00111110, // U  25
        0b00000000, // blank 26
        0b01000000  // Dash  27
};

#define CHAR_BLANK 26
#define CHAR_DASH 27

const uint8_t digits[] = {
    0b10000000,
    0b01000000,
    0b00100000,
    0b00010000,
    0b00001000,
    0b00000100,
}; // Digit pin positions

enum DisplayMode
{
    MODE_TIME,
    MODE_DATE,
    MODE_SET,
    MODE_MESSAGE
};
DisplayMode currentDisplayMode = MODE_MESSAGE;
enum Emulation : uint8_t
{
    WIFI = 0,
    MK5017 = 1,
    EmulationCount
};
enum Model : uint8_t
{
    MODEL_GC_1005 = 0,
    MODEL_GC_1092D = 1,
    MODEL_GC_1092A = 2,
    MODEL_GC_1094 = 3,
    MODEL_BOGUS = 4,
    ModelCount
};
const char *modelToString(Model m)
{
    switch (m)
    {
    case MODEL_GC_1005:
        return "GC-1005";
    case MODEL_GC_1092D:
        return "GC-1092D";
    case MODEL_GC_1092A:
        return "GC-1092A";
    case MODEL_GC_1094:
        return "GC-1094";
    case MODEL_BOGUS:
        return "Unset";
    default:
        return "Unknown";
    }
}

constexpr int MESSAGE_COUNT = 10;
const uint8_t messagearray[MESSAGE_COUNT][NUM_DIGITS] = {
    {1, 0, 0, 5, CHAR_BLANK, CHAR_BLANK}, // 1005
    {1, 0, 9, 2, 14, CHAR_BLANK},         // 1092D
    {1, 0, 9, 2, 10, CHAR_BLANK},         // 1092A
    {1, 0, 9, 4, CHAR_BLANK, CHAR_BLANK}, // 1094
    {23, 24, 10, 22, 24, CHAR_BLANK},     // S t A r t
    {12, 20, 18, 13, 24, 14},             // C o n c t d
    {23, 15, 24, 25, 21, CHAR_BLANK},     // S E t U P
    {8, 8, 8, 8, 8, 8},
    {CHAR_DASH, CHAR_DASH, CHAR_DASH, CHAR_DASH, CHAR_DASH, CHAR_DASH},
    {CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK},
};
int messageindex = 7; // default to eights

struct EmulationInfo
{
    Emulation emulation;
    Model model;
};
EmulationInfo info;
const int EEPROM_ADDR_EMULATION = 0;
const int EEPROM_ADDR_MODEL = 1;
constexpr Emulation defaultEmulation = WIFI;
constexpr Model defaultModel = MODEL_GC_1005;

// end of display stuff

// Switches
struct DebouncedSwitch
{
    bool state = false;      // current stable state
    bool changed = false;    // true for one loop when state toggles
    bool lastRead = false;   // last raw sample
    uint32_t lastChange = 0; // micros() of last raw transition
    uint32_t changedAt = 0;  // when .changed was set
};

DebouncedSwitch sw_1, sw_2, sw_3, sw_4, sw_5, sw_6, sw_7, sw_HOLD;
// static DebouncedSwitch *switches[] = { &sw_1, &sw_2, &sw_3, &sw_4, &sw_5, &sw_6, &sw_7, &sw_HOLD };
const uint32_t debounceDelay = 1000; // us

// touch
int threshold = 30;
bool touchActive = false;
bool lastTouchActive = true;
bool testingLower = true;

// Extra inputs and globals for emulation
bool is24HourMode = false; // Default to 12-hour
constexpr uint32_t HALF_OFFSET_US = 500000; // 0.5 s in microseconds
uint32_t secondEdgeMicros = 0;              // micros at the top-of-second edge
volatile bool halfSecond = false;
volatile bool topOfSecond = false;
bool halfSecondFired = false;
volatile int lastMinuteSeen = 0;

// Set Mode Globals
enum EditTarget
{
    EDIT_TIME,
    EDIT_ALARM,
    EDIT_DATE,
    EDIT_MODEL
};

String timestring = "";
uint8_t hr24 = 255; // 0–23, canonical hour, 255 means unset
uint8_t minutes;    // 0–59
uint8_t seconds;    // 0–59
String datestring = "";
String setstring = "";
String displaystring = "";
EditTarget activeTarget = EDIT_TIME;
bool inEditMode = false;
bool dateSet = false;
volatile bool dateChanged = false;
bool autoDate = false;

// Alarm globals
enum AlarmState
{
    ALARM_IDLE,
    ALARM_ARMED,
    ALARM_SOUNDING,
    ALARM_SNOOZE
};
AlarmState alarmState = ALARM_IDLE;
DateTime alarmDT = DateTime(2025, 1, 1, 0, 0, 0);
bool alarmSet = false;
bool alarmEnabled = false;
uint32_t timeSeconds = -1;
uint32_t alarmSeconds = 0;
uint32_t snoozeTargetS = 0;
const uint8_t SNOOZE_MIN = 7;
const uint8_t ALARM_MAX_MIN = 60;

// Wifi OTA fallback
bool otaModeActive = false;              // Tracks whether OTA mode is currently active
unsigned long otaStartTime = 0;          // Timestamp when OTA mode was triggered
const unsigned long otaTimeout = 600000; // Timeout duration - 10 minutes
// === End of global variables and constants ===

// === Forward function declarations ===
void setupConfigInit();
void setupEmulationInit();
bool IRAM_ATTR onDisplayTimer(void *);
void displayTask(void *pvParameters);
void gotTouchEvent();
void handleTouchEvent();
void configModeCallback(WiFiManager *myWiFiManager);
void monitorAndRecoverNtp();
void updateDisplayValue();
void updateDisplayStateIfNeeded();
void checkOtaTimeout();
void CheckSwitches();
EmulationInfo loadEEPROM();
void saveEEPROM(Emulation emu, Model model);
bool debounceSwitch(DebouncedSwitch &sw, bool raw, uint32_t nowMicros);
void DispatchSwitches();
void incrementMinuteOnes();
void incrementMinuteTens();
void incrementDate(bool m);
void incrementHours();
void handleSetMode(EditTarget target);
void enterSetMode(EditTarget target);
void handleTimedDispatches();
void updateStrings();
void commitSetMode();
void keepRTCinsync();
void updateAlarm();
void requestSnooze();
void alarmToneDriver();
void alarmToneInit();
void alarmToneStart();
void alarmToneStop();
void clearRTC();
void ResetOn3xTimeSet();
void ArduinoOTA_begin();
// =====================================

void setup()
{
    EEPROM.begin(64);
    Serial.begin(115200);
    pinMode(pinData, OUTPUT);
    pinMode(pinClock, OUTPUT);
    pinMode(pinLatch, OUTPUT);
    pinMode(pinKA, INPUT);
    pinMode(pinKB, INPUT);
    pinMode(pinAM, OUTPUT);
    pinMode(sw_HOLD_Pin, INPUT);
    xTaskCreatePinnedToCore(
        displayTask,        // Task function
        "DisplayTask",      // Name
        2048,               // Stack size
        NULL,               // Parameters
        2,                  // Priority (higher than idle)
        &displayTaskHandle, // Handle
        1                   // Core (1 = App core, 0 = Pro core)
    );
    rmt_tx_stop(RMT_CH);
    ITimer1.attachInterruptInterval(T_INTERVAL, onDisplayTimer);
    touchAttachInterrupt(pinTouch, &gotTouchEvent, threshold); 
    // Touch ISR will be activated when touchRead is lower than the Threshold
    touchInterruptSetThresholdDirection(testingLower);
    currentDisplayMode = MODE_MESSAGE;
    messageindex = 7; // eights
    updateDisplayValue();
    setupConfigInit();
    setupEmulationInit();
    alarmToneInit();
    delay(50);


} // End of setup

void loop()
{
    CheckSwitches();
    handleTimedDispatches();
    updateDisplayValue();
    handleTouchEvent();
    DispatchSwitches();
    // WIFI mode uses NTP, MK5017 mode uses DS3231 RTC
    if (info.emulation == MK5017)
    {
        if (info.model != MODEL_GC_1092D) 
        {
            updateAlarm();
            alarmToneDriver();
        }
        // Check model and set if needed
        if (info.model == MODEL_BOGUS && !inEditMode)
        {
            handleSetMode(EDIT_MODEL);
        }
    }
    else if (info.emulation == WIFI)
    {
        monitorAndRecoverNtp();
    }

    ArduinoOTA.handle();
    checkOtaTimeout();
    events(); // ezTime tick
    // Switch status readout - prints to serial for debugging. 
    //   DebouncedSwitch *switches[] = { &sw_1, &sw_2, &sw_3, &sw_4, &sw_5, &sw_6, &sw_7, &sw_HOLD };
    //   const char *labels[] = { "1.TS", "2.AS_DS", "3.H_M", "4.M_D", "5.SN_ACT", "6.AE_ATS", "7.12_24", "SW_HOLD" };
    //  for (int i = 0; i < 8; i++) {
    //    if (switches[i]->changed) {
    //      Serial.print(labels[i]);
    //      Serial.println(switches[i]->state ? " ON" : " OFF");
    //    }
    //  }
    // clear switch changed (readout only) flags at end of loop()
    //for (int i = 0; i < 8; ++i) switches[i]->changed = false;
    // end Switch status readout

    topOfSecond = false;
    halfSecond = false;
} // End of loop

void updateDisplayValue()
{ // Updates display messages but not dynamic content
    switch (currentDisplayMode)
    {
    case MODE_TIME:
    {
        for (int i = 0; i < NUM_DIGITS; i++)
        {
            int digit = timestring.charAt(i) - '0';
            if (i == 0 && digit == 0 && !is24HourMode)
            {
                displayBuffer[i] = SEGS_OFF; // blank leading zero in 12hr
            }
            else
            {
                displayBuffer[i] = chars[digit];
            }
        }
        break;
    }
    case MODE_DATE:
    {
        if (info.emulation == WIFI || info.model == MODEL_GC_1092D)
        {
            int len = datestring.length();
            for (int i = 0; i < NUM_DIGITS; i++)
            {
                if (i < len)
                {
                    int digit = datestring.charAt(i) - '0';
                    if (i == 0 && digit == 0)
                    {
                        displayBuffer[i] = SEGS_OFF; // blank leading zero
                    }
                    else
                    {
                        displayBuffer[i] = chars[digit];
                    }
                }
                else
                {
                    displayBuffer[i] = chars[CHAR_BLANK]; // Fill remaining digits with blanks
                }
            }
        }
        else
        {
            currentDisplayMode = MODE_TIME;
            Serial.println("Date display not supported in MK5017 mode for this model, reverting to time display");
        }
        break;
    }
    case MODE_SET:
    {
        int len = displaystring.length();
        for (int i = 0; i < NUM_DIGITS; i++)
        {
            if (i < len)
            {
                int digit = displaystring.charAt(i) - '0';
                if (i == 0 && digit == 0 && (!is24HourMode || activeTarget == EDIT_DATE))
                {
                    displayBuffer[i] = SEGS_OFF; // blank leading zero in 12hr
                }
                else
                {
                    displayBuffer[i] = chars[digit];
                }
            }
            else
            {
                displayBuffer[i] = chars[CHAR_BLANK]; // Fill remaining digits with blanks
            }
        }
        break;
    }
    case MODE_MESSAGE:
    {
        for (int i = 0; i < NUM_DIGITS; i++)
        {
            displayBuffer[i] = chars[messagearray[messageindex][i]];
        }
        break;
    }
    }
}

void updateDisplayStateIfNeeded() {
    // Generate time and date strings for normal run. Handle blanking if enabled
    // Get time and date for determining weekend and time-based blanking
    int currentTime = tz.dateTime("His").toInt(); // HHMMSS as int
    String currentDay = tz.dateTime("l");
    // Upate time and date so they are never stale, except hold
    if (!sw_HOLD.state) {
        timestring = tz.dateTime(is24HourMode ? "His" : "his");
        datestring = tz.dateTime("md");
    }
    bool isWeekend = weekendBlankingEnabled && (currentDay == weekendDay1 || currentDay == weekendDay2);
    bool withinDisplayWindow = (!timeBlankingEnabled) || (currentTime >= ontime && currentTime < offtime);

    if (info.emulation == MK5017)
        {
            if (info.model == MODEL_BOGUS) return;
            // Disable blanking in MK5017
            withinDisplayWindow = true;
            isWeekend = false;
            // 1092D date stuff
            if (info.model == MODEL_GC_1092D && !inEditMode)
                {
                    if (autoDate || touchoverride){
                           
                        if ((tz.second() % 10 == 8) || (tz.second() % 10 == 9)) {
                            currentDisplayMode = MODE_DATE;
                            //Serial.println("matches 8/9 sec - autodate on");
                        } else if (currentDisplayMode != MODE_TIME && !touchoverride) {
                            currentDisplayMode = MODE_TIME;
                            //Serial.println("Setting back mode_time - autodate on");
                        }
                    }
                        
                    // Date display logic
                    if (datecounter > 1 && (!autoDate || touchoverride))
                    {
                        datecounter--;
                    }
                    else if (currentDisplayMode == MODE_DATE && (!autoDate || touchoverride))
                    {
                        datecounter = 0;
                        if (touchoverride) touchoverride = false;
                        currentDisplayMode = MODE_TIME;
                    }
                
                }
        return;
    }
    // Date display logic
    if (datecounter > 0 && currentDisplayMode == MODE_DATE)
    {
        datecounter--;
    }
    else if (currentDisplayMode == MODE_DATE)
    {
        
        if (datecounter == 0) {
            if (touchoverride) touchoverride = false;
            currentDisplayMode = MODE_TIME;
        }
    }
    // Manual override logic
    if (manualOverride) {
        if (currentDisplayMode == MODE_DATE) {
            
        } else if ((!overrideForBlank && withinDisplayWindow) || (overrideForBlank && !withinDisplayWindow)) {
            manualOverride = false;
            } else {
                currentDisplayMode = overrideForBlank ? MODE_MESSAGE : MODE_TIME;
                if (overrideForBlank) messageindex = 9;
                updateDisplayValue();
                return;
            }
    }

    // Normal blanking logic
    if (currentDisplayMode != MODE_DATE && !overrideForBlank){
        
        if (isWeekend && !overrideForBlank) {
            currentDisplayMode = MODE_MESSAGE;
            messageindex = 9;
            Serial.println("Weekend blanking active");
        }
        else if (!isWeekend && !withinDisplayWindow && !overrideForBlank) {
            currentDisplayMode = MODE_MESSAGE;
            messageindex = 9;
            Serial.println("Time-of-day blanking active");
        }
        else if (!isWeekend && withinDisplayWindow && !overrideForBlank) {
            currentDisplayMode = MODE_TIME;
            //Serial.println("Normal time display active");
        }
    }
    updateDisplayValue();
}

bool IRAM_ATTR onDisplayTimer(void *)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(displayTaskHandle, &xHigherPriorityTaskWoken);
    return true;
}

void displayTask(void *pvParameters)
{
    for (;;)
    {
        // Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Perform display update here
        digitalWrite(pinLatch, LOW);
        shiftOut(pinData, pinClock, MSBFIRST, 0x00);
        shiftOut(pinData, pinClock, MSBFIRST, 0x00);
        digitalWrite(pinLatch, HIGH);
        delayMicroseconds(25);
        digitalWrite(pinLatch, LOW);
        shiftOut(pinData, pinClock, LSBFIRST, (digits[curdigit] ^ DIG_INVERT));
        shiftOut(pinData, pinClock, MSBFIRST, (displayBuffer[curdigit] ^ SEG_INVERT));
        digitalWrite(pinLatch, HIGH);
        if (++curdigit >= NUM_DIGITS)
            curdigit = 0;
    }
}

void handleTimedDispatches()
{
    if (secondChanged())
    {
        // Second changed
        updateDisplayStateIfNeeded();
        secondEdgeMicros = micros();
        topOfSecond = true;      // one-loop pulse
        halfSecondFired = false; // reset for this new second
        // Keep these up to date for alarm
        timeSeconds = (uint32_t)tz.hour() * 3600UL + (uint32_t)tz.minute() * 60UL + (uint32_t)tz.second();
        alarmSeconds = (uint32_t)alarmDT.hour() * 3600UL + (uint32_t)alarmDT.minute() * 60UL + (uint32_t)alarmDT.second();

        // Minute changed: seconds == 0 indicates top of minute
        if (tz.dateTime("s").toInt() % 60 == 0)
        {
            int minuteNow = tz.dateTime("i").toInt();
            // If this minute hasn't been processed yet, call minute handlers
            if ((minuteNow != lastMinuteSeen) && lastMinuteSeen != 0)
            {
                lastMinuteSeen = minuteNow;

                // Direct per-5-minute call when minute % 5 == 0
                if ((minuteNow % 5) == 0)
                {
                    if (info.emulation == MK5017)
                        keepRTCinsync();
                }
            }
            else
                lastMinuteSeen++;
        }
    }
    uint32_t now = micros();
    if (!halfSecondFired && (now - secondEdgeMicros) >= HALF_OFFSET_US)
    {
        halfSecond = true;      // one-loop pulse
        halfSecondFired = true; // don’t fire again until next second
    }
}
void incrementHours()
{
    hr24 = (hr24 + 1) % 24;
    updateStrings();
}

void incrementMinuteOnes()
{
    // Original behavior is only ones digit increment, no rollover
    uint8_t tens = minutes / 10;
    uint8_t ones = minutes % 10;
    ones = (ones + 1) % 10;
    minutes = tens * 10 + ones;
    updateStrings();
}

void incrementMinuteTens()
{
    minutes = (minutes + 10) % 60;
    updateStrings();
}

void incrementDate(bool m)
{
    dateChanged = true;
    int month = displaystring.substring(0, 2).toInt(); // MM
    int day = displaystring.substring(2, 4).toInt();   // DD
    if (m)
    {
        day++;
        if (day > 31)
        {
            month++;
            day = 1;
        }
        if (month > 12)
            month = 1;
    }
    else
    {
        month++;
        if (month > 12)
            month = 1;
    }
    // Update string
    char buf[5];
    sprintf(buf, "%02d%02d", month, day);
    displaystring = String(buf);
}

void handleSetMode(EditTarget target)
{
    if (!inEditMode)
    {
        enterSetMode(target);
    }

    if (inEditMode && (topOfSecond || halfSecond))
    {
        if (!sw_HOLD.state)
        {
            switch (activeTarget)
            {
            case EDIT_TIME:
            case EDIT_ALARM:
                if (sw_7.changed)
                {
                    is24HourMode = sw_7.state;
                    updateStrings();
                }
                if (sw_3.state && sw_4.state)
                    incrementMinuteTens();
                else if (sw_3.state)
                    incrementHours();
                else if (sw_4.state)
                    incrementMinuteOnes();

                break;
            case EDIT_DATE:
                if (sw_3.state)
                    incrementDate(false);
                if (sw_4.state)
                    incrementDate(true);
                break;
            case EDIT_MODEL:
                if (sw_3.state)
                {
                    messageindex = (messageindex + 1) % 4;
                    Serial.println(messageindex);
                    updateDisplayValue();
                }
                break;
            }
            updateDisplayValue();
        }
    }

    bool exitRequested = false;
    switch (activeTarget)
    {
    case EDIT_TIME:
        exitRequested = sw_HOLD.state
                            ? (sw_HOLD.changed && !sw_HOLD.state)
                            : (!sw_1.state);
        break;
    case EDIT_DATE:
    case EDIT_ALARM:
        exitRequested = !sw_2.state;
        break;
    case EDIT_MODEL:
        exitRequested = (sw_1.changed && sw_1.state);
        break;
    }

    if (inEditMode && exitRequested)
    {
        commitSetMode();
    }
}

void enterSetMode(EditTarget target)
{
    switch (target)
    {
    case EDIT_TIME:
        if (hr24 == 255)
        {
            hr24 = 0;
            minutes = 0;
            seconds = 0;
        }
        else
        {
            hr24 = tz.hour();
            minutes = tz.minute();
            seconds = sw_HOLD.state ? tz.second() : 0;
        }
        updateStrings();
        currentDisplayMode = MODE_SET;
        break;

    case EDIT_ALARM:
        if (alarmSet)
        {
            alarmDT = rtc.getAlarm1();
            hr24 = alarmDT.hour();
            minutes = alarmDT.minute();
            seconds = 0;
        }
        else
        {
            hr24 = 0;
            minutes = 0;
            seconds = 0;
        }
        updateStrings();
        currentDisplayMode = MODE_SET;
        break;

    case EDIT_DATE:
        dateChanged = false;
        dateSet ? displaystring = tz.dateTime("md") : displaystring = "0101";
        currentDisplayMode = MODE_SET;
        break;

    case EDIT_MODEL:
        messageindex = 0;
        currentDisplayMode = MODE_MESSAGE;
        overrideForBlank = true;
        break;
    }

    activeTarget = target;
    inEditMode = true;
    Serial.println("Entered set mode: " + String(activeTarget));
    updateDisplayValue();
}

void commitSetMode()
{
    if (!inEditMode)
        return;

    switch (activeTarget)
    {
    case EDIT_TIME:
    {
        int mm = tz.month();
        int dd = tz.day();
        // Update canonical time into tz/rtc
        tz.setTime(hr24, minutes, seconds, dd, mm, 2025);
        // rtc.adjust(DateTime(2025, 1, 1, hr24, minutes, seconds + 1));
        rtc.adjust(DateTime(2025, mm, dd, hr24, minutes, seconds));
        keepRTCinsync();
        break;
    }

    case EDIT_ALARM:
    {
        // Alarm is minute-granular; force seconds = 00
        uint8_t mn = minutes;
        uint8_t ss = 0;

        // Store as DateTime (date fields are dummy and ignored elsewhere)
        alarmDT = DateTime(2025, 1, 1, hr24, mn, ss);
        alarmSet = true;

        Serial.println("Committed alarm (HH:MM): " + String(hr24) + ":" + String(mn));
        rtc.setAlarm1(alarmDT, DS3231_A1_Hour);
        rtc.clearAlarm(1);
        rtc.disableAlarm(1);
        break;
    }

    case EDIT_DATE:
    {
        if (dateChanged)
        {
            int mm = displaystring.substring(0, 2).toInt(); // MM
            int dd = displaystring.substring(2, 4).toInt(); // DD
            // Get current time to set back with new MM/DD
            int h = tz.hour();
            if (h > 12)
                h -= 12; // Force AM if changing date for proper day rollover
            int m = tz.minute();
            int s = tz.second();
            dateSet = true;
            dateChanged = false;
            tz.setTime(h, m, s, dd, mm, 2025);
            rtc.adjust(DateTime(2025, mm, dd, h, m, s));
            Serial.println("Exit Date set: " + String(displaystring));
        }
        else
            Serial.println("Exiting DATE - no changes");
        break;
    }

    case EDIT_MODEL:
    {

        Model setmodel = static_cast<Model>(messageindex);
        Serial.println("Model will be set to " + String(modelToString(setmodel)));
        saveEEPROM(info.emulation, setmodel);
        currentDisplayMode = MODE_MESSAGE;
        messageindex = 8; // dashes
        updateDisplayValue();
        clearRTC();
        delay(2000);
        ESP.restart();
        delay(1000);
        break;
    }
    }

    // Reset edit state
    inEditMode = false;
    lastMinuteSeen = -1;
    setstring = "";
    currentDisplayMode = MODE_TIME;
    Serial.println("Exited set mode");    
    updateDisplayStateIfNeeded();
    updateDisplayValue();
    
}

void updateStrings()
{
    // Canonical string
    char buf[7];
    sprintf(buf, "%02d%02d%02d", hr24, minutes, seconds);
    setstring = String(buf);

    // AM/PM pin
    if (info.model != MODEL_GC_1092D)
        digitalWrite(pinAM, (hr24 >= 12) ? LOW : HIGH);

    // Display string
    uint8_t hrDisp = is24HourMode ? hr24 : ((hr24 % 12) == 0 ? 12 : hr24 % 12);
    sprintf(buf, "%02d%02d%02d", hrDisp, minutes, seconds);
    displaystring = String(buf);
    updateDisplayValue();
}
void gotTouchEvent()
{
    if (lastTouchActive != testingLower)
    {
        //Serial.println("Touch!");
        touchActive = !touchActive;
        testingLower = !testingLower;
        // Touch ISR will be inverted: Lower <--> Higher than the Threshold after ISR event is noticed
        touchInterruptSetThresholdDirection(testingLower);
    }
}

void handleTouchEvent() {
    #define GRACE_MS 50             // Time to wait after any switch change before evaluating states
    #define RELEASE_COOLDOWN_MS 500 // Ignore individual sw_5 and sw_6 for 500ms after D_555 ends
    bool computedTouchActive = false;
    static unsigned long lastChangeTime = 0;
    static unsigned long lastD555EndTime = 0;
    static bool prevComputed = true;
    static bool sw_6_changed_last = false; //track if sw_6 changed during this cycle

    if (info.emulation == WIFI)
    {
        // Start grace timer whenever any switch changes
        if (sw_2.changed || sw_5.changed || sw_6.changed) {
            if (sw_6.changed) sw_6_changed_last = true;
            lastChangeTime = millis();
        }
        bool D_555 = (sw_2.state && sw_5.state);
        if ((millis() - lastChangeTime) >= GRACE_MS) {  // After GRACE_MS, evaluate stable states
            bool Others = ((sw_6_changed_last && sw_6.state) || sw_5.state);

            if (D_555) {
                computedTouchActive = true; // D_555 wins
            } else {
                if ((millis() - lastD555EndTime) >= RELEASE_COOLDOWN_MS) {  // Check cooldown after D_555 release
                    computedTouchActive = Others;
                } else {
                    computedTouchActive = false; // Block Others during cooldown
                    if (Others) {
                    }
                }
            }
        } else {
            computedTouchActive = false; // Still in grace period
        }
        static bool wasD555 = false;
        if (!D_555 && wasD555) {
            lastD555EndTime = millis();
            sw_6_changed_last = false; 
        }
        wasD555 = D_555;
        if (computedTouchActive != prevComputed) {
            prevComputed = computedTouchActive;
        }
    }

    bool effectiveTouchActive = touchActive || computedTouchActive ;
    if (lastTouchActive != effectiveTouchActive) {
        lastTouchActive = effectiveTouchActive;
        
        if (effectiveTouchActive) {
            if (currentDisplayMode == MODE_TIME) {
                currentDisplayMode = MODE_DATE;
                Serial.println("Touch event: Show date");
                touchoverride = true;
                datecounter = (info.emulation == WIFI) ? showdatesec : 5;
            }
            else if (info.emulation == WIFI && currentDisplayMode == MODE_DATE) {
                manualOverride = true;
                overrideForBlank = true;
                currentDisplayMode = MODE_MESSAGE;
                messageindex = 9;
                Serial.println("Touch event: Blank display");
            }
            else if (info.emulation == WIFI && currentDisplayMode == MODE_MESSAGE) {
                manualOverride = true;
                overrideForBlank = false;
                currentDisplayMode = MODE_TIME;
                Serial.println("Touch event: Show time");
            }
        }
    }
}

void CheckSwitches()
{
    bool KA = digitalRead(pinKA);
    bool KB = digitalRead(pinKB);
    uint32_t now = micros();

    if (curdigit == 1)
    { // D6
        debounceSwitch(sw_1, KA, now);
        debounceSwitch(sw_2, KB, now);
    }
    else if (curdigit == 2)
    { // D5
        debounceSwitch(sw_3, KA, now);
        debounceSwitch(sw_4, KB, now);
    }
    else if (curdigit == 3)
    { // D4
        debounceSwitch(sw_5, KA, now);
        debounceSwitch(sw_6, KB, now);
    }
    else if (curdigit == 4)
    { // D3
        debounceSwitch(sw_7, KA, now);
    }

       

    // --- 60Hz presence check ---
    static bool lastRead = false;
    static uint32_t lastTransition = 0;
    bool raw = digitalRead(sw_HOLD_Pin);

    if (raw != lastRead)
    {
        lastRead = raw;
        lastTransition = now;
    }

    bool rawHz = ((now - lastTransition) > 100000);
    if (Disable_swHOLD)
    {
        rawHz = false;
    }
    debounceSwitch(sw_HOLD, rawHz, now);
}

bool debounceSwitch(DebouncedSwitch &sw, bool raw, uint32_t nowMicros)
{
    sw.changed = false;

    if (raw != sw.lastRead)
    {
        sw.lastRead = raw;
        sw.lastChange = nowMicros;
    }

    if ((nowMicros - sw.lastChange) > debounceDelay)
    {
        if (raw != sw.state)
        {

            sw.state = raw;
            sw.changed = true;
        }
    }

    return sw.changed;
}

void DispatchSwitches()
// sw_1 - Time Set
// sw_2 - Alarm Set / Date Set
// sw_3 - Hour / Month Increment
// sw_4 - Minute / Day Increment
// sw_5 - Snooze / Show Date
// sw_6 - Alarm Enable / Auto Date
// sw_7 - 12/24 Hour Mode
// sw_HOLD - Hold (1092 only)
{
    if ((sw_3.changed || sw_4.changed) && (sw_3.state && sw_4.state) && !inEditMode && !otaModeActive){
        ArduinoOTA_begin();
    }
    if (sw_1.changed && sw_1.state){
        ResetOn3xTimeSet();
    }
    if (info.emulation == MK5017){
        if (sw_1.changed && sw_1.state)
        {
            handleSetMode(EDIT_TIME);
        }

        if (sw_2.changed && sw_2.state)
        {
            if (info.model != MODEL_GC_1092D)
            {
                handleSetMode(EDIT_ALARM);
            }
            else
            {
                handleSetMode(EDIT_DATE);
            }
        }
        if  (sw_5.changed && (info.model != (MODEL_GC_1092D || MODEL_BOGUS)))
        {
            requestSnooze(); // state machine will handle transition
        }

        if (sw_6.changed)
        {
            if (info.model != MODEL_GC_1092D)
            {
                alarmEnabled = sw_6.state;
                if (!sw_6.state)
                    alarmToneStop();
                //Serial.println(alarmEnabled ? "alarmEnabled ON" : "alarmEnabled OFF");
            }
            else
            {        
                autoDate = !sw_6.state; 
                if (!sw_2.state){
                currentDisplayMode = MODE_TIME;   
                }
            }
        }

        if (sw_7.changed)
        {
            is24HourMode = sw_7.state; // toggle mode
            if (info.model != MODEL_GC_1092D)
                updateStrings(); // rebuild displaystring from canonical for set mode
            if (!sw_HOLD.state)
                timestring = tz.dateTime(is24HourMode ? "His" : "his");
            updateDisplayValue();
        }

        if (sw_HOLD.changed && sw_HOLD.state)
        {
            handleSetMode(EDIT_TIME);
        }
    }
    // Keep handling edit mode updates/exit
    if (inEditMode)
    {
        handleSetMode(activeTarget);
    }
}

EmulationInfo loadEEPROM()
{
    uint8_t e = EEPROM.read(EEPROM_ADDR_EMULATION);
    uint8_t md = EEPROM.read(EEPROM_ADDR_MODEL);

    Emulation emu = defaultEmulation;
    Model model = defaultModel;

    if (e < static_cast<uint8_t>(EmulationCount))
    {
        emu = static_cast<Emulation>(e);
    }

    if (md < static_cast<uint8_t>(ModelCount))
    {
        model = static_cast<Model>(md);
    }

    return EmulationInfo{emu, model};
}

void saveEEPROM(Emulation emu, Model model)
{
    Serial.println("saveEEPROM");
    uint8_t e = static_cast<uint8_t>(emu);
    uint8_t m = static_cast<uint8_t>(model);
    EEPROM.write(EEPROM_ADDR_EMULATION, e);
    EEPROM.write(EEPROM_ADDR_MODEL, m);
    EEPROM.commit();
}

void monitorAndRecoverNtp()
{
    static unsigned long lastCheck = 0;

    if (millis() - lastCheck < ntpinterval * 1000)
        return;
    lastCheck = millis();

    time_t nowTime = now();
    time_t lastNtp = lastNtpUpdateTime();

    if ((nowTime - lastNtp) > (ntpinterval * 3))
    {
        Serial.println("NTP stale. Attempting resync...");
        currentDisplayMode = MODE_MESSAGE;
        messageindex = 8; // dashes
        updateDisplayValue();
        if (!waitForSync(5))
        {
            Serial.println("Resync failed. Restarting WiFi...");

            WiFi.disconnect();
            WiFi.mode(WIFI_OFF);
            delay(1000);

            WiFi.mode(WIFI_STA);
            WiFi.begin();

            if (!waitForSync(10))
            {
                Serial.println("Still no NTP. Rebooting...");
                ESP.restart();
            }
            else
            {
                currentDisplayMode = MODE_TIME;
                Serial.println("NTP resynced successfully.");
            }
        }
    }
}

void checkOtaTimeout()
{
    if (otaModeActive && millis() - otaStartTime > otaTimeout)
    {
        Serial.println("OTA timeout exceeded. Reverting to STA mode...");
        WiFi.softAPdisconnect(true); // Shut down OTA AP
        if (info.emulation == WIFI){
            WiFi.mode(WIFI_STA);         // Return to station mode
            WiFi.begin();                // Reconnect using stored credentials
        }
        otaModeActive = false;       // Reset OTA mode flag
    }
}

void configModeCallback(WiFiManager *myWiFiManager)
{
    Serial.println("Entered setup mode");
    // Setup display
    currentDisplayMode = MODE_MESSAGE;
    messageindex = 6; // Setup
    updateDisplayValue();
    Serial.println(WiFi.softAPIP());
    // if you used auto generated SSID, print it
    Serial.println(myWiFiManager->getConfigPortalSSID());
}

void keepRTCinsync()
{
    DateTime rtcnow = rtc.now();
    DateTime BatteryDead = rtc.now() + TimeSpan(0, BatteryBackup, 0, 0);
    rtc.setAlarm2(BatteryDead, DS3231_A2_Hour);
    // rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    // rtc.disableAlarm(1);
    rtc.disableAlarm(2);
    // Set ezTime FROM RTC
    tz.setTime(rtcnow.hour(), rtcnow.minute(), rtcnow.second(),
               rtcnow.day(), rtcnow.month(), rtcnow.year());

    Serial.println("ezTime synced from RTC");
    Serial.println("RTC Time: " + String(rtcnow.hour()) + ':' + String(rtcnow.minute()) + ':' + String(rtcnow.second()));
    Serial.println("ezTime: " + String(tz.dateTime(is24HourMode ? "His" : "his")));

}

void setupConfigInit()
{
    // Startup switch check - H switch to change Mode or M switch for Safe mode
    delay(1000);
    for (int i = 0; i < 8; i++)
    {
        CheckSwitches(); // check sw_1 through sw_7
        delay(25);
    }

    info = loadEEPROM();
    if (info.emulation == MK5017 && sw_4.state)
    {
        // Disable sw_HOLD and clear RTC RAM
        Serial.println("Minute switch held - Clearing Model, disabling sw_HOLD, clearing RTC ram");
        Disable_swHOLD = true;
        clearRTC();
        saveEEPROM(info.emulation, MODEL_BOGUS);
        currentDisplayMode = MODE_MESSAGE;
        messageindex = 8; // dashes
        updateDisplayValue();
        delay(2000);
        ESP.restart();
        delay(1000);
    }

    if (info.emulation == MK5017 && sw_3.state)
    {
        saveEEPROM(WIFI, info.model);
        Serial.println("Emulation set to WIFI via hour advance switch. Rebooting...");
        currentDisplayMode = MODE_MESSAGE;
        messageindex = 8; // dashes
        updateDisplayValue();
        delay(2000);
        ESP.restart();
        delay(1000);
    }
    else if (info.emulation == WIFI && sw_3.state)
    {
        saveEEPROM(MK5017, info.model);
        Serial.println("Emulation set to MK5017 via hour advance switch. Rebooting...");
        currentDisplayMode = MODE_MESSAGE;
        messageindex = 8; // dashes
        updateDisplayValue();
        delay(2000);
        ESP.restart();
        delay(1000);
    }
    Serial.println("Emulation mode is " + String(info.emulation == MK5017 ? "MK5017" : "WIFI"));
    Serial.println("Clock model is " + String(modelToString(info.model)));
}

void setupEmulationInit()
{
    // Main setup section for MK5017
    if (info.emulation == MK5017)
    {
        if (info.model == MODEL_BOGUS) return; // Wait for model set
        // Disable eztime sync
        tz.setPosix(F("UTC"));
        setInterval(0);

        // Initialize RTC
        if (!rtc.begin())
        {
            Serial.println("RTC not responding");
        }
        else
        {
            Serial.println("RTC found");
            DateTime rtcnow = rtc.now();
            DateTime alarm2 = rtc.getAlarm2();
            uint32_t nowS = (uint32_t)rtcnow.hour() * 3600 + (uint32_t)rtcnow.minute() * 60 + (uint32_t)rtcnow.second();
            uint32_t alarmS = (uint32_t)alarm2.hour() * 3600 + (uint32_t)alarm2.minute() * 60 + (uint32_t)alarm2.second();
            if (nowS <= alarmS)
            {
                Serial.println("RTC had good data...seeding from RTC");
                tz.setTime(rtcnow.hour(), rtcnow.minute(), rtcnow.second(),
                           rtcnow.day(), rtcnow.month(), rtcnow.year());
                hr24 = tz.hour();
                alarmDT = rtc.getAlarm1();
                alarmEnabled = sw_6.state;
                if (info.model == MODEL_GC_1092D)
                {
                    autoDate = !sw_6.state;
                    dateSet = true;
                }
                alarmSet = true;
                inEditMode = false;
                lastMinuteSeen = -1;
                setstring = "";
                currentDisplayMode = MODE_TIME;
                updateDisplayStateIfNeeded();
            }
            else
            {
                Serial.println("RTC had invalid data. Reset time");
                rtc.adjust(DateTime(2025, 1, 1, 0, 0, 0));
                currentDisplayMode = MODE_MESSAGE;
                messageindex = 7; // eights
                //handleSetMode(EDIT_TIME);
            }
        }
    }
    // Main setup section for WIFI
    if (info.emulation == WIFI)
    {   
        Disable_swHOLD = true;
        currentDisplayMode = MODE_MESSAGE;
        messageindex = 4; // Start
        updateDisplayValue();
        tz.setPosix(LOCALTZ_POSIX);
        setServer(ntpserver);
        setInterval(ntpinterval);
        setDebug(ntpdebug);

        WiFiManager wifiManager;
        wifiManager.setHostname(wifihostname);
        WiFi.mode(WIFI_STA);
        // Optional: reset settings for testing
        // wifiManager.resetSettings();

        // Set callback when entering AP mode
        wifiManager.setAPCallback(configModeCallback);
        wifiManager.setConfigPortalTimeout(180);

        // Attempt to connect, fallback to AP if needed
        if (!wifiManager.autoConnect(wifisetupssid.c_str()))
        {
            Serial.println("failed to connect and hit timeout");
            ESP.restart();
            delay(1000);
        }

        Serial.println("connected...");
        currentDisplayMode = MODE_MESSAGE;
        messageindex = 5; // Conctd
        updateDisplayValue();
        delay(2500); // give connection time to stabilize
        waitForSync(30);
        currentDisplayMode = MODE_TIME;
        updateDisplayStateIfNeeded();
    }
    ArduinoOTA
        .onStart([]()
                    {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
    else  // U_SPIFFS
        type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type); })
        .onEnd([]()
                { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                    {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
    
}
void updateAlarm()
{
    switch (alarmState)
    {
    case ALARM_IDLE:
        if (alarmEnabled)
        {
            alarmState = ALARM_ARMED;
            Serial.println("ALARM_ARMED");
        }
        break;

    case ALARM_ARMED:
        if (!alarmEnabled)
        {
            alarmState = ALARM_IDLE;
            Serial.println("ALARM_IDLE");
        }
        else if (timeSeconds == alarmSeconds)
        {
            alarmState = ALARM_SOUNDING;
            Serial.println("ALARM_SOUNDING");
        }
        break;

    case ALARM_SOUNDING:
    {
        if (!alarmEnabled)
        {
            alarmToneStop();
            alarmState = ALARM_IDLE;
            break;
        }

        uint32_t elapsed = timeSeconds - alarmSeconds;
        if (elapsed >= (ALARM_MAX_MIN * 60))
        {
            alarmToneStop();
            Serial.println("Alarm reached max duration");
            alarmState = ALARM_IDLE;
        }
        break;
    }

    case ALARM_SNOOZE:
        if (!alarmEnabled)
        {
            alarmState = ALARM_IDLE;
        }
        else if (timeSeconds >= snoozeTargetS)
        {
            alarmState = ALARM_SOUNDING;
        }
        break;
    }
}

void requestSnooze()
{
    Serial.println("Snooze");
    if (alarmState == ALARM_SOUNDING)
    {
        snoozeTargetS = timeSeconds + (SNOOZE_MIN * 60);
        alarmToneStop();
        alarmState = ALARM_SNOOZE;
    }
}

void alarmToneDriver()
{
    if (alarmState != ALARM_SOUNDING)
    {
        alarmToneStop(); // ensure silence in all other states
        return;
    }

    // Only SOUNDING state reaches here
    if (topOfSecond)
    {
        alarmToneStart(); // start burst at second boundary
    }
    if (halfSecond)
    {
        alarmToneStop(); // stop burst at half-second
    }
}

void alarmToneInit(void)
{
    if (info.model != MODEL_GC_1092D) {
        rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(RMT_PIN, RMT_CH);
        cfg.clk_div = 80; // 1 µs ticks
        cfg.tx_config.idle_output_en = true;
        cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
        cfg.tx_config.carrier_en = false;
        cfg.tx_config.loop_en = false;
        rmt_config(&cfg);
        rmt_driver_install(cfg.channel, 0, 0);

        int idx = 0;
        for (int c = 0; c < CYCLES_PER_HALFSECOND; c++)
        {
            for (int i = 0; i < 6; i++)
            {
                pulse_items[idx].level0 = 1;
                pulse_items[idx].duration0 = 16;
                pulse_items[idx].level1 = 0;
                pulse_items[idx].duration1 = 532;
                idx++;
            }
            pulse_items[idx].level0 = 0;
            pulse_items[idx].duration0 = 96;
            pulse_items[idx].level1 = 0;
            pulse_items[idx].duration1 = 1;
            idx++;
        }
    } else {
        //For 1092D, set pin HIGH to enable 12/24 switch
        Serial.println("1092D Enable 12/24 Hour Switch");
        pinMode(RMT_PIN, OUTPUT);
        digitalWrite(RMT_PIN, HIGH);    
    }
}

void alarmToneStart(void)
{
    if (toneActive)
        return;
    Serial.println("AlarmStart");
    rmt_write_items(RMT_CH, pulse_items, TOTAL_ITEMS, false); // play once
    toneActive = true;
}

void alarmToneStop(void)
{
    if (!toneActive)
        return;
    Serial.println("AlarmStop");
    rmt_tx_stop(RMT_CH); // in case you want to cut it short
    toneActive = false;
}

void clearRTC()
{
    rtc.begin();
    DateTime alarm2Time = DateTime(2025, 1, 1, 00, 00, 00);
    rtc.setAlarm1(alarm2Time, DS3231_A1_Date);
    rtc.setAlarm2(alarm2Time, DS3231_A2_Date);
}

void ResetOn3xTimeSet() {
  static int timeSetCount = 0;
  static unsigned long startTime = 0;
  unsigned long now = millis();

  if (timeSetCount == 0) {
    startTime = now;
    timeSetCount = 1;
  } else if (now - startTime <= 5000) {
    timeSetCount++;
    Serial.println("timeSetCount: " + String(timeSetCount));
    if (timeSetCount >= 3) {
      Serial.println("Resetting ESP32...");
      delay(1000);
      ESP.restart();
    }
  } else {
    startTime = now;
    timeSetCount = 1;
  }
}

void ArduinoOTA_begin() {
   WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(wifiotassid.c_str(), wifiotapass.c_str());
    ArduinoOTA.begin();
    otaModeActive = true;
    otaStartTime = millis();
    Serial.println("OTA mode activated");
}
