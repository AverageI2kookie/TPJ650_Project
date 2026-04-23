// ============================================================
// Weather Tamagotchi — ESP32-S3
// Reads ambient temperature via thermistor (once per hour) and
// maps it to one of five seasons, each with its own animation,
// LED colour, and sound. Feed / train the pet via RFID-triggered
// minigames. Pet it with the capacitive touch pad. Full Wi-Fi
// dashboard served over WebSocket for live monitoring and control.
// ============================================================

#include "heatshrink_decoder.h"   // Decompresses the Bad Apple video stream
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>    // Non-blocking async HTTP + WebSocket server
#include <LittleFS.h>             // Filesystem for storing large RLE animation files
#include <SPI.h>
#include <MFRC522.h>              // RC522 RFID reader driver
#include <Adafruit_GFX.h>         // Graphics primitives (text, shapes)
#include <Adafruit_SharpMem.h>    // Sharp memory display driver (bit-bangs SPI)
#include <math.h>

// Sprite data compiled into flash (PROGMEM) — small enough to fit
#include "musicMelody.h"            // Bad Apple note/duration arrays
#include "idle_mord.h"              // Standing idle sprite frames
#include "idle_chair_mord_cycle.h"  // Chair idle sprite frames
#include "eating_mord.h"            // Eating animation frames
#include "weight_mord.h"            // Training animation frames

// Pixel colour values for the Sharp display (1-bit: black or white)
#define BLACK 0
#define WHITE 1

// ================= SHARP DISPLAY =================
// Software SPI on pins 10/11/12 — does not use the hardware SPI bus,
// so it coexists safely with the RC522 which is on different pins.
#define SHARP_SCK   10
#define SHARP_MOSI  11
#define SHARP_SS    12
Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 400, 240, 1000000);

// ================= WIFI =================
const char* WIFI_SSID = "PA3XO-NITRO17";
const char* WIFI_PASS = "p@3x0wifi";

// ================= INPUTS =================
#define TOUCH_PIN   14   // TTP223 capacitive touch sensor
#define VIB_PIN     13   // Vibration switch (normally HIGH, pulses LOW on shock)
#define TILT_PIN    6    // Ball tilt switch — used as shake counter input
#define THERM_PIN   4    // NTC thermistor ADC input

// ================= BUTTONS =================
// Active-LOW push buttons with internal pull-ups
#define BTN_SW5     5
#define BTN_SW8     8
#define BTN_SW9     9

// ================= RFID RC522 =================
// RC522 uses the hardware SPI bus. RST is GPIO3 — GPIO1 is the
// ESP32-S3 UART TX line and causes silent boot failures if used instead.
#define RC522_SS_PIN    21
#define RC522_SCK_PIN   47
#define RC522_MOSI_PIN  48
#define RC522_MISO_PIN  2
#define RC522_RST_PIN   3

MFRC522 rfid(RC522_SS_PIN, RC522_RST_PIN);

// UIDs of the two known RFID tokens
const char* RFID_CARD_UID = "35 C5 A6 AC";   // Card → eating (Simon Says) minigame
const char* RFID_FOB_UID  = "CD 1A 06 31";   // Fob  → shake/training minigame

// ================= OUTPUTS =================
#define MOTOR_PIN   7    // Vibration motor (digital on/off)
#define BUZZER_PIN  15   // Passive buzzer driven with tone()
#define RGB_R_PIN   18
#define RGB_G_PIN   17
#define RGB_B_PIN   16

// Common-anode RGB: LOW = on, HIGH = off
const bool RGB_COMMON_ANODE = true;

// ================= THERMISTOR =================
// Steinhart-Hart simplified (Beta) equation constants for a 10 kΩ NTC.
// Resistance is derived from the voltage divider formed with R_FIXED.
const float R_FIXED = 10000.0;   // Fixed resistor in voltage divider (Ω)
const float BETA    = 3950.0;    // Thermistor Beta coefficient
const float T0      = 298.15;    // Reference temperature (25 °C in Kelvin)
const float R0      = 10000.0;   // Thermistor resistance at T0 (Ω)

// ================= TEMPERATURE TARGET / TEST OVERRIDE =================
// The web dashboard slider can temporarily spoof the sensor reading,
// allowing season transitions to be tested without physically heating
// or cooling the device. The override expires after 10 seconds.
float gTempTargetC = 22.0;
float gTempTargetF = 71.6;

bool gTempOverrideActive = false;
float gTempOverrideC = 22.0;
uint32_t gTempOverrideLastMs = 0;
const uint32_t TEMP_OVERRIDE_TIMEOUT_MS = 10000;   // Override expires after 10 s

// Thermistor is only sampled once per hour when the override is inactive.
// This avoids unnecessary ADC reads and is sufficient for slow-changing ambient temp.
uint32_t gLastTempReadMs = 0;
const uint32_t TEMP_UPDATE_INTERVAL_MS = 3600000; // 1 hour

// ================= MODES =================
// Only Tamagotchi mode exists in this build.
enum DeviceMode {
  MODE_TAMAGOTCHI = 0
};

// All animation states the pet sprite can be in
enum AnimType {
  ANIM_IDLE = 0,
  ANIM_EATING = 1,
  ANIM_TRAINING = 2,
  ANIM_SEASON_TRANSITION = 3,   // One-shot intro animation when season changes
  ANIM_SEASON_IDLE = 4,         // Looping seasonal idle (streamed from LittleFS)
  ANIM_PET = 5,
  ANIM_SUCCESS = 6,
  ANIM_FAILED = 7
};

// Temperature-mapped season — drives animations and LED colour
enum SeasonType {
  SEASON_NORMAL = 0,
  SEASON_WINTER = 1,
  SEASON_RAIN = 2,
  SEASON_FALL = 3,
  SEASON_SUMMER = 4
};

// Tracks what the current RFID scan is waiting to trigger after the minigame
enum PendingActionType {
  ACTION_NONE = 0,
  ACTION_EAT = 1,
  ACTION_TRAIN = 2
};

// State machine for the 3-2-1 countdown and the active minigame phase
enum MiniGameState {
  MINIGAME_NONE = 0,
  MINIGAME_COUNTDOWN_3 = 1,
  MINIGAME_COUNTDOWN_2 = 2,
  MINIGAME_COUNTDOWN_1 = 3,
  MINIGAME_SHAKE = 4,          // Tilt-shake game (training / fob)
  MINIGAME_SIMON_WAIT = 5      // Simon Says prompt waiting for input (eating / card)
};

// The four possible Simon Says prompt types mapped to physical inputs
enum SimonPromptType {
  SIMON_LEFT = 0,    // SW9 (GPIO9)
  SIMON_MID = 1,     // SW5 (GPIO5)
  SIMON_RIGHT = 2,   // SW8 (GPIO8)
  SIMON_TOUCH = 3    // Touch pad (GPIO14)
};

DeviceMode gMode = MODE_TAMAGOTCHI;

// Tracks which animation frame the pet is currently displaying
struct SpriteAnimState {
  AnimType type = ANIM_IDLE;
  int currentFrame = 0;
  int loopsRemaining = -1;     // -1 = loop forever
  uint32_t lastFrameMs = 0;
  uint16_t frameDelay = 120;   // ms between frames
};

SpriteAnimState gAnim;

// ================= NORMAL IDLE CYCLING =================
// The idle animation alternates between a standing sprite and a chair sprite.
// Chair idle fires randomly with a cooldown to prevent it from appearing too often.
bool gUseChairIdle = false;
uint32_t gNextChairIdleEligibleMs = 0;
const uint32_t CHAIR_IDLE_MIN_GAP_MS = 45000;      // At least 45 s between chair idles
const uint8_t CHAIR_IDLE_CHANCE_PERCENT = 25;       // 25% chance after each standing loop

// ================= SEASONAL FS DATA =================
// Large seasonal animations live in LittleFS, not flash.
// All frames are 400×240 px, 1 bit per pixel = 50 bytes/row, 12 000 bytes/frame.
const int FS_FRAME_W = 400;
const int FS_FRAME_H = 240;
const int FS_BYTES_ROW = 50;
const int FS_BYTES_PER_FRAME = FS_FRAME_H * FS_BYTES_ROW;   // 12000

// Paths to RLE-compressed animation files on LittleFS
const char* FALL_TRANSITION_FILE   = "/fall_transition.rle";
const char* FALL_IDLE_FILE         = "/fall_idle.rle";
const char* RAIN_TRANSITION_FILE   = "/rain_transition.rle";
const char* RAIN_IDLE_FILE         = "/rain_idle.rle";
const char* SUMMER_TRANSITION_FILE = "/summer_transition.rle";
const char* SUMMER_IDLE_FILE       = "/summer_idle.rle";
const char* WINTER_TRANSITION_FILE = "/winter_transition.rle";
const char* WINTER_IDLE_FILE       = "/winter_idle.rle";
const char* PET_FILE               = "/pet_mord17.rle";
const char* SUCCESS_FILE           = "/success_mord2.rle";
const char* FAILED_FILE            = "/failed_mord2.rle";

// Frame counts must match what the RLE encoder produced for each file
const int FALL_TRANSITION_FRAME_COUNT   = 24;
const int FALL_IDLE_FRAME_COUNT         = 17;
const int RAIN_TRANSITION_FRAME_COUNT   = 37;
const int RAIN_IDLE_FRAME_COUNT         = 20;
const int SUMMER_TRANSITION_FRAME_COUNT = 20;
const int SUMMER_IDLE_FRAME_COUNT       = 12;
const int WINTER_TRANSITION_FRAME_COUNT = 33;
const int WINTER_IDLE_FRAME_COUNT       = 18;
const int PET_FRAME_COUNT               = 17;
const int SUCCESS_FRAME_COUNT           = 2;
const int FAILED_FRAME_COUNT            = 2;

SeasonType gCurrentSeason = SEASON_NORMAL;    // What the thermistor currently says
SeasonType gDisplayedSeason = SEASON_NORMAL;  // What the display is actually showing

// ================= WEATHER TRANSITION NOTIFICATION =================
// When a season change is detected, the motor runs and the LED blinks
// in the new season's colour for WEATHER_NOTIFY_DURATION_MS milliseconds.
bool gWeatherNotifyActive = false;
SeasonType gWeatherNotifySeason = SEASON_NORMAL;
uint32_t gWeatherNotifyUntilMs = 0;
uint32_t gWeatherBlinkLastMs = 0;
bool gWeatherBlinkOn = false;
const uint32_t WEATHER_NOTIFY_DURATION_MS = 5000;   // 5 s notification window
const uint32_t WEATHER_BLINK_INTERVAL_MS = 220;     // Blink every 220 ms

// ================= TIMING =================
bool gLastTouch = false;
uint32_t gLastTouchMs = 0;
bool gDashboardSensorsArmed = false;
uint32_t gDashboardEnteredMs = 0;
uint32_t gLastSeasonEvalMs = 0;   // Throttles season re-evaluation to once per second

// ================= SHAKE MINIGAME =================
PendingActionType gPendingAction = ACTION_NONE;
MiniGameState gMiniGameState = MINIGAME_NONE;
uint32_t gMiniGameStateStartMs = 0;
uint32_t gShakeWindowStartMs = 0;
uint32_t gLastTiltTriggerMs = 0;
int gShakeCount = 0;
const uint32_t COUNTDOWN_STEP_MS = 700;    // Time each countdown digit is shown
const uint32_t SHAKE_LABEL_MS = 900;
const uint32_t SHAKE_WINDOW_MS = 5000;     // 5 s to complete the shake game
const int REQUIRED_SHAKES = 10;            // Tilt transitions needed to succeed
const uint32_t SHAKE_DEBOUNCE_MS = 100;    // Ignores bounces faster than 100 ms
bool gLastTiltForCount = HIGH;

// Simon Says state — holds a randomised 5-prompt sequence
SimonPromptType gSimonSequence[5];
int gSimonPromptIndex = 0;                        // Current prompt index (0–4)
SimonPromptType gCurrentSimonPrompt = SIMON_LEFT;
uint32_t gSimonPromptStartMs = 0;
const int SIMON_TOTAL_PROMPTS = 5;
const uint32_t SIMON_PROMPT_WINDOW_MS = 3000;   // 3 s to respond to each prompt
bool gSimonWaitingRelease = false;               // True until inputs are released between prompts

// ================= AUDIO =================
// Bad Apple melody runs on Core 0 via a FreeRTOS task so it doesn't
// block the main loop running on Core 1.
volatile bool audioRunning = false;
TaskHandle_t melodyTaskHandle = NULL;
bool gToneStarted = false;

bool gMusicEnabled = true;
int gVolumeLevel = 2;   // 0 = mute, 1 = low, 2 = medium, 3 = high

// Season-change jingle also uses a FreeRTOS task to avoid blocking the display loop
volatile bool seasonMelodyRunning = false;
TaskHandle_t seasonMelodyTaskHandle = NULL;


// ================= EARLY FORWARD DECLARATIONS FOR WEATHER HELPERS =================
// These functions are used inside the weather helpers below but defined later —
// forward declarations resolve the compile-order issue.
void rgbOff();
void rgbRed();
void rgbGreen();
void rgbBlue();
void rgbYellow();
void rgbWhite();
void motorOn();
void motorOff();
void stopWeatherNotification();

// Maps each season to its designated steady LED colour
void setWeatherLedSteady(SeasonType season) {
  switch (season) {
    case SEASON_SUMMER: rgbRed(); break;
    case SEASON_FALL:   rgbYellow(); break;
    case SEASON_NORMAL: rgbGreen(); break;
    case SEASON_RAIN:   rgbBlue(); break;
    case SEASON_WINTER: rgbWhite(); break;
    default: rgbOff(); break;
  }
}

// Same mapping used during the blink phase of the weather notification
void applyWeatherBlinkColor(SeasonType season) {
  switch (season) {
    case SEASON_SUMMER: rgbRed(); break;
    case SEASON_FALL:   rgbYellow(); break;
    case SEASON_NORMAL: rgbGreen(); break;
    case SEASON_RAIN:   rgbBlue(); break;
    case SEASON_WINTER: rgbWhite(); break;
    default: rgbOff(); break;
  }
}

// Called when a season change is detected — starts the 5 s motor + LED blink alert
void startWeatherNotification(SeasonType season) {
  gWeatherNotifyActive = true;
  gWeatherNotifySeason = season;
  gWeatherNotifyUntilMs = millis() + WEATHER_NOTIFY_DURATION_MS;
  gWeatherBlinkLastMs = 0;
  gWeatherBlinkOn = false;
  motorOn();
}

// Called every loop iteration while a notification is active — handles the LED blink
void updateWeatherNotification() {
  if (!gWeatherNotifyActive) return;

  if (millis() >= gWeatherNotifyUntilMs) {
    stopWeatherNotification();
    return;
  }

  motorOn();

  if (millis() - gWeatherBlinkLastMs >= WEATHER_BLINK_INTERVAL_MS) {
    gWeatherBlinkLastMs = millis();
    gWeatherBlinkOn = !gWeatherBlinkOn;

    if (gWeatherBlinkOn) {
      applyWeatherBlinkColor(gWeatherNotifySeason);
    } else {
      rgbOff();
    }
  }
}

// Ends the notification — stops the motor and restores the steady season LED colour
void stopWeatherNotification() {
  gWeatherNotifyActive = false;
  motorOff();

  if (gMode == MODE_TAMAGOTCHI) {
    if (gAnim.type != ANIM_EATING && gAnim.type != ANIM_TRAINING) {
      setWeatherLedSteady(gDisplayedSeason);
    }
  }
}

// ================= BUTTON STATE =================
// Previous-frame readings for edge detection (HIGH → LOW = pressed)
bool gLastBtn5 = HIGH;
bool gLastBtn8 = HIGH;
bool gLastBtn9 = HIGH;
uint32_t gLastButtonMs = 0;   // Timestamp of last accepted press (global debounce)

// ================= RFID STATE =================
String gLastRFIDUID = "None";
String gLastRFIDType = "None";
uint32_t gLastRFIDMs = 0;   // Prevents re-scanning the same card within 2 s

// ================= SERVER =================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");   // WebSocket endpoint — browser connects here for live data

// ================= GLOBAL STATE =================
// Sensor readings and output states that are included in every WebSocket broadcast
String ledState = "off";
bool buzzerState = false;
bool motorState = false;
int gTouch = 0;
int gVib   = 1;
int gTilt  = 1;
int gTherm = 0;
float gTempC = 0.0;
float gTempF = 0.0;
String gLastEvent = "Boot";
uint32_t gLastEventMs = 0;
uint32_t gLastDisplayMs = 0;
uint32_t gLastBroadcastMs = 0;   // Throttles WebSocket broadcasts to every 300 ms
uint32_t gLastSerialMs = 0;      // Throttles Serial debug output

// ================= FUNCTION PROTOTYPES =================
void broadcastState();
void enterTamagotchiMode();
void handleTamagotchiRFID();
void drawAnimationFrame(const uint8_t* frames, int frameIndex);
void spriteStartIdle();
void spriteStartEating();
void spriteStartTraining();
void spriteStartPet();
void spriteStartSuccess();
void spriteStartFailed();
void spriteStartSeasonTransition(SeasonType season);
void spriteStartSeasonIdle(SeasonType season);
void spriteTick();
String readRFIDUID();
bool rfidHealthCheck(bool printVersion = false);
void hardResetRFID();
bool initRFIDRobust(bool printVersion = true);
const char* modeName(DeviceMode m);
const char* animName(AnimType a);
const char* seasonName(SeasonType s);
void setTempTargetC(float tempC);
float getEffectiveTempC();
void updateTempOverride();
SeasonType determineSeason(float tempC);
void updateSeasonState();
bool drawFrameFromRLE(const char* path, int frameIndex);
void drawTempOverlay();
void playSeasonChangeMelody();
void stopSeasonMelody();
void handleTouchPadPet();
void startShakeCountdown(PendingActionType action);
void updateShakeMiniGame();
void drawCountdownScreen(const char* text);
void drawSimonPrompt(SimonPromptType prompt);
void startSimonGame();
void beginNextSimonPrompt();
bool isSimonPromptTriggered(SimonPromptType prompt, bool btn5, bool btn8, bool btn9, bool touchActive);
void playPetTone();
void playSuccessTone();
void playFailedTone();

// WEATHER
void setWeatherLedSteady(SeasonType season);
void applyWeatherBlinkColor(SeasonType season);
void startWeatherNotification(SeasonType season);
void updateWeatherNotification();
void stopWeatherNotification();

// RGB + MOTOR (FIX FOR YOUR ERROR)
void rgbOff();
void rgbRed();
void rgbGreen();
void rgbBlue();
void rgbYellow();
void rgbWhite();
void motorOn();
void motorOff();

// ================= RGB HELPERS =================
// Single write function handles common-anode polarity inversion
void rgbWrite(bool rOn, bool gOn, bool bOn) {
  if (RGB_COMMON_ANODE) {
    digitalWrite(RGB_R_PIN, rOn ? LOW : HIGH);
    digitalWrite(RGB_G_PIN, gOn ? LOW : HIGH);
    digitalWrite(RGB_B_PIN, bOn ? LOW : HIGH);
  } else {
    digitalWrite(RGB_R_PIN, rOn ? HIGH : LOW);
    digitalWrite(RGB_G_PIN, gOn ? HIGH : LOW);
    digitalWrite(RGB_B_PIN, bOn ? HIGH : LOW);
  }
}

// Convenience wrappers — each also updates ledState for the dashboard JSON
void rgbOff()     { rgbWrite(false, false, false); ledState = "off"; }
void rgbRed()     { rgbWrite(true,  false, false); ledState = "red"; }
void rgbGreen()   { rgbWrite(false, true,  false); ledState = "green"; }
void rgbBlue()    { rgbWrite(false, false, true ); ledState = "blue"; }
void rgbYellow()  { rgbWrite(true,  true,  false); ledState = "yellow"; }
void rgbCyan()    { rgbWrite(false, true,  true ); ledState = "cyan"; }
void rgbMagenta() { rgbWrite(true,  false, true ); ledState = "magenta"; }
void rgbWhite()   { rgbWrite(true,  true,  true ); ledState = "white"; }

// ================= OUTPUT HELPERS =================
void motorOn() {
  digitalWrite(MOTOR_PIN, HIGH);
  motorState = true;
}

void motorOff() {
  digitalWrite(MOTOR_PIN, LOW);
  motorState = false;
}

// Short blocking pulse — used for one-shot haptic feedback
void motorPulse(uint16_t msOn = 80) {
  motorOn();
  delay(msOn);
  motorOff();
}

// buzzerOn/Off check audioRunning and seasonMelodyRunning before touching
// the pin so they don't clash with the FreeRTOS melody tasks
void buzzerOn() {
  if (!audioRunning && !seasonMelodyRunning) {
    tone(BUZZER_PIN, 1000);
    gToneStarted = true;
  }
  buzzerState = true;
}

void buzzerOff() {
  if (!audioRunning && !seasonMelodyRunning && gToneStarted) {
    noTone(BUZZER_PIN);
    gToneStarted = false;
  }
  buzzerState = false;
}

// Blocks the main loop for msOn ms — only used for short UI confirmation tones
void playToneBlocking(uint16_t freq, uint16_t msOn) {
  if (audioRunning || seasonMelodyRunning) return;
  tone(BUZZER_PIN, freq, msOn);
  gToneStarted = true;
  buzzerState = true;
  delay(msOn);
  noTone(BUZZER_PIN);
  gToneStarted = false;
  buzzerState = false;
}

void buzzerBeep(uint16_t msOn = 120) {
  if (!audioRunning && !seasonMelodyRunning) {
    playToneBlocking(1000, msOn);
  }
}

// Two-note rising tone played when the pet is touched
void playPetTone() {
  if (audioRunning || seasonMelodyRunning) return;
  tone(BUZZER_PIN, 1320, 80);
  delay(90);
  tone(BUZZER_PIN, 1480, 90);
  delay(100);
  noTone(BUZZER_PIN);
  gToneStarted = false;
  buzzerState = false;
}

// Ascending two-note fanfare repeated 3× on minigame success
void playSuccessTone() {
  if (audioRunning || seasonMelodyRunning) return;
  const int tonesArr[2] = {1568, 1760};
  for (int loop = 0; loop < 3; loop++) {
    for (int i = 0; i < 2; i++) {
      tone(BUZZER_PIN, tonesArr[i], 180);
      delay(190);
    }
    delay(40);
  }
  noTone(BUZZER_PIN);
  gToneStarted = false;
  buzzerState = false;
}

// Descending two-note tone repeated 3× on minigame failure
void playFailedTone() {
  if (audioRunning || seasonMelodyRunning) return;
  const int tonesArr[2] = {440, 349};
  for (int loop = 0; loop < 3; loop++) {
    for (int i = 0; i < 2; i++) {
      tone(BUZZER_PIN, tonesArr[i], 220);
      delay(230);
    }
    delay(30);
  }
  noTone(BUZZER_PIN);
  gToneStarted = false;
  buzzerState = false;
}

// Short neutral blip played when a Simon prompt appears on screen
void playPromptTone() {
  if (audioRunning || seasonMelodyRunning) return;
  tone(BUZZER_PIN, 980, 85);
  delay(95);
  noTone(BUZZER_PIN);
  buzzerState = false;
}

// Higher blip played when a Simon prompt is answered correctly
void playPositivePromptTone() {
  if (audioRunning || seasonMelodyRunning) return;
  tone(BUZZER_PIN, 1450, 110);
  delay(120);
  noTone(BUZZER_PIN);
  buzzerState = false;
}

// ================= BAD APPLE AUDIO =================
// Runs on Core 0 so the video decode running on Core 1 is not blocked.
// Iterates through note/duration arrays from musicMelody.h.
// Volume is implemented by shortening the active tone duration (duty-cycle trick).
void melodyTask(void* pvParameters) {
  for (int i = 0; i < BADAPPLESIZE; i++) {
    if (!audioRunning) break;

    int dur = BadAppleduration[i];

    if (!gMusicEnabled || gVolumeLevel == 0) {
      if (gToneStarted) {
        noTone(BUZZER_PIN);
        gToneStarted = false;
      }
      vTaskDelay(pdMS_TO_TICKS(dur));
      continue;
    }

    int noteIndex = BadApplenotes[i];
    int freq = note_frequencies[noteIndex];

    if (freq > 0) {
      int playDur = dur;
      if (gVolumeLevel == 1) playDur = dur / 3;
      else if (gVolumeLevel == 2) playDur = (dur * 2) / 3;

      tone(BUZZER_PIN, freq, playDur);
      gToneStarted = true;
    } else {
      if (gToneStarted) {
        noTone(BUZZER_PIN);
        gToneStarted = false;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(dur));
  }

  if (gToneStarted) {
    noTone(BUZZER_PIN);
    gToneStarted = false;
  }

  audioRunning = false;
  melodyTaskHandle = NULL;
  vTaskDelete(NULL);
}

void startAudio() {
  if (!gMusicEnabled) return;
  if (melodyTaskHandle != NULL) return;
  if (seasonMelodyRunning) return;
  audioRunning = true;
  xTaskCreatePinnedToCore(melodyTask, "melody", 4096, NULL, 1, &melodyTaskHandle, 0);
}

void stopAudio() {
  audioRunning = false;
  if (gToneStarted) {
    noTone(BUZZER_PIN);
    gToneStarted = false;
  }
}

// ================= SEASON CHANGE MELODY =================
// Four-note ascending jingle played as a FreeRTOS task so it doesn't
// stall the display or sensor loop while notes are playing.
void seasonMelodyTask(void* pvParameters) {
  const int notes[] = { 784, 988, 1175, 1568 };
  const int durs[]  = { 100, 100, 120, 180 };

  for (int i = 0; i < 4; i++) {
    if (!seasonMelodyRunning) break;
    if (gMusicEnabled && gVolumeLevel > 0) {
      int playDur = durs[i];
      if (gVolumeLevel == 1) playDur = durs[i] / 3;
      else if (gVolumeLevel == 2) playDur = (durs[i] * 2) / 3;
      tone(BUZZER_PIN, notes[i], playDur);
      gToneStarted = true;
      buzzerState = true;
    }
    vTaskDelay(pdMS_TO_TICKS(durs[i] + 20));
  }

  noTone(BUZZER_PIN);
  gToneStarted = false;
  buzzerState = false;
  seasonMelodyRunning = false;
  seasonMelodyTaskHandle = NULL;
  vTaskDelete(NULL);
}

void playSeasonChangeMelody() {
  if (audioRunning) return;
  if (seasonMelodyTaskHandle != NULL) return;
  seasonMelodyRunning = true;
  xTaskCreatePinnedToCore(seasonMelodyTask, "seasonMelody", 2048, NULL, 1, &seasonMelodyTaskHandle, 0);
}

void stopSeasonMelody() {
  seasonMelodyRunning = false;
  noTone(BUZZER_PIN);
  gToneStarted = false;
  buzzerState = false;
}

// ================= THERMISTOR / TEMP OVERRIDE =================
// Converts the 12-bit ADC reading to °C using the Beta equation.
// The voltage is first derived from the ADC count, then used to calculate
// thermistor resistance via the voltage divider formula, then converted to temperature.
void computeTemperature(int adc) {
  if (adc <= 0) {
    gTempC = -999;
    gTempF = -999;
    return;
  }
  if (adc >= 4095) {
    gTempC = 999;
    gTempF = 999;
    return;
  }

  float voltage = (adc / 4095.0f) * 3.3f;
  float resistance = R_FIXED * (voltage / (3.3f - voltage));

  float tempK = 1.0f / ((1.0f / T0) + (1.0f / BETA) * log(resistance / R0));
  gTempC = tempK - 273.15f;
  gTempF = (gTempC * 9.0f / 5.0f) + 32.0f;
}

// Called by the web dashboard slider — activates the 10 s temperature override
void setTempTargetC(float tempC) {
  if (tempC < -10.0) tempC = -10.0;
  if (tempC > 35.0) tempC = 35.0;

  gTempTargetC = tempC;
  gTempTargetF = (gTempTargetC * 9.0 / 5.0) + 32.0;

  // testing override
  gTempOverrideActive = true;
  gTempOverrideC = tempC;
  gTempOverrideLastMs = millis();
}

// Returns the override temperature if active, otherwise the real thermistor reading
float getEffectiveTempC() {
  if (gTempOverrideActive) return gTempOverrideC;
  return gTempC;
}

// Checked every loop — deactivates the override once 10 s have elapsed
void updateTempOverride() {
  if (gTempOverrideActive && (millis() - gTempOverrideLastMs >= TEMP_OVERRIDE_TIMEOUT_MS)) {
    gTempOverrideActive = false;
    gLastEvent = "Temp Override Off";
    broadcastState();
  }
}

// ================= SEASON LOGIC =================
// Maps temperature to a season using fixed thresholds.
SeasonType determineSeason(float tempC) {
  if (tempC <= 4.0)  return SEASON_WINTER;
  if (tempC <= 10.0) return SEASON_RAIN;
  if (tempC <= 16.0) return SEASON_FALL;
  if (tempC < 26.0)  return SEASON_NORMAL;
  return SEASON_SUMMER;
}

const char* seasonName(SeasonType s) {
  switch (s) {
    case SEASON_NORMAL: return "Normal";
    case SEASON_WINTER: return "Winter";
    case SEASON_RAIN: return "Rain";
    case SEASON_FALL: return "Fall";
    case SEASON_SUMMER: return "Summer";
    default: return "Unknown";
  }
}

// Runs once per second — compares current temperature to the displayed season
// and triggers the transition animation, LED alert, and jingle on any change.
void updateSeasonState() {
  if (millis() - gLastSeasonEvalMs < 1000) return;
  gLastSeasonEvalMs = millis();

  SeasonType newSeason = determineSeason(getEffectiveTempC());
  gCurrentSeason = newSeason;

  if (gMode != MODE_TAMAGOTCHI) return;
  if (gAnim.type == ANIM_EATING || gAnim.type == ANIM_TRAINING) return;

  if (newSeason != gDisplayedSeason) {
    gDisplayedSeason = newSeason;
    gLastEvent = String("Season -> ") + seasonName(newSeason);
    playSeasonChangeMelody();
    startWeatherNotification(newSeason);

    if (newSeason == SEASON_NORMAL) {
      spriteStartIdle();
    } else {
      spriteStartSeasonTransition(newSeason);
    }
    broadcastState();
  }
}

// ================= RFID =================
// Pulls RST low for 150 ms then releases it — forces a full hardware
// reset of the RC522 when the SPI health check fails.
void hardResetRFID() {
  pinMode(RC522_RST_PIN, OUTPUT);

  digitalWrite(RC522_RST_PIN, LOW);
  delay(150);

  digitalWrite(RC522_RST_PIN, HIGH);
  delay(300);
}

// Full recovery sequence: ends SPI, hard-resets RST, restarts SPI,
// re-inits the RC522, then reads VersionReg to confirm the chip is alive.
bool initRFIDRobust(bool printVersion) {
  SPI.end();
  delay(100);

  pinMode(RC522_SS_PIN, OUTPUT);
  digitalWrite(RC522_SS_PIN, HIGH);
  delay(20);

  hardResetRFID();

  SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);
  delay(100);

  rfid.PCD_Init();
  delay(100);

  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);

  if (printVersion) {
    Serial.printf("[RFID] VersionReg=0x%02X %s\n",
                  v,
                  (v == 0x00 || v == 0xFF) ? "(no response)" : "(OK)");
  }

  return (v != 0x00 && v != 0xFF);
}

// Formats the scanned card UID as a space-separated uppercase hex string
String readRFIDUID() {
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (i > 0) uid += " ";
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

// Reads VersionReg — 0x00 or 0xFF means the bus is not reaching the chip.
// Any other value means the module is alive (clones often return 0x88, etc.)
bool rfidHealthCheck(bool printVersion) {
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);

  if (printVersion) {
    Serial.printf("[RFID] VersionReg=0x%02X %s\n",
                  v,
                  (v == 0x00 || v == 0xFF) ? "(no response)" : "(OK)");
  }

  if (v == 0x00 || v == 0xFF) {
    return false;
  }

  return true;
}

// Called every loop — re-asserts SPI pins first (display.refresh() can reclaim
// the bus), health-checks the module, then scans for new cards.
// On a valid scan it identifies the UID and kicks off the appropriate minigame.
void handleTamagotchiRFID() {
  if (gMode != MODE_TAMAGOTCHI) return;
  if (millis() - gLastRFIDMs < 2000) return;   // 2 s cooldown between scans

  // Re-assert SPI to RC522 pins — display.refresh() reconfigures the shared bus
  SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);

  if (!rfidHealthCheck(false)) {
    Serial.println("[RFID] Module not responding — hard reset + reinit...");

    bool ok = false;
    for (int i = 0; i < 5; i++) {
      if (initRFIDRobust(true)) {
        ok = true;
        break;
      }
      delay(300);
    }

    if (!ok) {
      Serial.println("[RFID] Reinit failed — still no response.");
      gLastRFIDMs = millis();
    } else {
      Serial.println("[RFID] Reinit OK.");
    }
    return;
  }

  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  String uid = readRFIDUID();
  gLastRFIDUID = uid;

  if (uid == RFID_CARD_UID) {
    gLastRFIDType = "Card";
    gLastEvent = "Food Scan";
    playToneBlocking(1200, 140);
    startShakeCountdown(ACTION_EAT);    // Card → Simon Says → eating animation
  } else if (uid == RFID_FOB_UID) {
    gLastRFIDType = "Fob";
    gLastEvent = "Training Scan";
    playToneBlocking(1800, 180);
    startShakeCountdown(ACTION_TRAIN);  // Fob → shake game → training animation
  } else {
    gLastRFIDType = "Unknown";
    gLastEvent = "Unknown RFID";
    rgbMagenta();
    playToneBlocking(900, 120);
  }

  gLastRFIDMs = millis();

  Serial.print("[Tamagotchi RFID] ");
  Serial.print(gLastRFIDType);
  Serial.print(" UID=");
  Serial.println(gLastRFIDUID);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  broadcastState();
}

// ================= DRAW HELPERS =================
// Overlays the current temperature in the top-right corner of every frame.
// Prints "TEST" alongside it when the dashboard override is active.
void drawTempOverlay() {
  float shownTemp = getEffectiveTempC();

  display.fillRect(250, 0, 150, 18, WHITE);
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.setCursor(252, 5);
  display.print(shownTemp, 1);
  display.print("C");

  if (gTempOverrideActive) {
    display.setCursor(320, 5);
    display.print("TEST");
  }
}

// Renders a single frame from a PROGMEM sprite array (idle, eating, training).
// Clears the buffer, draws pixel-by-pixel from packed bits, overlays the
// temperature readout, then calls display.refresh().
void drawAnimationFrame(const uint8_t* frames, int frameIndex) {
  int frameOffset = frameIndex * SPRITE_FRAME_H * SPRITE_BYTES_ROW;

  display.clearDisplayBuffer();

  for (int row = 0; row < SPRITE_FRAME_H; row++) {
    for (int byteIdx = 0; byteIdx < SPRITE_BYTES_ROW; byteIdx++) {
      uint8_t b = pgm_read_byte(&frames[frameOffset + row * SPRITE_BYTES_ROW + byteIdx]);

      for (int bit = 0; bit < 8; bit++) {
        int col = byteIdx * 8 + bit;
        if (col >= SPRITE_FRAME_W) break;
        bool dark = (b >> (7 - bit)) & 1;
        display.drawPixel(col, row, dark ? BLACK : WHITE);
      }
    }
  }

  drawTempOverlay();
  display.refresh();
}

// Reads and decompresses a single frame from an RLE file on LittleFS.
// File format: "RLE1" magic | width(u16) | height(u16) | frameCount(u16) |
//              bytesPerFrame(u32) | offset table (frameCount × u32) |
//              payload: run-length triplets of runLen(u16) + value(u8).
// Returns false and logs an error if any read or validation step fails.
bool drawFrameFromRLE(const char* path, int frameIndex) {
  static uint8_t frameBuf[FS_BYTES_PER_FRAME];

  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.print("[RLE ERROR] Failed to open: ");
    Serial.println(path);
    return false;
  }

  uint8_t magic[4];
  if (f.read(magic, 4) != 4 || magic[0] != 'R' || magic[1] != 'L' || magic[2] != 'E' || magic[3] != '1') {
    Serial.print("[RLE ERROR] Bad header: ");
    Serial.println(path);
    f.close();
    return false;
  }

  uint16_t width = 0;
  uint16_t height = 0;
  uint16_t frameCount = 0;
  uint32_t rawFrameBytes = 0;

  if (f.read((uint8_t*)&width, 2) != 2 ||
      f.read((uint8_t*)&height, 2) != 2 ||
      f.read((uint8_t*)&frameCount, 2) != 2 ||
      f.read((uint8_t*)&rawFrameBytes, 4) != 4) {
    Serial.print("[RLE ERROR] Header read failed: ");
    Serial.println(path);
    f.close();
    return false;
  }

  if (width != FS_FRAME_W || height != FS_FRAME_H || rawFrameBytes != FS_BYTES_PER_FRAME) {
    Serial.print("[RLE ERROR] Unexpected dimensions in ");
    Serial.println(path);
    f.close();
    return false;
  }

  if (frameIndex < 0 || frameIndex >= frameCount) {
    Serial.print("[RLE ERROR] Frame out of range in ");
    Serial.print(path);
    Serial.print(" frame=");
    Serial.println(frameIndex);
    f.close();
    return false;
  }

  // Jump to the per-frame offset table entry for this frame index
  size_t headerSize = 4 + 2 + 2 + 2 + 4;
  size_t tablePos = headerSize + ((size_t)frameIndex * 4);

  if (!f.seek(tablePos, SeekSet)) {
    Serial.print("[RLE ERROR] Seek table failed: ");
    Serial.println(path);
    f.close();
    return false;
  }

  uint32_t startOffset = 0;
  uint32_t nextOffset = 0;
  if (f.read((uint8_t*)&startOffset, 4) != 4) {
    Serial.print("[RLE ERROR] Read start offset failed: ");
    Serial.println(path);
    f.close();
    return false;
  }

  // For the last frame, nextOffset is inferred from file size rather than the table
  if (frameIndex == frameCount - 1) {
    nextOffset = (uint32_t)(f.size() - (headerSize + ((size_t)frameCount * 4)));
  } else {
    if (f.read((uint8_t*)&nextOffset, 4) != 4) {
      Serial.print("[RLE ERROR] Read next offset failed: ");
      Serial.println(path);
      f.close();
      return false;
    }
  }

  size_t payloadBase = headerSize + ((size_t)frameCount * 4);
  size_t framePos = payloadBase + startOffset;
  size_t frameCompressedSize = nextOffset - startOffset;

  if (!f.seek(framePos, SeekSet)) {
    Serial.print("[RLE ERROR] Seek frame failed: ");
    Serial.println(path);
    f.close();
    return false;
  }

  // Decompress run-length encoded triplets into frameBuf
  size_t outPos = 0;
  size_t consumed = 0;

  while (consumed < frameCompressedSize && outPos < FS_BYTES_PER_FRAME) {
    uint16_t runLen = 0;
    uint8_t value = 0;

    if (f.read((uint8_t*)&runLen, 2) != 2 || f.read(&value, 1) != 1) {
      Serial.print("[RLE ERROR] Compressed read failed: ");
      Serial.println(path);
      f.close();
      return false;
    }

    consumed += 3;

    for (uint16_t i = 0; i < runLen && outPos < FS_BYTES_PER_FRAME; i++) {
      frameBuf[outPos++] = value;
    }
  }

  f.close();

  if (outPos != FS_BYTES_PER_FRAME) {
    Serial.print("[RLE ERROR] Decompressed size mismatch: ");
    Serial.print(path);
    Serial.print(" got=");
    Serial.print(outPos);
    Serial.print(" expected=");
    Serial.println(FS_BYTES_PER_FRAME);
    return false;
  }

  // Write decompressed pixels to the display buffer then refresh
  display.clearDisplayBuffer();

  for (int row = 0; row < FS_FRAME_H; row++) {
    for (int byteIdx = 0; byteIdx < FS_BYTES_ROW; byteIdx++) {
      uint8_t b = frameBuf[row * FS_BYTES_ROW + byteIdx];

      for (int bit = 0; bit < 8; bit++) {
        int col = byteIdx * 8 + bit;
        if (col >= FS_FRAME_W) break;
        bool dark = (b >> (7 - bit)) & 1;
        display.drawPixel(col, row, dark ? BLACK : WHITE);
      }
    }
  }

  drawTempOverlay();
  display.refresh();
  return true;
}

// ================= SPRITE SYSTEM =================
const uint16_t IDLE_DELAY_MS = 120;
const uint16_t EATING_DELAY_MS = 70;
const uint16_t TRAINING_DELAY_MS = 80;
const uint16_t SEASON_TRANSITION_DELAY_MS = 90;
const uint16_t SEASON_IDLE_DELAY_MS = 120;

// Starts idle animation — chooses standing or chair based on gUseChairIdle.
// Also restores the steady season LED colour if no notification is running.
void spriteStartIdle() {
  gAnim.type = ANIM_IDLE;
  gAnim.currentFrame = 0;
  gAnim.loopsRemaining = 1;
  gAnim.lastFrameMs = millis();
  gAnim.frameDelay = IDLE_DELAY_MS;

  if (!gWeatherNotifyActive) {
    setWeatherLedSteady(gDisplayedSeason);
  }

  if (gUseChairIdle) {
    drawAnimationFrame(idleChairMordFrames, 0);
  } else {
    drawAnimationFrame(idleMordFrames, 0);
  }
}

void spriteStartEating() {
  gAnim.type = ANIM_EATING;
  gAnim.currentFrame = 0;
  gAnim.loopsRemaining = 2;
  gAnim.lastFrameMs = millis();
  gAnim.frameDelay = EATING_DELAY_MS;
  drawAnimationFrame(eatingMordFrames, 0);
}

void spriteStartTraining() {
  gAnim.type = ANIM_TRAINING;
  gAnim.currentFrame = 0;
  gAnim.loopsRemaining = 2;
  gAnim.lastFrameMs = millis();
  gAnim.frameDelay = TRAINING_DELAY_MS;
  drawAnimationFrame(trainingMordFrames, 0);
}

void spriteStartPet() {
  gAnim.type = ANIM_PET;
  gAnim.currentFrame = 0;
  gAnim.loopsRemaining = 1;
  gAnim.lastFrameMs = millis();
  gAnim.frameDelay = 60;
  gLastEvent = "Pet";
  drawFrameFromRLE(PET_FILE, 0);
  playPetTone();
}

void spriteStartSuccess() {
  gAnim.type = ANIM_SUCCESS;
  gAnim.currentFrame = 0;
  gAnim.loopsRemaining = 3;
  gAnim.lastFrameMs = millis();
  gAnim.frameDelay = 260;
  drawFrameFromRLE(SUCCESS_FILE, 0);
  playSuccessTone();
}

void spriteStartFailed() {
  gAnim.type = ANIM_FAILED;
  gAnim.currentFrame = 0;
  gAnim.loopsRemaining = 3;
  gAnim.lastFrameMs = millis();
  gAnim.frameDelay = 260;
  drawFrameFromRLE(FAILED_FILE, 0);
  playFailedTone();
}

// One-shot transition animation shown when the season first changes
void spriteStartSeasonTransition(SeasonType season) {
  gAnim.type = ANIM_SEASON_TRANSITION;
  gAnim.currentFrame = 0;
  gAnim.loopsRemaining = 1;
  gAnim.lastFrameMs = millis();
  gAnim.frameDelay = SEASON_TRANSITION_DELAY_MS;

  switch (season) {
    case SEASON_WINTER: drawFrameFromRLE(WINTER_TRANSITION_FILE, 0); break;
    case SEASON_RAIN:   drawFrameFromRLE(RAIN_TRANSITION_FILE, 0); break;
    case SEASON_FALL:   drawFrameFromRLE(FALL_TRANSITION_FILE, 0); break;
    case SEASON_SUMMER: drawFrameFromRLE(SUMMER_TRANSITION_FILE, 0); break;
    default: spriteStartIdle(); break;
  }
}

// Looping seasonal idle — plays indefinitely until the season changes again
void spriteStartSeasonIdle(SeasonType season) {
  gAnim.type = ANIM_SEASON_IDLE;
  gAnim.currentFrame = 0;
  gAnim.loopsRemaining = -1;
  gAnim.lastFrameMs = millis();
  gAnim.frameDelay = SEASON_IDLE_DELAY_MS;

  if (!gWeatherNotifyActive) {
    setWeatherLedSteady(season);
  }

  switch (season) {
    case SEASON_WINTER: drawFrameFromRLE(WINTER_IDLE_FILE, 0); break;
    case SEASON_RAIN:   drawFrameFromRLE(RAIN_IDLE_FILE, 0); break;
    case SEASON_FALL:   drawFrameFromRLE(FALL_IDLE_FILE, 0); break;
    case SEASON_SUMMER: drawFrameFromRLE(SUMMER_IDLE_FILE, 0); break;
    default: spriteStartIdle(); break;
  }
}

// Advances the animation by one frame if enough time has passed.
// Handles loop-completion transitions: eating/training → success,
// transition → season idle, pet/success/failed → back to idle or season idle.
void spriteTick() {
  if (gMode != MODE_TAMAGOTCHI) return;

  uint32_t now = millis();
  if (now - gAnim.lastFrameMs < gAnim.frameDelay) return;
  gAnim.lastFrameMs = now;

  const uint8_t* frameData = nullptr;
  int frameCount = 0;
  bool useFS = false;
  const char* fsPath = nullptr;

  if (gAnim.type == ANIM_EATING) {
    frameData = eatingMordFrames;
    frameCount = EATING_FRAME_COUNT;
  } else if (gAnim.type == ANIM_TRAINING) {
    frameData = trainingMordFrames;
    frameCount = TRAINING_FRAME_COUNT;
  } else if (gAnim.type == ANIM_IDLE) {
    if (gUseChairIdle) {
      frameData = idleChairMordFrames;
      frameCount = IDLE_CHAIR_FRAME_COUNT;
    } else {
      frameData = idleMordFrames;
      frameCount = IDLE_FRAME_COUNT;
    }
  } else if (gAnim.type == ANIM_PET) {
    useFS = true;
    fsPath = PET_FILE;
    frameCount = PET_FRAME_COUNT;
  } else if (gAnim.type == ANIM_SUCCESS) {
    useFS = true;
    fsPath = SUCCESS_FILE;
    frameCount = SUCCESS_FRAME_COUNT;
  } else if (gAnim.type == ANIM_FAILED) {
    useFS = true;
    fsPath = FAILED_FILE;
    frameCount = FAILED_FRAME_COUNT;
  } else if (gAnim.type == ANIM_SEASON_TRANSITION) {
    useFS = true;
    switch (gDisplayedSeason) {
      case SEASON_WINTER: fsPath = WINTER_TRANSITION_FILE; frameCount = WINTER_TRANSITION_FRAME_COUNT; break;
      case SEASON_RAIN:   fsPath = RAIN_TRANSITION_FILE;   frameCount = RAIN_TRANSITION_FRAME_COUNT; break;
      case SEASON_FALL:   fsPath = FALL_TRANSITION_FILE;   frameCount = FALL_TRANSITION_FRAME_COUNT; break;
      case SEASON_SUMMER: fsPath = SUMMER_TRANSITION_FILE; frameCount = SUMMER_TRANSITION_FRAME_COUNT; break;
      default: spriteStartIdle(); return;
    }
  } else if (gAnim.type == ANIM_SEASON_IDLE) {
    useFS = true;
    switch (gDisplayedSeason) {
      case SEASON_WINTER: fsPath = WINTER_IDLE_FILE; frameCount = WINTER_IDLE_FRAME_COUNT; break;
      case SEASON_RAIN:   fsPath = RAIN_IDLE_FILE;   frameCount = RAIN_IDLE_FRAME_COUNT; break;
      case SEASON_FALL:   fsPath = FALL_IDLE_FILE;   frameCount = FALL_IDLE_FRAME_COUNT; break;
      case SEASON_SUMMER: fsPath = SUMMER_IDLE_FILE; frameCount = SUMMER_IDLE_FRAME_COUNT; break;
      default: spriteStartIdle(); return;
    }
  }

  gAnim.currentFrame++;

  if (gAnim.currentFrame >= frameCount) {
    gAnim.currentFrame = 0;

    if (gAnim.loopsRemaining > 0) {
      gAnim.loopsRemaining--;

      if (gAnim.loopsRemaining == 0) {
        if (gAnim.type == ANIM_IDLE) {
          // After each standing idle loop, randomly decide whether to switch to chair idle
          if (gUseChairIdle) {
            gUseChairIdle = false;
          } else if (millis() >= gNextChairIdleEligibleMs &&
                     random(100) < CHAIR_IDLE_CHANCE_PERCENT) {
            gUseChairIdle = true;
            gNextChairIdleEligibleMs = millis() + CHAIR_IDLE_MIN_GAP_MS;
          } else {
            gUseChairIdle = false;
          }
          spriteStartIdle();
        } else if (gAnim.type == ANIM_EATING) {
          spriteStartSuccess();
        } else if (gAnim.type == ANIM_TRAINING) {
          spriteStartSuccess();
        } else if (gAnim.type == ANIM_PET || gAnim.type == ANIM_SUCCESS || gAnim.type == ANIM_FAILED) {
          if (gDisplayedSeason == SEASON_NORMAL) {
            spriteStartIdle();
          } else {
            spriteStartSeasonIdle(gDisplayedSeason);
          }
        } else if (gAnim.type == ANIM_SEASON_TRANSITION) {
          spriteStartSeasonIdle(gDisplayedSeason);
        }
        return;
      }
    }
  }

  if (useFS) {
    drawFrameFromRLE(fsPath, gAnim.currentFrame);
  } else {
    drawAnimationFrame(frameData, gAnim.currentFrame);
  }
}

// ================= LABEL HELPERS =================
const char* modeName(DeviceMode m) {
  switch (m) {
    case MODE_TAMAGOTCHI: return "Tamagotchi";
    default: return "Unknown";
  }
}

const char* animName(AnimType a) {
  switch (a) {
    case ANIM_IDLE: return "Idle";
    case ANIM_EATING: return "Eating";
    case ANIM_TRAINING: return "Training";
    case ANIM_SEASON_TRANSITION: return "Season Transition";
    case ANIM_SEASON_IDLE: return "Season Idle";
    case ANIM_PET: return "Pet";
    case ANIM_SUCCESS: return "Success";
    case ANIM_FAILED: return "Failed";
    default: return "Unknown";
  }
}

// ================= HTML =================
// Entire dashboard UI stored in flash as a raw string literal.
// It connects back to the ESP32 via WebSocket on /ws and updates
// all sensor values in real time without page reloads.
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>IoT platform</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root{
      --bg:#000000;
      --panel:#020402;
      --panel2:#041004;
      --text:#00ff9c;
      --muted:#00aa66;
      --border:#003322;
      --shadow:0 0 20px rgba(0,255,120,.15);
    }
    *{ box-sizing:border-box; font-family:"Courier New", monospace; }
    body{
      margin:0;
      background:radial-gradient(circle at center, #001a00 0%, #000000 80%);
      color:var(--text);
      overflow-x:hidden;
    }
    .wrap{ max-width:1100px; margin:24px auto; padding:0 16px; }
    .hero{
      background:linear-gradient(180deg,#001100,#000000);
      border:1px solid var(--border);
      border-radius:12px;
      padding:18px;
      box-shadow:var(--shadow);
      margin-bottom:18px;
    }
    .hero h1{
      margin:0;
      font-size:26px;
      color:#00ff9c;
      text-shadow:0 0 8px #00ff9c;
    }
    .sub{ color:var(--muted); font-size:13px; }
    .grid{ display:grid; grid-template-columns:repeat(12,1fr); gap:14px; }
    .card{
      background:linear-gradient(180deg,var(--panel),var(--panel2));
      border:1px solid var(--border);
      border-radius:10px;
      padding:16px;
      box-shadow:var(--shadow);
    }
    .span-4{grid-column:span 4}
    .span-6{grid-column:span 6}
    .span-8{grid-column:span 8}
    .span-12{grid-column:span 12}
    @media(max-width:900px){
      .span-4,.span-6,.span-8,.span-12{grid-column:span 12}
    }
    .label{ color:var(--muted); font-size:12px; margin-bottom:6px; }
    .value{ font-size:26px; font-weight:bold; text-shadow:0 0 6px #00ff9c; }
    .small{ color:var(--muted); font-size:12px; }
    .status{
      display:inline-block;
      padding:5px 10px;
      border-radius:999px;
      font-size:11px;
      font-weight:bold;
      background:#001100;
      border:1px solid var(--border);
    }
    .ok{color:#00ff9c}
    .bad{color:#003300}
    .toolbar{ display:flex; flex-wrap:wrap; gap:8px; margin-top:10px; }
    button{
      border:1px solid #00ff9c;
      border-radius:8px;
      padding:10px 12px;
      background:black;
      color:#00ff9c;
      font-weight:bold;
      cursor:pointer;
      transition:all 0.2s ease;
    }
    button:hover{
      background:#00ff9c;
      color:black;
      box-shadow:0 0 10px #00ff9c;
    }
    .meter{
      width:100%;
      height:12px;
      border-radius:999px;
      background:#001100;
      border:1px solid var(--border);
      overflow:hidden;
      margin-top:8px;
    }
    .fill{
      height:100%;
      width:0%;
      background:linear-gradient(90deg,#00ff9c,#00cc66);
      box-shadow:0 0 10px #00ff9c;
      transition:width .2s ease;
    }
    .diag{ display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:6px; }
    .diagRow{
      background:rgba(0,255,100,.03);
      border:1px solid var(--border);
      border-radius:8px;
      padding:10px;
    }
    .event{
      font-size:16px;
      font-weight:bold;
      color:#00ff9c;
      text-shadow:0 0 6px #00ff9c;
    }
    .footer{
      color:#004422;
      text-align:center;
      font-size:11px;
      margin-top:16px;
    }
    .modeBtn{
      width:100%;
      padding:14px;
      font-size:16px;
      border-radius:10px;
      border:1px solid #00ff9c;
      background:black;
      color:#00ff9c;
      cursor:pointer;
      margin-bottom:14px;
    }
    .modeBtn:hover{
      background:#00ff9c;
      color:black;
      box-shadow:0 0 15px #00ff9c;
    }
    input[type="range"]{ accent-color:#00ff9c; width:100%; margin-top:12px; }
  </style>
</head>
<body>
<div class="wrap">
  <div class="hero">
    <h1>IoT platform</h1>
    <div class="sub">Real-time diagnostics and control</div>
    <div class="toolbar" style="margin-top:14px;">
      <span class="status" id="wsStatus">Connecting…</span>
      <span class="status">IP: <span id="ip">--</span></span>
      <span class="status">UPTIME: <span id="uptime">0</span> ms</span>
      <span class="status">MODE: <span id="modeVal">--</span></span>
      <span class="status">SEASON: <span id="seasonVal">--</span></span>
      <span class="status">TEST: <span id="testVal">OFF</span></span>
      <span class="status">MUSIC: <span id="musicVal">--</span></span>
      <span class="status">VOL: <span id="volumeVal">--</span></span>
      <span class="status">RFID: <span id="rfidTypeVal">--</span></span>
      <span class="status">ANIM: <span id="animVal">--</span></span>
    </div>
  </div>


  <div class="grid">
    <div class="card span-4">
      <div class="label">TOUCH SENSOR (GPIO14)</div>
      <div class="value" id="touchVal">0</div>
      <div class="small">TTP223 INPUT STATE</div>
    </div>

    <div class="card span-4">
      <div class="label">VIBRATION SWITCH (GPIO13)</div>
      <div class="value" id="vibVal">1</div>
      <div class="small">BLUE LED + MOTOR + BUZZER</div>
    </div>

    <div class="card span-4">
      <div class="label">BALL TILT SWITCH (GPIO6)</div>
      <div class="value" id="tiltVal">1</div>
      <div class="small">RED LED + MOTOR ONLY</div>
    </div>

    <div class="card span-8">
      <div class="label">TEMPERATURE (GPIO4)</div>
      <div class="value"><span id="tempC">0.0</span> C / <span id="tempF">0.0</span> F</div>
      <div class="small">THERMISTOR TEMPERATURE</div>
      <div class="meter"><div class="fill" id="thermFill"></div></div>
    </div>

    <div class="card span-4">
      <div class="label">LAST EVENT</div>
      <div class="event" id="eventVal">Boot</div>
      <div class="small">LATEST SENSOR OR ACTION</div>
    </div>

    <div class="card span-8">
      <div class="label">TEMPERATURE TARGET / TEST OVERRIDE</div>
      <div class="value"><span id="tempTargetC">22.0</span> C / <span id="tempTargetF">71.6</span> F</div>
      <div class="small">MOVING THE SLIDER OVERRIDES THE THERMISTOR FOR 10 SECONDS</div>
      <input type="range" id="tempSlider" min="-10" max="35" step="0.5" value="22" oninput="tempSliderChanged(this.value)">
    </div>

    <div class="card span-6">
      <div class="label">BUTTON MAP</div>
      <div class="small">TAMAGOTCHI MODE:</div>
            <div class="small">RFID CARD = EATING</div>
      <div class="small">RFID FOB = TRAINING</div>
      <div class="small" style="margin-top:10px;">TOUCH PAD = PET</div>
    </div>

    <div class="card span-6">
      <div class="label">RFID STATUS</div>
      <div class="value" id="rfidTypeCard">None</div>
      <div class="small">LAST TYPE: <span id="rfidTypeText">None</span></div>
      <div class="small">LAST UID: <span id="rfidUidVal">None</span></div>
    </div>

    <div class="card span-6">
      <div class="label">RGB LED CONTROL</div>
      <div class="value" id="ledVal">off</div>
      <div class="toolbar">
        <button onclick="sendCmd('led:red')">RED</button>
        <button onclick="sendCmd('led:green')">GREEN</button>
        <button onclick="sendCmd('led:blue')">BLUE</button>
        <button onclick="sendCmd('led:off')">OFF</button>
      </div>
    </div>

    <div class="card span-6">
      <div class="label">BUZZER & MOTOR CONTROL</div>
      <div class="diag">
        <div class="diagRow">
          <div class="small">BUZZER</div>
          <div class="value" id="buzzerVal">off</div>
          <div class="toolbar">
            <button onclick="sendCmd('buzzer:on')">ON</button>
            <button onclick="sendCmd('buzzer:off')">OFF</button>
          </div>
        </div>
        <div class="diagRow">
          <div class="small">MOTOR</div>
          <div class="value" id="motorVal">off</div>
          <div class="toolbar">
            <button onclick="sendCmd('motor:on')">ON</button>
            <button onclick="sendCmd('motor:off')">OFF</button>
          </div>
        </div>
      </div>
    </div>
  </div>

  <div class="footer">Weather Tamagotchi</div>
</div>

<script>
  let ws;

  function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }

  function sendCmd(cmd) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(cmd);
  }

  // Sends a tempTarget: command on every slider input event.
  // The ESP32 applies it as a 10 s override; the UI syncs on the next broadcast.
  function tempSliderChanged(val) {
    const c = parseFloat(val);
    const f = (c * 9 / 5) + 32;
    setText("tempTargetC", c.toFixed(1));
    setText("tempTargetF", f.toFixed(1));
    sendCmd("tempTarget:" + c.toFixed(1));
  }

  // Receives the JSON state broadcast and updates every element on the page
  function updateUI(d) {
    setText("ip", d.ip);
    setText("uptime", d.uptime);
    setText("touchVal", d.touch);
    setText("vibVal", d.vib);
    setText("tiltVal", d.tilt);
    setText("tempC", d.tempC);
    setText("tempF", d.tempF);
    setText("tempTargetC", d.tempTargetC);
    setText("tempTargetF", d.tempTargetF);
    setText("eventVal", d.event);
    setText("ledVal", d.led);
    setText("buzzerVal", d.buzzer ? "on" : "off");
    setText("motorVal", d.motor ? "on" : "off");
    setText("modeVal", d.modeName || d.mode);
    setText("seasonVal", d.season);
    setText("testVal", d.tempOverrideActive ? "ON" : "OFF");
    setText("musicVal", d.musicEnabled ? "ON" : "OFF");
    setText("volumeVal", d.volumeLevel);
    setText("rfidTypeVal", d.rfidType);
    setText("rfidTypeCard", d.rfidType);
    setText("rfidTypeText", d.rfidType);
    setText("rfidUidVal", d.rfidUid);
    setText("animVal", d.anim);

    const pct = Math.max(0, Math.min(100, Math.round((d.therm / 4095) * 100)));
    document.getElementById("thermFill").style.width = pct + "%";

    // Only sync the slider if the user isn't currently dragging it
    const slider = document.getElementById("tempSlider");
    if (slider && document.activeElement !== slider) slider.value = d.tempTargetC;
  }

  // Connects to the WebSocket and auto-reconnects after 1.5 s if the connection drops
  function connectWS() {
    ws = new WebSocket(`ws://${location.host}/ws`);

    ws.onopen = () => {
      const e = document.getElementById("wsStatus");
      e.textContent = "LIVE CONNECTED";
      e.classList.add("ok");
      e.classList.remove("bad");
    };

    ws.onclose = () => {
      const e = document.getElementById("wsStatus");
      e.textContent = "DISCONNECTED";
      e.classList.remove("ok");
      e.classList.add("bad");
      setTimeout(connectWS, 1500);
    };

    ws.onmessage = (event) => {
      try { updateUI(JSON.parse(event.data)); } catch (e) {}
    };
  }

  connectWS();
</script>
</body>
</html>
)rawliteral";

// ================= JSON =================
// Builds the state JSON string broadcast to all connected WebSocket clients.
// Called on every significant event and also on the 300 ms timer in the main loop.
String makeJson() {
  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"uptime\":" + String(millis()) + ",";
  json += "\"touch\":" + String(gTouch) + ",";
  json += "\"vib\":" + String(gVib) + ",";
  json += "\"tilt\":" + String(gTilt) + ",";
  json += "\"therm\":" + String(gTherm) + ",";
  json += "\"tempC\":" + String(gTempC, 1) + ",";
  json += "\"tempF\":" + String(gTempF, 1) + ",";
  json += "\"tempTargetC\":" + String(gTempTargetC, 1) + ",";
  json += "\"tempTargetF\":" + String(gTempTargetF, 1) + ",";
  json += "\"tempOverrideActive\":" + String(gTempOverrideActive ? "true" : "false") + ",";
  json += "\"tempOverrideC\":" + String(gTempOverrideC, 1) + ",";
  json += "\"event\":\"" + gLastEvent + "\",";
  json += "\"led\":\"" + ledState + "\",";
  json += "\"buzzer\":" + String(buzzerState ? "true" : "false") + ",";
  json += "\"motor\":" + String(motorState ? "true" : "false") + ",";
  json += "\"mode\":" + String((int)gMode) + ",";
  json += "\"modeName\":\"" + String(modeName(gMode)) + "\",";
  json += "\"season\":\"" + String(seasonName(gDisplayedSeason)) + "\",";
  json += "\"musicEnabled\":" + String(gMusicEnabled ? "true" : "false") + ",";
  json += "\"volumeLevel\":" + String(gVolumeLevel) + ",";
  json += "\"rfidUid\":\"" + gLastRFIDUID + "\",";
  json += "\"rfidType\":\"" + gLastRFIDType + "\",";
  json += "\"anim\":\"" + String(animName(gAnim.type)) + "\"";
  json += "}";
  return json;
}

void broadcastState() {
  ws.textAll(makeJson());
}

// ================= WIFI =================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(500);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to Wi-Fi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi FAILED");
  }
}

// ================= MODE SWITCHING =================
// Resets all outputs, reads the current temperature, and enters the
// correct starting animation for the currently detected season.
void enterTamagotchiMode() {
  gMode = MODE_TAMAGOTCHI;
  gLastEvent = "Tamagotchi ON";
  stopAudio();
  motorOff();
  rgbOff();
  gWeatherNotifyActive = false;
  gLastTouchMs = millis();

  gDisplayedSeason = determineSeason(getEffectiveTempC());
  if (gDisplayedSeason == SEASON_NORMAL) {
    spriteStartIdle();
  } else {
    spriteStartSeasonTransition(gDisplayedSeason);
  }
}

// ================= COMMAND HANDLING =================
// Parses commands received from the web dashboard over WebSocket.
// "tempTarget:XX.X" activates the 10 s temperature override.
// All other commands directly control outputs or the LED.
void handleCommand(const String& cmd) {
  if (cmd.startsWith("tempTarget:")) {
    String val = cmd.substring(String("tempTarget:").length());
    float target = val.toFloat();
    setTempTargetC(target);
    gLastEvent = "Temp Target Set";
    broadcastState();
    return;
  }

  if      (cmd == "led:red")     { rgbRed();   gLastEvent = "LED RED"; }
  else if (cmd == "led:green")   { rgbGreen(); gLastEvent = "LED GREEN"; }
  else if (cmd == "led:blue")    { rgbBlue();  gLastEvent = "LED BLUE"; }
  else if (cmd == "led:off")     { rgbOff();   gLastEvent = "LED OFF"; }
  else if (cmd == "buzzer:on")   { if (!audioRunning && !seasonMelodyRunning) { buzzerOn(); gLastEvent = "Buzzer ON"; } }
  else if (cmd == "buzzer:off")  { buzzerOff(); gLastEvent = "Buzzer OFF"; }
  else if (cmd == "motor:on")    { motorOn(); gLastEvent = "Motor ON"; }
  else if (cmd == "motor:off")   { motorOff(); gLastEvent = "Motor OFF"; }

  broadcastState();
}

// Handles WebSocket connection and incoming message events.
// On connect, immediately sends the current full state to the new client.
void onWsEvent(AsyncWebSocket* s, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->text(makeJson());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String cmd;
      for (size_t i = 0; i < len; i++) cmd += (char)data[i];
      handleCommand(cmd);
    }
  }
}

// Registers the WebSocket handler and the single HTTP GET route for the dashboard
void setupServer() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", INDEX_HTML);
  });
  server.begin();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed(micros());   // Seeds the RNG for Simon Says sequence generation

  // Configure all output pins
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);

  // Buttons use internal pull-ups — LOW when pressed
  pinMode(BTN_SW5, INPUT_PULLUP);
  pinMode(BTN_SW8, INPUT_PULLUP);
  pinMode(BTN_SW9, INPUT_PULLUP);

  motorOff();
  buzzerOff();
  rgbOff();

  pinMode(TOUCH_PIN, INPUT);
  pinMode(VIB_PIN, INPUT_PULLUP);
  pinMode(TILT_PIN, INPUT_PULLUP);
  analogReadResolution(12);   // 12-bit ADC for thermistor precision

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed!");
  }

  setCpuFrequencyMhz(240);   // Max clock for smooth animation performance

  // Initialise the Sharp display and blank it
  display.begin();
  display.clearDisplay();
  display.refresh();

  // Initialise RC522 — up to 10 attempts with full hard-reset recovery between tries
  delay(300);
  SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);
  pinMode(RC522_RST_PIN, OUTPUT);
  digitalWrite(RC522_RST_PIN, HIGH);

  bool rfidReady = false;
  for (int i = 0; i < 10; i++) {
    Serial.printf("[RFID] Init attempt %d...\n", i + 1);
    if (initRFIDRobust(true)) {
      rfidReady = true;
      break;
    }
    delay(400);
  }

  if (rfidReady) {
    Serial.println("[RFID] Module OK.");
  } else {
    Serial.println("[RFID] WARNING: Module not responding after retries.");
  }

  // RGB startup flash — visual confirmation that all three LED channels work
  rgbRed();   delay(150);
  rgbGreen(); delay(150);
  rgbBlue();  delay(150);
  rgbOff();

  connectWiFi();
  setupServer();

  // Read temperature once at boot so the correct season loads immediately.
  // After this, the thermistor is only sampled once per hour in the main loop.
  gTherm = analogRead(THERM_PIN);
  computeTemperature(gTherm);
  gLastTempReadMs = millis();
  gCurrentSeason = determineSeason(getEffectiveTempC());
  gDisplayedSeason = gCurrentSeason;

  gLastTouchMs = millis();
  if (gDisplayedSeason == SEASON_NORMAL) {
    spriteStartIdle();
  } else {
    startWeatherNotification(gDisplayedSeason);
    spriteStartSeasonTransition(gDisplayedSeason);
  }

  Serial.println("=== WEATHER TAMAGOTCHI ===");
  Serial.println("Touch = Pet");
  Serial.println("RFID Card = Eating shake minigame");
  Serial.println("RFID Fob = Training shake minigame");
  Serial.println("Seasonal animations loaded from LittleFS");
  Serial.println("IoT slider temporarily overrides thermistor for 10 seconds");
  Serial.println("Thermistor updates once per hour when override is off");
}

// Renders a large centred text string on a white background.
// Used for the 3-2-1 countdown digits and the "SHAKE!" prompt.
void drawCountdownScreen(const char* text) {
  display.clearDisplayBuffer();
  display.fillRect(0, 0, 400, 240, WHITE);
  display.setTextColor(BLACK);
  int textSize = (strcmp(text, "SHAKE!") == 0) ? 5 : 7;
  int len = strlen(text);
  int charW = 6 * textSize;
  int totalW = len * charW;
  int x = (400 - totalW) / 2;
  int y = (240 - (8 * textSize)) / 2;
  display.setTextSize(textSize);
  display.setCursor(x, y);
  display.print(text);
  display.refresh();
}

// Kicks off the shared 3-2-1 countdown that precedes both minigames
void startShakeCountdown(PendingActionType action) {
  if (gMode != MODE_TAMAGOTCHI) return;
  gPendingAction = action;
  gMiniGameState = MINIGAME_COUNTDOWN_3;
  gMiniGameStateStartMs = millis();
  gShakeWindowStartMs = 0;
  gLastTiltTriggerMs = 0;
  gShakeCount = 0;
  gLastTiltForCount = digitalRead(TILT_PIN);
  rgbRed();
  drawCountdownScreen("3");
  playToneBlocking(900, 80);
}

// State machine that drives both minigames.
// Called every loop — each state checks its exit condition and transitions
// to the next state or resolves the game with a success or failure result.
void updateShakeMiniGame() {
  if (gMiniGameState == MINIGAME_NONE) return;

  uint32_t now = millis();

  if (gMiniGameState == MINIGAME_COUNTDOWN_3) {
    if (now - gMiniGameStateStartMs >= COUNTDOWN_STEP_MS) {
      gMiniGameState = MINIGAME_COUNTDOWN_2;
      gMiniGameStateStartMs = now;
      rgbYellow();
      drawCountdownScreen("2");
      playToneBlocking(1000, 80);
    }
    return;
  }

  if (gMiniGameState == MINIGAME_COUNTDOWN_2) {
    if (now - gMiniGameStateStartMs >= COUNTDOWN_STEP_MS) {
      gMiniGameState = MINIGAME_COUNTDOWN_1;
      gMiniGameStateStartMs = now;
      rgbYellow();
      drawCountdownScreen("1");
      playToneBlocking(1100, 80);
    }
    return;
  }

  if (gMiniGameState == MINIGAME_COUNTDOWN_1) {
    if (now - gMiniGameStateStartMs >= COUNTDOWN_STEP_MS) {
      // After the countdown: card → Simon Says, fob → shake game
      if (gPendingAction == ACTION_EAT) {
        startSimonGame();
      } else {
        gMiniGameState = MINIGAME_SHAKE;
        gMiniGameStateStartMs = now;
        gShakeWindowStartMs = now;
        gShakeCount = 0;
        rgbGreen();
        drawCountdownScreen("SHAKE!");
        playToneBlocking(1500, 180);
        gLastTiltForCount = digitalRead(TILT_PIN);
      }
    }
    return;
  }

  if (gMiniGameState == MINIGAME_SIMON_WAIT) {
    bool btn5 = digitalRead(BTN_SW5);
    bool btn8 = digitalRead(BTN_SW8);
    bool btn9 = digitalRead(BTN_SW9);
    bool touchActive = (digitalRead(TOUCH_PIN) == HIGH);

    // Wait for all inputs to release before the timer starts for the next prompt
    if (gSimonWaitingRelease) {
      if (btn5 == HIGH && btn8 == HIGH && btn9 == HIGH && !touchActive) {
        gSimonWaitingRelease = false;
        gSimonPromptStartMs = millis();
      }
      return;
    }

    // Timeout — player didn't respond within the 3 s window
    if (now - gSimonPromptStartMs >= SIMON_PROMPT_WINDOW_MS) {
      gMiniGameState = MINIGAME_NONE;
      gPendingAction = ACTION_NONE;
      gLastEvent = "Simon Failed";
      spriteStartFailed();
      broadcastState();
      return;
    }

    // Correct input — advance to the next prompt
    if (isSimonPromptTriggered(gCurrentSimonPrompt, btn5, btn8, btn9, touchActive)) {
      rgbGreen();
      playPositivePromptTone();
      gSimonPromptIndex++;
      delay(90);
      beginNextSimonPrompt();
      return;
    }

    // Any wrong input immediately fails the round
    bool wrongPress =
      ((btn9 == LOW) && gCurrentSimonPrompt != SIMON_LEFT) ||
      ((btn5 == LOW) && gCurrentSimonPrompt != SIMON_MID) ||
      ((btn8 == LOW) && gCurrentSimonPrompt != SIMON_RIGHT) ||
      (touchActive && gCurrentSimonPrompt != SIMON_TOUCH);

    if (wrongPress) {
      gMiniGameState = MINIGAME_NONE;
      gPendingAction = ACTION_NONE;
      gLastEvent = "Simon Failed";
      spriteStartFailed();
      broadcastState();
      return;
    }
    return;
  }

  if (gMiniGameState == MINIGAME_SHAKE) {
    // Count rising-edge tilt transitions with debounce
    bool tiltState = digitalRead(TILT_PIN);
    if (gLastTiltForCount == HIGH && tiltState == LOW && (now - gLastTiltTriggerMs >= SHAKE_DEBOUNCE_MS)) {
      gLastTiltTriggerMs = now;
      gShakeCount++;
      // LED colour tracks progress: green → blue → magenta
      if (gShakeCount <= 2) rgbGreen();
      else if (gShakeCount <= 4) rgbBlue();
      else rgbMagenta();
    }
    gLastTiltForCount = tiltState;

    // 5 s window expired — evaluate whether the player hit the target
    if (now - gShakeWindowStartMs >= SHAKE_WINDOW_MS) {
      gMiniGameState = MINIGAME_NONE;
      if (gShakeCount >= REQUIRED_SHAKES) {
        gPendingAction = ACTION_NONE;
        gLastEvent = "Train Success";
        spriteStartTraining();
      } else {
        gPendingAction = ACTION_NONE;
        gLastEvent = "Shake Failed";
        spriteStartFailed();
      }
      broadcastState();
    }
    return;
  }
}

// Renders the Simon Says prompt word centred on a white background
void drawSimonPrompt(SimonPromptType prompt) {
  display.clearDisplayBuffer();
  display.fillRect(0, 0, 400, 240, WHITE);
  display.setTextColor(BLACK);
  display.setTextSize(5);

  const char* text = "MID";
  if (prompt == SIMON_LEFT) text = "LEFT";
  else if (prompt == SIMON_RIGHT) text = "RIGHT";
  else if (prompt == SIMON_TOUCH) text = "TOUCH";

  int len = strlen(text);
  int charW = 6 * 5;
  int totalW = len * charW;
  int x = (400 - totalW) / 2;
  int y = (240 - (8 * 5)) / 2;
  display.setCursor(x, y);
  display.print(text);
  display.refresh();
}

// Generates a random 5-prompt Simon sequence and shows the first prompt
void startSimonGame() {
  randomSeed(micros());
  for (int i = 0; i < SIMON_TOTAL_PROMPTS; i++) {
    gSimonSequence[i] = (SimonPromptType)random(0, 4);
  }
  gSimonPromptIndex = 0;
  beginNextSimonPrompt();
}

// Shows the next prompt in the sequence, or resolves the game successfully
// if all 5 prompts have been answered correctly.
void beginNextSimonPrompt() {
  if (gSimonPromptIndex >= SIMON_TOTAL_PROMPTS) {
    gMiniGameState = MINIGAME_NONE;
    gPendingAction = ACTION_NONE;
    gLastEvent = "Eat Success";
    spriteStartEating();
    broadcastState();
    return;
  }

  gCurrentSimonPrompt = gSimonSequence[gSimonPromptIndex];
  gMiniGameState = MINIGAME_SIMON_WAIT;
  gSimonPromptStartMs = millis();
  gSimonWaitingRelease = true;   // Don't accept input until everything is released first
  rgbRed();
  drawSimonPrompt(gCurrentSimonPrompt);
  playPromptTone();
}

// Returns true if the player activated the correct input for the given prompt
bool isSimonPromptTriggered(SimonPromptType prompt, bool btn5, bool btn8, bool btn9, bool touchActive) {
  switch (prompt) {
    case SIMON_LEFT: return btn9 == LOW;
    case SIMON_MID: return btn5 == LOW;
    case SIMON_RIGHT: return btn8 == LOW;
    case SIMON_TOUCH: return touchActive;
    default: return false;
  }
}

// Detects a fresh touch-pad press and triggers the pet animation.
// Suppressed during active minigames and during eating/training/result animations.
void handleTouchPadPet() {
  if (gMode != MODE_TAMAGOTCHI) return;
  if (gMiniGameState != MINIGAME_NONE) return;

  bool touchActive = (digitalRead(TOUCH_PIN) == HIGH);
  uint32_t now = millis();

  if (touchActive && !gLastTouch && (now - gLastTouchMs > 300)) {
    gLastTouchMs = now;
    if (gAnim.type != ANIM_PET && gAnim.type != ANIM_EATING && gAnim.type != ANIM_TRAINING &&
        gAnim.type != ANIM_SUCCESS && gAnim.type != ANIM_FAILED) {
      spriteStartPet();
      broadcastState();
    }
  }

  gLastTouch = touchActive;
}

// ================= LOOP =================
// Main loop runs on Core 1. Reads digital sensors every iteration,
// samples the thermistor only once per hour (or immediately at boot),
// handles button edge detection, and dispatches to subsystem update functions.
void loop() {
  const uint32_t now = millis();

  gTouch = digitalRead(TOUCH_PIN);
  gVib   = digitalRead(VIB_PIN);
  gTilt  = digitalRead(TILT_PIN);
  updateTempOverride();

  // Thermistor is read once per hour when the test override is not active.
  // The last reading is held until the next scheduled update.
  if (!gTempOverrideActive && (now - gLastTempReadMs >= TEMP_UPDATE_INTERVAL_MS)) {
    gTherm = analogRead(THERM_PIN);
    computeTemperature(gTherm);
    gLastTempReadMs = now;
    Serial.printf("[TEMP UPDATE] ADC=%d TempC=%.2f TempF=%.2f\n", gTherm, gTempC, gTempF);
  }

  bool btn5 = digitalRead(BTN_SW5);
  bool btn8 = digitalRead(BTN_SW8);
  bool btn9 = digitalRead(BTN_SW9);

  // 180 ms global button debounce — logs event and broadcasts for dashboard visibility
  if (now - gLastButtonMs > 180) {
    if (gMode == MODE_TAMAGOTCHI) {
      if (btn5 == LOW && gLastBtn5 == HIGH) {
        gLastButtonMs = now;
        gLastEvent = "SW5";
        broadcastState();
      }
      if (btn8 == LOW && gLastBtn8 == HIGH) {
        gLastButtonMs = now;
        gLastEvent = "SW8";
        broadcastState();
      }
      if (btn9 == LOW && gLastBtn9 == HIGH) {
        gLastButtonMs = now;
        gLastEvent = "SW9";
        broadcastState();
      }
    }
  }

  gLastBtn5 = btn5;
  gLastBtn8 = btn8;
  gLastBtn9 = btn9;

  handleTouchPadPet();

  if (gMode == MODE_TAMAGOTCHI) {
    handleTamagotchiRFID();       // Check for new RFID scans
    updateSeasonState();           // Re-evaluate season from temperature (once per second)
    updateWeatherNotification();   // Advance LED blink and motor during season-change alert
    updateShakeMiniGame();         // Advance countdown / shake / Simon state machine

    // Only animate when no minigame is blocking the display
    if (gMiniGameState == MINIGAME_NONE) {
      spriteTick();
    }

    // Broadcast state to dashboard every 300 ms
    if (now - gLastBroadcastMs > 300) {
      gLastBroadcastMs = now;
      broadcastState();
    }

    // Serial debug line every 700 ms
    if (now - gLastSerialMs > 700) {
      gLastSerialMs = now;
      Serial.printf(
        "[Tamagotchi] Touch=%d Vib=%d Tilt=%d ADC=%d TempC=%.1f TempF=%.1f EffTemp=%.1f Override=%d Season=%s Anim=%s RFID=%s UID=%s LED=%s Motor=%d Buzzer=%d\n",
        gTouch, gVib, gTilt, gTherm, gTempC, gTempF, getEffectiveTempC(),
        gTempOverrideActive, seasonName(gDisplayedSeason), animName(gAnim.type),
        gLastRFIDType.c_str(), gLastRFIDUID.c_str(),
        ledState.c_str(), motorState, buzzerState
      );
    }
  }

  ws.cleanupClients();   // Free memory from any disconnected WebSocket clients
  delay(5);
}