# Weather Tamagotchi

An ESP32-S3 based Tamagotchi-style pet that reacts to real-world temperature. The season changes automatically based on what the thermistor reads, triggering different animations, LED colors, and a short melody. Feed your pet by scanning an RFID card and completing a Simon Says minigame. Train it with the fob and a shake minigame. Pet it with the touch pad. Monitor everything live from a browser over Wi-Fi.

![Device Photo](images/device.jpg)

---

## Hardware

| Component | Pin(s) |
|---|---|
| Sharp Memory Display (400×240) | SCK=10, MOSI=11, SS=12 |
| RFID RC522 | SCK=47, MOSI=48, MISO=2, SS=21, RST=3 |
| Thermistor (NTC 10kΩ) | GPIO4 |
| Touch Sensor (TTP223) | GPIO14 |
| Ball Tilt Switch | GPIO13 |
| Vibration Motor | GPIO7 |
| Buzzer | GPIO15 |
| RGB LED (common anode) | R=18, G=17, B=16 |
| Button SW5 | GPIO5 |
| Button SW8 | GPIO8 |
| Button SW9 | GPIO9 |

![Schematic] <img width="912" height="885" alt="FinalSCH" src="https://github.com/user-attachments/assets/fcc6b772-361e-4fbe-9c03-b07094e89b2c" />


> The Sharp display uses software (bit-bang) SPI on pins 10/11/12, so there is no hardware SPI conflict with the RC522.

> RST is on GPIO3. GPIO1 is the ESP32-S3 UART TX line and causes the RC522 to fail silently on boot if used.

---

## Seasons

Temperature is read from the thermistor every second and mapped to one of five seasons. A calibration offset of -2.0°C is applied to correct for self-heating. When the season changes, a transition animation plays, the RGB LED switches color, the motor vibrates for 5 seconds, and a short 4-note melody plays.

| Season | Temp Range | LED Color |
|---|---|---|
| Winter | ≤ 4°C | White |
| Rain | 4 – 10°C | Blue |
| Fall | 10 – 16°C | Yellow |
| Normal | 16 – 26°C | Green |
| Summer | ≥ 26°C | Red |

The LED holds the season color as a steady light once the notification finishes. It pauses during eating and training animations.

---

## Minigames

### RFID Card — Simon Says (Eating)
Scan the card to start a 3-2-1 countdown, then a sequence of 5 prompts appear on the display one at a time. Each prompt tells you which button or sensor to activate within 3 seconds.

| Prompt | Input |
|---|---|
| LEFT | SW9 (GPIO9) |
| MID | SW5 (GPIO5) |
| RIGHT | SW8 (GPIO8) |
| TOUCH | Touch pad (GPIO14) |

Hit all 5 in time and the eating animation plays. Press the wrong input or run out of time and you get the failed animation.

### RFID Fob — Shake Game (Training)
Scan the fob to start a 3-2-1 countdown, then SHAKE the device using the tilt switch. You have 5 seconds to register 10 tilts.

The RGB LED tracks your progress — green → blue → magenta as you rack up shakes. Hit 10 and the training animation plays, miss and you get the failed animation.

---

## Other Interactions

### Touch Pad
Tap the TTP223 sensor to trigger a petting animation with a short two-tone sound. Won't fire during eating, training, or an active minigame.

### Idle Cycling
The idle animation alternates between a standing idle and a chair idle. After each full standing idle loop, there's a 25% chance of switching to the chair idle, with a minimum 45-second gap between chair appearances so it doesn't spam.

---

## Web Dashboard

The device serves a real-time WebSocket dashboard at its IP address. Open it in any browser on the same network. The IP is printed to Serial on boot.

From the dashboard you can:
- Watch live sensor readings (touch, vibration, tilt, temperature, ADC raw value)
- Control the RGB LED, buzzer, and motor manually
- Drag the temperature slider to override the thermistor for 10 seconds — handy for testing season transitions without physically heating or cooling anything
- See current season, animation state, RFID scan history, and last event

---

## Animations

Sprites are stored two different ways depending on size:

**Flash (PROGMEM)** — small enough to live in program memory:
- Idle (standing)
- Idle (chair)
- Eating
- Training

**LittleFS (RLE files)** — too large for flash, stored on the filesystem:
- Seasonal transition and idle animations for Winter, Rain, Fall, and Summer
- Pet animation
- Success and failed animations

RLE files use a custom binary format: `RLE1` magic bytes, width, height, frame count, bytes-per-frame, a per-frame offset lookup table, then compressed pixel data as `(uint16 run length, uint8 value)` triplets.

---

## LittleFS Files

Upload these to the ESP32 filesystem before flashing. Use the PlatformIO **Upload Filesystem Image** task or `esptool`.

```
/fall_transition.rle      24 frames
/fall_idle.rle            17 frames
/rain_transition.rle      37 frames
/rain_idle.rle            20 frames
/summer_transition.rle    20 frames
/summer_idle.rle          12 frames
/winter_transition.rle    33 frames
/winter_idle.rle          18 frames
/pet_mord17.rle           17 frames
/success_mord2.rle         2 frames
/failed_mord2.rle          2 frames
```

---

## Dependencies

- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SharpMem](https://github.com/adafruit/Adafruit_SharpMem)
- [MFRC522](https://github.com/miguelbalboa/rfid)
- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
- [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)
- [heatshrink](https://github.com/atomicobject/heatshrink) — must be built with `HEATSHRINK_DYNAMIC_ALLOC=false`
- LittleFS (built into the ESP32 Arduino core)

---

## Wi-Fi

SSID and password are hardcoded near the top of `main.cpp`:

```cpp
const char* WIFI_SSID = "your-ssid";
const char* WIFI_PASS = "your-password";
```

---

## RFID Troubleshooting

Every scan cycle re-asserts the SPI bus pins before reading from the RC522. A health check reads `VersionReg` each cycle — if it returns `0x00` or `0xFF`, the module gets a hard RST toggle (low 150ms → high) and attempts to re-initialise up to 5 times before giving up for that cycle.

On first boot, up to 10 init attempts are made before continuing. The result is printed to Serial.

If it keeps failing:
1. Make sure the RC522 is on **3.3V**, not 5V — it needs up to ~100mA peak so power it from your regulator directly, not from the ESP32's 3.3V pin if it's already loaded
2. Check all four SPI lines are wired correctly (SCK=47, MOSI=48, MISO=2, SS=21)
3. Confirm RST is on GPIO3 and not floating
