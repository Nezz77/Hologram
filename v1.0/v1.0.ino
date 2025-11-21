#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <NeoPixelBus.h>
#include <math.h>
#include "time.h"  // Required for NTP client and time functions

// --- FORWARD DECLARATIONS TO FIX LINKER ERROR (MANDATORY FIX) ---
void setup();
void loop();
// ----------------------------------------------------------------

// ======== Pin Configuration ========
#define ESC_PIN 18
#define LED_PIN 5
#define HALL_PIN 32

// Changed to 31 LEDs as common for this type of fan
#define NUM_LEDS 31

// ======== Ultrasonic Sensor Pins ========
#define TRIG_PIN 25  // Trigger pin for HC-SR04
#define ECHO_PIN 26  // Echo pin for HC-SR04

// ======== NTP Time Configuration ========
const char* ntpServer = "pool.ntp.org";
// Default time zone: UTC+5:30 (Colombo/India Standard Time)
const long gmtOffset_sec = 5 * 3600 + 30 * 60;
const int daylightOffset_sec = 0;
// ======================================


// ======== Objects ========
Servo esc;
NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> strip(NUM_LEDS, LED_PIN);
WebServer server(80);

// ======== WiFi Credentials ========
const char* ssid = "Galaxy Note10 ";
const char* password = "flba9547";

// ======== Variables ========
volatile bool newPulse = false;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long rotationTime = 0;
int throttle = 1200;
int minThrottle = 1000;
int maxThrottle = 1800;
String displayText = "HELLO";
RgbColor currentColor(0, 0, 255);
int delayTime = 400;  // fallback delay
float currentRPM = 0;

// === Timing Smoothing for Stable POV (FIXED-POINT EMA) ===
volatile unsigned long smoothRotationTime = 0;
const uint8_t EMA_SHIFT = 3;      // 2^3 = 8 (Divide by 8)
const uint8_t EMA_INFLUENCE = 1;  // New value takes 1/8 of the average
const uint8_t EMA_OLD = 7;        // Old average takes 7/8 of the average
// ============================================================

// 🔧 Text width scaling factor (adjust between 0.5–1.0)
float textStretch = 0.7;

// 🧭 Adjustable angular position offset
int angleOffset = 0;  // microseconds to delay drawing start

// 📐 Maximum percentage of the circle the text/emoji can occupy
const float MAX_DRAWING_SECTOR_PERCENT = 30.0;

// === State Tracking and Mode Control ===
bool obstacleWasPresent = false;

// New: Display mode - ADDED MODE_CLOCK
enum PovMode { MODE_TEXT,
               MODE_EMOJI,
               MODE_WAVE,
               MODE_CLOCK };
PovMode currentMode = MODE_CLOCK;  // Default to CLOCK mode

// New: Variables for the low-RPM Wave animation
int waveOffset = 0;               // Tracks the angular shift of the wave (0-255)
const int WAVE_SPEED = 2;         // How many pixels to shift per rotation (adjust for faster/slower movement)
const int WAVE_BRIGHTNESS = 200;  // Max brightness for the wave effect


// New: Emoji Configuration (5x10 Heart)
#define EMOJI_HEIGHT 10
#define EMOJI_WIDTH 5
// Calculates the starting index to center the 10-pixel graphic on 31 LEDs
const int EMOJI_START_LED = (NUM_LEDS - EMOJI_HEIGHT) / 2;

// 10-bit Heart Graphic (5 columns)
// Each column is represented by two bytes (16 bits total). We only use the first 10 bits.
// Index 0: LOWER_BYTE (Bits 0-7, inner LEDs)
// Index 1: UPPER_BYTE (Bits 8-9, outer LEDs)
// Rows 0-9, 0 is the innermost LED (index EMOJI_START_LED)
const byte heart10x5[EMOJI_WIDTH][2] = {
  // { R0-R7 (Inner), R8-R15 (Outer) }
  // Col 0:
  { 0b00001110, 0b00000001 },
  // Col 1:
  { 0b00010001, 0b00000010 },
  // Col 2 (Center):
  { 0b00100000, 0b00000001 },
  // Col 3:
  { 0b00010001, 0b00000010 },
  // Col 4:
  { 0b00001110, 0b00000001 }
};
// ==============================================

// ======== Basic 5x7 Font (Unchanged) ========
const byte font5x7[][5] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00 },  // 32  space
  { 0x00, 0x00, 0x5F, 0x00, 0x00 },  // 33  !
  { 0x00, 0x07, 0x00, 0x07, 0x00 },  // 34  "
  { 0x14, 0x7F, 0x14, 0x7F, 0x14 },  // 35  #
  { 0x24, 0x2A, 0x7F, 0x2A, 0x12 },  // 36  $
  { 0x62, 0x64, 0x08, 0x13, 0x26 },  // 37  %
  { 0x36, 0x49, 0x56, 0x20, 0x50 },  // 38  &
  { 0x00, 0x05, 0x03, 0x00, 0x00 },  // 39  '
  { 0x00, 0x1C, 0x22, 0x41, 0x00 },  // 40  (
  { 0x00, 0x41, 0x22, 0x1C, 0x00 },  // 41  )
  { 0x14, 0x08, 0x3E, 0x08, 0x14 },  // 42  *
  { 0x08, 0x08, 0x3E, 0x08, 0x08 },  // 43  +
  { 0x00, 0x50, 0x30, 0x00, 0x00 },  // 44  ,
  { 0x08, 0x08, 0x08, 0x08, 0x08 },  // 45  -
  { 0x00, 0x60, 0x60, 0x00, 0x00 },  // 46  .
  { 0x20, 0x10, 0x08, 0x04, 0x02 },  // 47  /
  { 0x3E, 0x51, 0x49, 0x45, 0x3E },  // 48  0
  { 0x00, 0x42, 0x7F, 0x40, 0x00 },  // 49  1
  { 0x42, 0x61, 0x51, 0x49, 0x46 },  // 50  2
  { 0x21, 0x41, 0x45, 0x4B, 0x31 },  // 51  3
  { 0x18, 0x14, 0x12, 0x7F, 0x10 },  // 52  4
  { 0x27, 0x45, 0x45, 0x45, 0x39 },  // 53  5
  { 0x3C, 0x4A, 0x49, 0x49, 0x30 },  // 54  6
  { 0x01, 0x71, 0x09, 0x05, 0x03 },  // 55  7
  { 0x36, 0x49, 0x49, 0x49, 0x36 },  // 56  8
  { 0x06, 0x49, 0x49, 0x29, 0x1E },  // 57  9
  { 0x00, 0x36, 0x36, 0x00, 0x00 },  // 58  :
  { 0x00, 0x56, 0x36, 0x00, 0x00 },  // 59  ;
  { 0x08, 0x14, 0x22, 0x41, 0x00 },  // 60  <
  { 0x14, 0x14, 0x14, 0x14, 0x14 },  // 61  =
  { 0x00, 0x41, 0x22, 0x14, 0x08 },  // 62  >
  { 0x02, 0x01, 0x51, 0x09, 0x06 },  // 63  ?
  { 0x32, 0x49, 0x79, 0x41, 0x3E },  // 64  @
  { 0x7E, 0x11, 0x11, 0x11, 0x7E },  // 65  A
  { 0x7F, 0x49, 0x49, 0x49, 0x36 },  // 66  B
  { 0x3E, 0x41, 0x41, 0x41, 0x22 },  // 67  C
  { 0x7F, 0x41, 0x41, 0x22, 0x1C },  // 68  D
  { 0x7F, 0x49, 0x49, 0x49, 0x41 },  // 69  E
  { 0x7F, 0x09, 0x09, 0x09, 0x01 },  // 70  F
  { 0x3E, 0x41, 0x49, 0x49, 0x7A },  // 71  G
  { 0x7F, 0x08, 0x08, 0x08, 0x7F },  // 72  H
  { 0x00, 0x41, 0x7F, 0x41, 0x00 },  // 73  I
  { 0x20, 0x40, 0x41, 0x3F, 0x01 },  // 74  J
  { 0x7F, 0x10, 0x28, 0x44, 0x00 },  // 75  K
  { 0x7F, 0x40, 0x40, 0x40, 0x40 },  // 76  L
  { 0x7F, 0x02, 0x0C, 0x02, 0x7F },  // 77  M
  { 0x7F, 0x04, 0x08, 0x10, 0x7F },  // 78  N
  { 0x3E, 0x41, 0x41, 0x41, 0x3E },  // 79  O
  { 0x7F, 0x09, 0x09, 0x09, 0x06 },  // 80  P
  { 0x3E, 0x41, 0x51, 0x21, 0x5E },  // 81  Q
  { 0x7F, 0x09, 0x19, 0x29, 0x46 },  // 82  R
  { 0x46, 0x49, 0x49, 0x49, 0x31 },  // 83  S
  { 0x01, 0x01, 0x7F, 0x01, 0x01 },  // 84  T
  { 0x3F, 0x40, 0x40, 0x40, 0x3F },  // 85  U
  { 0x1F, 0x20, 0x40, 0x20, 0x1F },  // 86  V
  { 0x3F, 0x40, 0x38, 0x40, 0x3F },  // 87  W
  { 0x63, 0x14, 0x08, 0x14, 0x63 },  // 88  X
  { 0x07, 0x08, 0x70, 0x08, 0x07 },  // 89  Y
  { 0x61, 0x51, 0x49, 0x45, 0x43 },  // 90  Z
  { 0x00, 0x7F, 0x41, 0x41, 0x00 },  // 91  [
  { 0x02, 0x04, 0x08, 0x10, 0x20 },  // 92  backslash
  { 0x00, 0x41, 0x41, 0x7F, 0x00 },  // 93  ]
  { 0x04, 0x02, 0x01, 0x02, 0x04 },  // 94  ^
  { 0x40, 0x40, 0x40, 0x40, 0x40 },  // 95  _
  { 0x00, 0x01, 0x02, 0x04, 0x00 },  // 96  `
  { 0x20, 0x54, 0x54, 0x54, 0x78 },  // 97  a
  { 0x7F, 0x48, 0x44, 0x44, 0x38 },  // 98  b
  { 0x38, 0x44, 0x44, 0x44, 0x20 },  // 99  c
  { 0x38, 0x44, 0x44, 0x48, 0x7F },  // 100 d
  { 0x38, 0x54, 0x54, 0x54, 0x18 },  // 101 e
  { 0x08, 0x7E, 0x09, 0x01, 0x02 },  // 102 f
  { 0x0C, 0x52, 0x52, 0x52, 0x3E },  // 103 g
  { 0x7F, 0x08, 0x04, 0x04, 0x78 },  // 104 h
  { 0x00, 0x44, 0x7D, 0x40, 0x00 },  // 105 i
  { 0x20, 0x40, 0x44, 0x3D, 0x00 },  // 106 j
  { 0x7F, 0x10, 0x28, 0x44, 0x00 },  // 107 k
  { 0x00, 0x41, 0x7F, 0x40, 0x00 },  // 108 l
  { 0x7C, 0x04, 0x18, 0x04, 0x78 },  // 109 m
  { 0x7C, 0x08, 0x04, 0x04, 0x78 },  // 110 n
  { 0x38, 0x44, 0x44, 0x44, 0x38 },  // 111 o
  { 0x7C, 0x14, 0x14, 0x14, 0x08 },  // 112 p
  { 0x08, 0x14, 0x14, 0x18, 0x7C },  // 113 q
  { 0x7C, 0x08, 0x04, 0x04, 0x08 },  // 114 r
  { 0x48, 0x54, 0x54, 0x54, 0x24 },  // 115 s
  { 0x04, 0x3F, 0x44, 0x40, 0x20 },  // 116 t
  { 0x3C, 0x40, 0x40, 0x20, 0x7C },  // 117 u
  { 0x1C, 0x20, 0x40, 0x20, 0x1C },  // 118 v
  { 0x3C, 0x40, 0x30, 0x40, 0x3C },  // 119 w
  { 0x44, 0x28, 0x10, 0x28, 0x44 },  // 120 x
  { 0x0C, 0x50, 0x50, 0x50, 0x3C },  // 121 y (fixed lowercase version)
  { 0x44, 0x64, 0x54, 0x4C, 0x44 },  // 122 z
  { 0x00, 0x08, 0x36, 0x41, 0x00 },  // 123 {
  { 0x00, 0x00, 0x7F, 0x00, 0x00 },  // 124 |
  { 0x00, 0x41, 0x36, 0x08, 0x00 },  // 125 }
  { 0x08, 0x04, 0x08, 0x10, 0x08 },  // 126 ~
};

// ======== Interrupt: Hall Sensor Pulse (Unchanged) ========
void IRAM_ATTR hallISR() {
  unsigned long now = micros();
  if (lastPulseTime != 0) {
    rotationTime = now - lastPulseTime;
    currentRPM = 60.0 * 1000000.0 / rotationTime;

    // Exponential Moving Average (EMA) using integer math for ISR safety.
    if (smoothRotationTime == 0) {
      smoothRotationTime = rotationTime;
    } else {
      smoothRotationTime = ((rotationTime * EMA_INFLUENCE) + (smoothRotationTime * EMA_OLD)) >> EMA_SHIFT;
    }
  }
  lastPulseTime = now;
  newPulse = true;
}

// ======== Helper: Draw One Column (for 5x7 Text) ========
void drawColumn(byte columnBits, RgbColor color) {
  strip.ClearTo(RgbColor(0, 0, 0));
  for (int i = 0; i < 7; i++) {
    // We only use the first 7 LEDs for the 5x7 font height
    if (columnBits & (1 << i)) strip.SetPixelColor(i, color);
  }
  strip.Show();
}

// ======== Helper: Draw One Column (for 5x10 Emoji) ========
void drawEmojiColumn(const byte* columnBytes, RgbColor color) {
  strip.ClearTo(RgbColor(0, 0, 0));

  byte lowerByte = columnBytes[0];  // R0-R7
  byte upperByte = columnBytes[1];  // R8-R9

  for (int i = 0; i < EMOJI_HEIGHT; i++) {
    bool isLit = false;
    if (i < 8) {
      // Check bits 0-7 from the lower byte
      if (lowerByte & (1 << i)) isLit = true;
    } else {
      // Check bits 8-9 (which are bits 0-1 of the upper byte)
      if (upperByte & (1 << (i - 8))) isLit = true;
    }

    if (isLit) {
      // Draw at the centered position
      strip.SetPixelColor(EMOJI_START_LED + i, color);
    }
  }
  strip.Show();
}


// ======== Display Text as POV (Updated to use textStretch) ========
void displayPOVText(const String& text, RgbColor color) {
  unsigned long stableRotationTime = smoothRotationTime;

  if (stableRotationTime == 0) return;

  // 1. Calculate Maximum Drawing Duration (limit the text to 30% of rotation)
  unsigned long maxDrawingDuration = (unsigned long)(stableRotationTime * MAX_DRAWING_SECTOR_PERCENT / 100.0);

  // 2. Calculate Column Count for the given text
  unsigned long totalCols = text.length() * 6;  // 5 columns + 1 blank space per character

  // 3. Calculate the ideal base delay per column, then scale it by the user's stretch factor.
  unsigned long baseColDelay = maxDrawingDuration / totalCols;

  // Apply the user's textStretch (0.5 to 1.0) to control the width.
  unsigned long colDelay = (unsigned long)(baseColDelay * textStretch);

  // Ensure the delay is non-zero
  if (colDelay == 0) colDelay = 1;

  // 4. Calculate total time drawing actually started
  unsigned long startTime = micros();

  // Wait before drawing — applies the angular position offset
  if (angleOffset > 0) delayMicroseconds(angleOffset);

  // --- DRAWING LOOP ---
  for (int c = 0; c < text.length(); c++) {
    char ch = text[c];
    if (ch < 32 || ch > 126) continue;
    const byte* letter = font5x7[ch - 32];

    for (int col = 0; col < 5; col++) {
      drawColumn(letter[col], color);
      delayMicroseconds(colDelay);
    }

    // Blank column between letters
    drawColumn(0, RgbColor(0, 0, 0));
    delayMicroseconds(colDelay);
  }

  // --- DARK PERIOD (The essential fix) ---
  strip.ClearTo(RgbColor(0, 0, 0));
  strip.Show();

  // 6. Wait for the remainder of the rotation time
  unsigned long endTime = micros();
  unsigned long drawingDuration = endTime - startTime;

  if (drawingDuration < stableRotationTime) {
    unsigned long waitTime = stableRotationTime - drawingDuration;
    unsigned long targetTime = micros() + waitTime;
    while (micros() < targetTime) {
      if (newPulse) break;
    }
  }
}

// ======== Display Emoji POV Logic (Updated to use textStretch) ========
void displayEmojiHeart(RgbColor color) {
  unsigned long stableRotationTime = smoothRotationTime;

  if (stableRotationTime == 0) return;

  // 1. Calculate Maximum Drawing Duration (Use the same sector limit)
  unsigned long maxDrawingDuration = (unsigned long)(stableRotationTime * MAX_DRAWING_SECTOR_PERCENT / 100.0);

  // 2. Calculate Column Count for the heart (5 columns + 2 blanks for centering)
  unsigned long totalCols = EMOJI_WIDTH + 2;

  // 3. Calculate the ideal base delay per column, then scale it by the user's stretch factor.
  unsigned long baseColDelay = maxDrawingDuration / totalCols;

  // Apply the user's textStretch (0.5 to 1.0) to control the width.
  unsigned long colDelay = (unsigned long)(baseColDelay * textStretch);

  // Ensure the delay is non-zero
  if (colDelay == 0) colDelay = 1;

  // 4. Calculate total time drawing actually started
  unsigned long startTime = micros();

  // Wait before drawing — applies the angular position offset
  if (angleOffset > 0) delayMicroseconds(angleOffset);

  // --- DRAWING LOOP ---
  // Optional: Draw a blank column for visual separation
  drawEmojiColumn(heart10x5[0], RgbColor(0, 0, 0));
  delayMicroseconds(colDelay);

  for (int col = 0; col < EMOJI_WIDTH; col++) {
    drawEmojiColumn(heart10x5[col], color);
    delayMicroseconds(colDelay);
  }

  // Optional: Draw a blank column for visual separation
  drawEmojiColumn(heart10x5[0], RgbColor(0, 0, 0));
  delayMicroseconds(colDelay);

  // --- DARK PERIOD ---
  strip.ClearTo(RgbColor(0, 0, 0));
  strip.Show();

  // 6. Wait for the remainder of the rotation time
  unsigned long endTime = micros();
  unsigned long drawingDuration = endTime - startTime;

  if (drawingDuration < stableRotationTime) {
    unsigned long waitTime = stableRotationTime - drawingDuration;
    unsigned long targetTime = micros() + waitTime;
    while (micros() < targetTime) {
      if (newPulse) break;
    }
  }
}

// ======== Display Color Wave POV Logic (Ideal for <1000 RPM) ========
void displayPOVWaves(RgbColor baseColor) {
  unsigned long stableRotationTime = smoothRotationTime;

  if (stableRotationTime == 0) return;

  // Use a larger sector for the wave to make it look smooth and continuous
  const float WAVE_DRAWING_SECTOR_PERCENT = 60.0;
  unsigned long maxDrawingDuration = (unsigned long)(stableRotationTime * WAVE_DRAWING_SECTOR_PERCENT / 100.0);

  // Use a high number of columns to ensure a dense, smooth image.
  const int NUM_WAVE_COLUMNS = 100;
  unsigned long colDelay = maxDrawingDuration / NUM_WAVE_COLUMNS;

  // Ensure delay is non-zero
  if (colDelay == 0) colDelay = 1;

  // Wait before drawing — applies the angular position offset
  if (angleOffset > 0) delayMicroseconds(angleOffset);

  // --- DRAWING LOOP ---
  for (int col = 0; col < NUM_WAVE_COLUMNS; col++) {

    strip.ClearTo(RgbColor(0, 0, 0));

    for (int i = 0; i < NUM_LEDS; i++) {
      // Calculate the position in the 0-255 range for a sine wave effect
      // i * 15: Controls the vertical (radial) frequency
      // col * 5: Controls the angular frequency
      // waveOffset * 5: Global movement over rotations (makes it look like it's moving)
      int position = (i * 15 + col * 5 + waveOffset * 5);

      // Use a sine wave to create a smooth brightness gradient.
      // We use position % 256 for the phase, then map sin() output (-1 to 1) to (0 to WAVE_BRIGHTNESS)
      byte brightness = (byte)(sin(position * PI / 128.0) * WAVE_BRIGHTNESS / 2 + WAVE_BRIGHTNESS / 2);

      // Mix the base color with the calculated brightness
      RgbColor waveColor(
        (baseColor.R * brightness) / 255,
        (baseColor.G * brightness) / 255,
        (baseColor.B * brightness) / 255);

      strip.SetPixelColor(i, waveColor);
    }

    strip.Show();
    delayMicroseconds(colDelay);
  }

  // --- DARK PERIOD ---
  strip.ClearTo(RgbColor(0, 0, 0));
  strip.Show();

  // Update the wave offset for continuous motion
  waveOffset = (waveOffset + WAVE_SPEED) % 256;

  // Wait for the remainder of the rotation time
  // Note: We don't track the duration here, but rely on the main loop's logic
  // to ensure the next drawing starts only after the next Hall pulse.
}

// ======== Display Clock as POV ========
void displayPOVClock(RgbColor color) {
  struct tm timeinfo;

  // Check if time is successfully synchronized
  if (!getLocalTime(&timeinfo)) {
    // Fallback if time synchronization fails
    displayPOVText("NO TIME", RgbColor(255, 0, 0));
    return;
  }

  // Format the time as HH:MM (24-hour format)
  char timeBuffer[6];  // HH:MM\0
  strftime(timeBuffer, sizeof(timeBuffer), "%H:%M", &timeinfo);

  // Call the generic text display function
  displayPOVText(String(timeBuffer), color);
}


// ======== Helper: Get Distance (Unchanged) ========
long getDistanceCm() {
  // Send 10us trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pulse with a 30ms timeout (safe range)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return 999;  // Return a large number to indicate "out of range" or timeout
  }

  // Calculate distance in cm
  // (duration in µs) / 58 = distance in cm
  long distance = duration / 58;
  return distance;
}


// ======== Web Interface (Updated to include mode switch and time display) ========
String htmlPage() {
  // Get current time string for display in UI
  struct tm timeinfo;
  String currentTimeStr = "Syncing...";
  if (getLocalTime(&timeinfo)) {
    char timeBuffer[20];  // HH:MM:SS format
    strftime(timeBuffer, sizeof(timeBuffer), "%H:%M:%S", &timeinfo);
    currentTimeStr = String(timeBuffer);
  }

  String page = R"rawliteral(
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">

<style>
  body {
    font-family: Arial, sans-serif;
    background: #0d0d0d;
    color: #f5f5f5;
    text-align: center;
    margin: 0;
    padding: 0;
  }

  h2 {
    margin-top: 20px;
    font-size: 26px;
    letter-spacing: 1px;
  }

  p {
    font-size: 16px;
    margin: 10px 0 20px 0;
  }

  .card {
    background: #1c1c1c;
    border-radius: 14px;
    padding: 20px;
    width: 90%;
    max-width: 420px;
    margin: 20px auto;
    box-shadow: 0 0 20px rgba(255, 255, 255, 0.04);
  }

  input[type='submit'] {
    background: #0078ff;
    color: white;
    border: none;
    padding: 10px 18px;
    margin-top: 10px;
    font-size: 15px;
    border-radius: 8px;
    cursor: pointer;
    width: 100%;
  }

  input[type='submit']:hover {
    background: #0a5cd6;
  }

  input[type='number'],
  input[type='text'],
  input[type='color'] {
    padding: 10px;
    width: 95%;
    margin-top: 10px;
    border-radius: 8px;
    border: none;
    background: #2a2a2a;
    color: white;
    font-size: 15px;
  }

  form {
    margin-bottom: 25px;
  }

  hr { display: none; } 
</style>

</head>

<body>

<h2>HOLOGRAM-FUSION 5</h2>

<p>
  Mode: <b>%MODE_TEXT%</b> |
  Time: <b>%CURRENT_TIME%</b> |
  Throttle: %THROTTLE% |
  RPM: %RPM%
</p>

<div class="card">
  <form action='/mode' method='get'>
    <input type='hidden' name='m' value='%NEXT_MODE%'>
    <input type='submit' value='Switch to %NEXT_MODE_TEXT% Mode'>
  </form>

  <form action='/set' method='get'>
    <input name='val' type='number' min='1000' max='1800' value='%THROTTLE%'>
    <input type='submit' value='Set Throttle'>
  </form>

  <form action='/color' method='get'>
    <input type='color' name='color' value='%COLOR%'>
    <input type='submit' value='Change Color'>
  </form>

  <form action='/text' method='get'>
    <input name='txt' type='text' maxlength='12' value='%TEXT%'>
    <input type='submit' value='Update Text'>
  </form>

  <form action='/delay' method='get'>
    <p>Delay Time (µs)</p>
    <input name='d' type='number' min='100' max='2000' value='%DELAY%'>
    <input type='submit' value='Set Delay'>
  </form>

  <form action='/stretch' method='get'>
    <p>Drawing Width Scale (0.5–1.0)</p>
    <input name='s' type='number' min='0.5' max='1.0' step='0.05' value='%STRETCH%'>
    <input type='submit' value='Set Scale'>
  </form>

  <form action='/offset' method='get'>
    <p>Text Position Offset (0–100%)</p>
    <input name='o' type='number' min='0' max='100' step='1' value='%OFFSET%'>
    <input type='submit' value='Set Offset'>
  </form>
</div>

</body>
</html>
)rawliteral";


  char buf[8];
  sprintf(buf, "#%02X%02X%02X", currentColor.R, currentColor.G, currentColor.B);
  page.replace("%COLOR%", String(buf));
  page.replace("%THROTTLE%", String(throttle));
  page.replace("%TEXT%", displayText);
  page.replace("%DELAY%", String(delayTime));
  page.replace("%STRETCH%", String(textStretch, 2));
  page.replace("%RPM%", String(currentRPM, 1));
  page.replace("%CURRENT_TIME%", currentTimeStr);

  // Mode replacements - Now handles 4 modes: TEXT, EMOJI, WAVE, CLOCK
  String modeText, nextModeText;
  int nextMode;

  if (currentMode == MODE_TEXT) {
    modeText = "TEXT";
    nextMode = MODE_EMOJI;
    nextModeText = "EMOJI";
  } else if (currentMode == MODE_EMOJI) {
    modeText = "EMOJI (❤️)";
    nextMode = MODE_WAVE;
    nextModeText = "WAVE (Slow RPM)";
  } else if (currentMode == MODE_WAVE) {
    modeText = "WAVE (Slow RPM)";
    nextMode = MODE_CLOCK;
    nextModeText = "CLOCK";
  } else {  // MODE_CLOCK
    modeText = "CLOCK";
    nextMode = MODE_TEXT;
    nextModeText = "TEXT";
  }

  page.replace("%MODE_TEXT%", modeText);
  page.replace("%NEXT_MODE%", String(nextMode));
  page.replace("%NEXT_MODE_TEXT%", nextModeText);

  // Use smoothRotationTime for offset calculation display
  float percent = (smoothRotationTime > 0) ? (angleOffset * 100.0 / smoothRotationTime) : 0;
  page.replace("%OFFSET%", String(percent, 1));

  return page;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}
void handleSet() {
  if (server.hasArg("val")) {
    int val = server.arg("val").toInt();
    if (val >= minThrottle && val <= maxThrottle) {
      throttle = val;
    }
  }
  server.send(200, "text/html", htmlPage());
}
void handleColor() {
  if (server.hasArg("color")) {
    String colorHex = server.arg("color");
    long colorValue = strtol(colorHex.substring(1).c_str(), NULL, 16);
    currentColor = RgbColor((colorValue >> 16) & 0xFF, (colorValue >> 8) & 0xFF, colorValue & 0xFF);
  }
  server.send(200, "text/html", htmlPage());
}
void handleText() {
  if (server.hasArg("txt")) { displayText = server.arg("txt"); }
  server.send(200, "text/html", htmlPage());
}
void handleDelay() {
  if (server.hasArg("d")) {
    int val = server.arg("d").toInt();
    if (val >= 100 && val <= 2000) { delayTime = val; }
  }
  server.send(200, "text/html", htmlPage());
}
void handleStretch() {
  if (server.hasArg("s")) {
    float s = server.arg("s").toFloat();
    if (s >= 0.5 && s <= 1.0) textStretch = s;
  }
  server.send(200, "text/html", htmlPage());
}
void handleOffset() {
  // Use smoothRotationTime for stable offset setting
  if (server.hasArg("o") && smoothRotationTime > 0) {
    float percent = server.arg("o").toFloat();
    if (percent >= 0 && percent <= 100) {
      angleOffset = (smoothRotationTime * percent) / 100.0;
      Serial.printf("Offset set to %.1f%% (%d µs)\n", percent, angleOffset);
    }
  }
  server.send(200, "text/html", htmlPage());
}
void handleMode() {
  if (server.hasArg("m")) {
    currentMode = (PovMode)server.arg("m").toInt();
    Serial.printf("Switched mode to %d\n", currentMode);
  }
  server.send(200, "text/html", htmlPage());
}

// ======== Setup (The entry function referenced by _Z5setupv) ========
void setup() {
  Serial.begin(115200);
  strip.Begin();
  strip.Show();

  // --- Setup ultrasonic sensor pins ---
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1000);
  delay(3000);
  Serial.println("ESC Armed.");

  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());

  // --- Configure Time ---
  Serial.println("Configuring NTP Time...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Time configuration complete.");


  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/color", handleColor);
  server.on("/text", handleText);
  server.on("/delay", handleDelay);
  server.on("/stretch", handleStretch);
  server.on("/offset", handleOffset);
  server.on("/mode", handleMode);
  server.begin();
  Serial.println("Web server started");
}

// ======== Loop (The entry function referenced by _Z4loopv) ========
void loop() {
  server.handleClient();

  // --- Safety logic ---
  long distance = getDistanceCm();

  if (distance < 15) {
    // --- OBSTACLE DETECTED ---
    if (!obstacleWasPresent) {
      Serial.print("!!! OBSTACLE < 15cm. Distance: ");
      Serial.print(distance);
      Serial.println(" cm. STOPPING MOTOR.");
      obstacleWasPresent = true;
    }

    // Stop the motor by sending minimum throttle
    esc.writeMicroseconds(minThrottle);

    // Turn off the LEDs
    strip.ClearTo(RgbColor(0, 0, 0));
    strip.Show();

    // Prevent POV display from running
    newPulse = false;

  } else {
    // --- PATH CLEAR ---
    if (obstacleWasPresent) {
      Serial.println("Path clear. Resuming normal operation.");
      obstacleWasPresent = false;
    }

    // Run motor at the user-defined speed
    esc.writeMicroseconds(throttle);

    // Run POV display logic as normal
    if (newPulse) {

      newPulse = false;

      // Select display function based on mode
      if (currentMode == MODE_TEXT) {
        displayPOVText(displayText, currentColor);
      } else if (currentMode == MODE_EMOJI) {
        displayEmojiHeart(currentColor);
      } else if (currentMode == MODE_WAVE) {
        // displayPOVWaves now includes the waveOffset update
        displayPOVWaves(currentColor);
      } else {  // MODE_CLOCK
        displayPOVClock(currentColor);
      }
    }
  }
}