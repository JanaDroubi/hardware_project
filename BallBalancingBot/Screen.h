#ifndef SCREEN_H
#define SCREEN_H

#include <Arduino.h>
#include <TouchScreen.h>

// --- Touchscreen pins ---
#define YP 16  // Analog pin
#define XM 15  // Analog pin
#define YM 14
#define XP 17

// --- Touchscreen calibration ---
#define TS_MINX 60
#define TS_MAXX 963
#define TS_MINY 80
#define TS_MAXY 947

// --- Screen dimensions in mm ---
#define SCREEN_WIDTH_MM 167.0
#define SCREEN_HEIGHT_MM 135.5

// --- Pressure thresholds (optional, not used in mm conversion) ---
#define MINPRESSURE 0.000000000001
#define MAXPRESSURE 500

// --- Struct to store coordinates ---
struct coords {
  double x_mm;
  double y_mm;
  int z;  // 1 = detected, 0 = not detected
};

// --- Function prototypes ---
double mapf(double x, double in_min, double in_max, double out_min, double out_max);
void screen_init();
bool check_detected();
coords get_coords();

#endif
