#include "Screen.h"

// --- Last known coordinates ---
static double last_x = 0;
static double last_y = 0;

// --- Initialize screen (serial) ---
void screen_init() {
    Serial.begin(115200);  // Must match Python
    while (!Serial) delay(10);
    Serial.println("✅ Ready to receive camera coordinates...");
}

// --- Check if new coordinates are available ---
bool check_detected() {
    return Serial.available() > 0;
}

// --- Get coordinates in mm from Python ---
coords get_coords() {
    coords p;
    String data = "";

    if (Serial.available()) {
        // Read a full line like "12.34,5.67\n"
        data = Serial.readStringUntil('\n');
        int commaIndex = data.indexOf(',');
        if (commaIndex > 0) {
            float x_mm = data.substring(0, commaIndex).toFloat();
            float y_mm = data.substring(commaIndex + 1).toFloat();

            // Save last known
            last_x = x_mm;
            last_y = y_mm;

            p.x_mm = x_mm;
            p.y_mm = y_mm;
            p.z = 1;  // detected

            // Optional: acknowledge back to Python
            Serial.print("ACK:");
            Serial.print(x_mm);
            Serial.print(",");
            Serial.println(y_mm);
        }
    } else {
        // No new data — use last known
        p.x_mm = last_x;
        p.y_mm = last_y;
        p.z = 0;
    }

    return p;
}
