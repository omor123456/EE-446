// Task 11 — Smart indoor event detector
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_HS300x.h>

// ---------- Raw values ----------
float rh = 0, temp = 0;
float mx = 0, my = 0, mz = 0;
int   r = 0, g = 0, b = 0, clearVal = 0;

// Previous magnetometer readings ----------
float mxPrev = 0, myPrev = 0, mzPrev = 0;

// ---------- Baselines (humidity, temp, light) ----------
float rhBase = 0, tempBase = 0;
int clearBase = 0;

// ---------- Flags ----------
bool humid_jump            = false;
bool temp_rise             = false;
bool mag_shift             = false;
bool light_or_color_change = false;

// ---------- THRESHOLDS ----------
const float RH_THRESHOLD    = 2.0f;   
const float TEMP_THRESHOLD  = 0.3f;    
const float MAG_THRESHOLD   = 10.0f;   // µT change on any axis
const int   CLEAR_THRESHOLD = 30;     

// ---------- Timing ----------
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL_MS = 1000;   // 1 s between prints

unsigned long lastEventTime = 0;
String lastEvent = "";
const unsigned long COOLDOWN_MS = 4000;  

String state = "BASELINE_NORMAL";

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!APDS.begin())   { Serial.println("APDS failed!");            while (1); }
  if (!IMU.begin())    { Serial.println("Failed to initialize IMU."); while (1); }
  if (!HS300x.begin()) { Serial.println("HS300x failed!");          while (1); }

  Serial.println("Warming up 2 sec...");
  delay(2000);

  // baseline for humidity, temp, light
  rhBase   = HS300x.readHumidity();
  tempBase = HS300x.readTemperature();

  int rr, gg, bb, cc;
  while (!APDS.colorAvailable());
  APDS.readColor(rr, gg, bb, cc);
  clearBase = cc;

  // seed magnetometer previous values
  while (!IMU.magneticFieldAvailable());
  IMU.readMagneticField(mx, my, mz);
  mxPrev = mx;
  myPrev = my;
  mzPrev = mz;

  Serial.print("Baseline: rh="); Serial.print(rhBase);
  Serial.print(" temp=");        Serial.print(tempBase);
  Serial.print(" clear=");       Serial.print(clearBase);
  Serial.print(" mx=");          Serial.print(mxPrev, 2);
  Serial.print(" my=");          Serial.print(myPrev, 2);
  Serial.print(" mz=");          Serial.println(mzPrev, 2);
}

void loop() {
  // ---------- READ SENSORS ----------
  rh   = HS300x.readHumidity();
  temp = HS300x.readTemperature();

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }

  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b, clearVal);
  }

  // ---------- FLAGS ----------
  humid_jump = (rh - rhBase)     > RH_THRESHOLD;
  temp_rise  = (temp - tempBase) > TEMP_THRESHOLD;

  // magnetic — true if ANY axis changed by more than threshold
  mag_shift = (fabs(mx - mxPrev) > MAG_THRESHOLD) ||
              (fabs(my - myPrev) > MAG_THRESHOLD) ||
              (fabs(mz - mzPrev) > MAG_THRESHOLD);

  // only update prev when a real change happened, so slow drift doesn't reset it
  if (mag_shift) {
    mxPrev = mx;
    myPrev = my;
    mzPrev = mz;
  }

  light_or_color_change = abs(clearVal - clearBase) > CLEAR_THRESHOLD;

  // ---------- CLASSIFY ----------
  String candidate = "BASELINE_NORMAL";
  if      (humid_jump && temp_rise)  candidate = "BREATH_OR_WARM_AIR_EVENT";
  else if (mag_shift)                candidate = "MAGNETIC_DISTURBANCE_EVENT";
  else if (light_or_color_change)    candidate = "LIGHT_OR_COLOR_CHANGE_EVENT";

  // ---------- COOLDOWN ----------
  unsigned long now = millis();
  if (candidate != "BASELINE_NORMAL") {
    if (candidate == lastEvent && (now - lastEventTime) < COOLDOWN_MS) {
      state = "BASELINE_NORMAL";
    } else {
      state = candidate;
      lastEvent = candidate;
      lastEventTime = now;
    }
  } else {
    state = "BASELINE_NORMAL";
  }

  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    Serial.print("raw,rh=");   Serial.print(rh, 1);
    Serial.print(",temp=");    Serial.print(temp, 1);
    Serial.print(",mx=");      Serial.print(mx, 2);
    Serial.print(",my=");      Serial.print(my, 2);
    Serial.print(",mz=");      Serial.print(mz, 2);
    Serial.print(",clear=");   Serial.println(clearVal);

    Serial.print("flags,humid_jump=");        Serial.print(humid_jump);
    Serial.print(",temp_rise=");              Serial.print(temp_rise);
    Serial.print(",mag_shift=");              Serial.print(mag_shift);
    Serial.print(",light_or_color_change=");  Serial.println(light_or_color_change);

    Serial.print("event,"); Serial.println(state);
  }
}