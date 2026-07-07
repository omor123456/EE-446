#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <PDM.h>

// ---------- Audio ----------
short sampleBuffer[256];
volatile int samplesRead = 0;

void onPDMData() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

// ---------- Raw sensor values ----------
int   micLevel       = 0;
int   lightValue     = 0;      // APDS clear channel
float motionValue    = 1.0f;   // accel magnitude assuming of gravity too
int   proximityValue = 255;    // 0 = close, 255 = far

// ---------- Binary flags ----------
bool sound    = false;
bool dark     = false;
bool moving   = false;
bool nearHand = false;

// ---------- Thresholds ----------
const int   MIC_THRESHOLD    = 15;
const int   DARK_THRESHOLD   = 100;
const float MOTION_THRESHOLD = 0.20f;   // doe deviation for 
const int   PROX_THRESHOLD   = 100;   

unsigned long lastPrint = 0;   
const unsigned long PRINT_INTERVAL_MS = 2000; //break time
String state = "";


void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!APDS.begin()) { Serial.println("APDS9960 failed!"); while (1); }
  if (!IMU.begin())  { Serial.println("IMU failed!");      while (1); }

  PDM.onReceive(onPDMData);
  if (!PDM.begin(1, 16000)) { Serial.println("PDM failed!"); while (1); }

  Serial.println("Workspace detection started.");
}

void loop() {
  // ---------- MICROPHONE ----------
  if (samplesRead > 0) {
    long sum = 0;
    for (int i = 0; i < samplesRead; i++) sum += abs(sampleBuffer[i]);
    micLevel = sum / samplesRead;
    samplesRead = 0;
  }

  // ---------- LIGHT ----------
  if (APDS.colorAvailable()) {
    int r, g, b, c;
    APDS.readColor(r, g, b, c);
    lightValue = c;   // clear channel = ambient brightness
  }

  // ---------- PROXIMITY ----------
  if (APDS.proximityAvailable()) {
    proximityValue = APDS.readProximity();   // 0 close, 255 far
  }

  // ---------- MOTION ----------
  if (IMU.accelerationAvailable()) {
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    motionValue = sqrt(ax*ax + ay*ay + az*az);  //so we can get the threshold 
  }

  sound    = (micLevel > MIC_THRESHOLD);
  dark     = (lightValue < DARK_THRESHOLD);
  moving   = (fabs(motionValue - 1.0f) > MOTION_THRESHOLD);
  nearHand = (proximityValue < PROX_THRESHOLD);   // inverted

  // STATE CLASSIFICATION
  if      (!sound && !dark && !moving && !nearHand) state = "QUIET_BRIGHT_STEADY_FAR";
  else if ( sound && !dark && !moving && !nearHand) state = "NOISY_BRIGHT_STEADY_FAR";
  else if (!sound &&  dark && !moving &&  nearHand) state = "QUIET_DARK_STEADY_NEAR";
  else if ( sound && !dark &&  moving &&  nearHand) state = "NOISY_BRIGHT_MOVING_NEAR";
  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();

    Serial.print("raw,mic=");      Serial.print(micLevel);
    Serial.print(",clear=");       Serial.print(lightValue);
    Serial.print(",motion=");      Serial.print(motionValue, 3);
    Serial.print(",prox=");        Serial.println(proximityValue);

    Serial.print("flags,sound=");  Serial.print(sound    ? 1 : 0);
    Serial.print(",dark=");        Serial.print(dark     ? 1 : 0);
    Serial.print(",moving=");      Serial.print(moving   ? 1 : 0);
    Serial.print(",near=");        Serial.println(nearHand ? 1 : 0);

    Serial.print("state,");        Serial.println(state);
  }
}