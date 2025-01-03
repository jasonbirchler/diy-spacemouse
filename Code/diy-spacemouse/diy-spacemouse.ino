#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <SimpleKalmanFilter.h>
#include "TLx493D_inc.hpp"
#include "parameters.h"

using namespace ifx::tlx493d;

const uint8_t POWER_PIN = QT_POWER_PIN;
TLx493D_A1B6 mag(Wire1, TLx493D_IIC_ADDR_A0_e);

SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);

// Setup buttons
OneButton button1(BUTTON1_GPIO, true);
OneButton button2(BUTTON2_GPIO, true);

double xOffset = 0, yOffset = 0, zOffset = 0;
double xRaw=0, yRaw = 0, zRaw = 0;
double xCurrent = 0, yCurrent = 0, zCurrent = 0;

int calSamples = CALIBRATION_SAMPLES;
int sensitivity = SENSITIVITY;
int magRange = MAG_RANGE;
int outRange = 127;       // Max allowed in HID report
double xyThreshold = XY_THRESHOLD;

int inRange = magRange * sensitivity;
double zThreshold = xyThreshold * Z_FACTOR;

// bool isOrbit = false;
// bool isPan = false;

enum Motion {
  MOVE,
  ORBIT,
  PAN
};

int currentState = MOVE;

void setup() {

  button1.attachClick(goHome);
  button1.attachLongPressStop(goHome);

  button2.attachClick(fitToScreen);
  button2.attachLongPressStop(fitToScreen);

  // mouse and keyboard init
  Mouse.begin();
  Keyboard.begin();

  Serial.begin(9600);
  Wire1.begin();

  // mag sensor init
  mag.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 0, 250000);
  mag.begin();

  // crude offset calibration on first boot
  for (int i = 1; i <= calSamples; i++) {

    mag.getMagneticField(&xRaw, &yRaw, &zRaw);

    xOffset += xRaw;
    yOffset += yRaw;
    zOffset += zRaw;

    Serial.print(".");
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println();
  Serial.println(xOffset);
  Serial.println(yOffset);
  Serial.println(zOffset);
}

void loop() {

  // keep watching the push buttons
  button1.tick();
  button2.tick();

  // get the mag data
  mag.getMagneticField(&xRaw, &yRaw, &zRaw);

  // update the filters
  xCurrent = xFilter.updateEstimate(xRaw - xOffset);
  yCurrent = yFilter.updateEstimate(yRaw - yOffset);
  zCurrent = zFilter.updateEstimate(zRaw - zOffset);

  // check the center threshold
  if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold) {

    switch (currentState) {
      case MOVE:
        Keyboard.releaseAll();
      case ORBIT:
        Keyboard.release(KEY_LEFT_SHIFT);
      case PAN:
        Keyboard.release(KEY_LEFT_ALT);
    }

    int xMove = 0;
    int yMove = 0;

    // map the magnetometer xy to the allowed 127 range in HID reports
    xMove = map(yCurrent, inRange, -inRange, -outRange, outRange);
    yMove = map(xCurrent, -inRange, inRange, -outRange, outRange);

    // orbit and panning
    if (abs(zCurrent) < zThreshold) { // if below the threshold, orbit
      Keyboard.press(KEY_LEFT_ALT);
      currentState = ORBIT;
    } else { // if over threshold, pan
      Keyboard.press(KEY_LEFT_SHIFT);
      currentState = PAN;
    }

    // move the cursor
    Mouse.move(yMove, xMove, 0);
  } else {

    // release the mouse and keyboard if within the center threshold
    Mouse.release(MOUSE_LEFT);
    Mouse.release(MOUSE_RIGHT);
    Mouse.release(MOUSE_MIDDLE);
    Keyboard.releaseAll();
    currentState = MOVE;
  }
}

// go to home view in FreeCAD by pressing HOME
void goHome() {
  Keyboard.press(KEY_HOME);

  delay(10);
  Keyboard.releaseAll();
  Serial.println("pressed home");
}

// in FreeCAD, fit all by "typing" v then f
void fitToScreen() {
  Keyboard.press('v');
  Keyboard.release('v');
  delay(10);
  Keyboard.press('f');
  Keyboard.release('f');

  Serial.println("pressed fit");
}
