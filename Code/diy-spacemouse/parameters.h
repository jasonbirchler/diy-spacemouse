// pins
#define QT_POWER_PIN 15 // This likely never needs to change for QtPy boards
#define BUTTON1_GPIO 27
#define BUTTON2_GPIO 24

// calibration
#define CALIBRATION_SAMPLES 300
#define XY_THRESHOLD 0.5  // Center threshold. i.e. how far from 0,0 is considered center
#define MAG_RANGE 3
#define SENSITIVITY 8
#define Z_FACTOR 1.6 // zThreshold = XY_THRESHOLD * Z_FACTOR