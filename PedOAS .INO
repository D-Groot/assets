#include <Servo.h>

// =====================================================
// 1. PIN CONFIGURATION - NODEMCU ESP8266
// =====================================================

#define IR_LEFT       D2
#define IR_CENTER     D5
#define IR_RIGHT      D6

#define SERVO_PIN     D1
#define MOTOR_ENA     D7

#define DETECTED_STATE LOW

// =====================================================
// 2. SERVO CONFIGURATION
// =====================================================

Servo fanServo;

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int CENTER_ANGLE = 90;

const unsigned long NORMAL_SWING = 30;
const unsigned long SLOW_SWING = 80;

int currentAngle = CENTER_ANGLE;
int swingDirection = 1;

unsigned long lastServoStepTime = 0;

// =====================================================
// 3. MOTOR CONFIGURATION
// =====================================================

const int NORMAL_PWM = 400;
const int MAX_PWM = 1023;

int currentPWM = 0;

// =====================================================
// 4. OCCUPANCY AND TIMING SETTINGS
// =====================================================

const unsigned long NO_MOTION_TIME = 15000;
const unsigned long FULL_OCCUPANCY_TIME = 5000;
const unsigned long DEBOUNCE_MS = 60;

unsigned long lastDetectionTime = 0;
unsigned long allSensorsActiveSince = 0;

bool fanOn = false;
bool startupGracePeriod = true;

bool fullOccupancy = false;
bool previousFullOccupancy = false;

// =====================================================
// 5. SENSOR VARIABLES
// =====================================================

const int SENSOR_COUNT = 3;

const int sensorPins[SENSOR_COUNT] = {
  IR_LEFT,
  IR_CENTER,
  IR_RIGHT
};

bool sensorState[SENSOR_COUNT] = {
  false, false, false
};

bool lastRawState[SENSOR_COUNT] = {
  false, false, false
};

unsigned long lastDebounceTime[SENSOR_COUNT] = {
  0, 0, 0
};

bool anyDetection = false;

// =====================================================
// 6. OPERATING MODES
// =====================================================

enum OperatingMode {
  ADAPTIVE,
  BASELINE
};

OperatingMode mode = ADAPTIVE;

// =====================================================
// 7. READ SENSORS WITH DEBOUNCE
// =====================================================

void readSensors() {

  anyDetection = false;

  for (int i = 0; i < SENSOR_COUNT; i++) {

    bool rawDetection =
      (digitalRead(sensorPins[i]) == DETECTED_STATE);

    if (rawDetection != lastRawState[i]) {

      lastDebounceTime[i] = millis();

      lastRawState[i] = rawDetection;
    }

    if (millis() - lastDebounceTime[i] >= DEBOUNCE_MS) {

      sensorState[i] = rawDetection;
    }

    if (sensorState[i]) {
      anyDetection = true;
    }
  }
}

// =====================================================
// 8. FULL OCCUPANCY DETECTION
// =====================================================

void updateFullOccupancy() {

  bool allActive =
    sensorState[0] &&
    sensorState[1] &&
    sensorState[2];

  if (allActive) {

    if (allSensorsActiveSince == 0) {
      allSensorsActiveSince = millis();
    }

    if (millis() - allSensorsActiveSince >= FULL_OCCUPANCY_TIME) {

      fullOccupancy = true;
    }

  } else {

    allSensorsActiveSince = 0;
    fullOccupancy = false;
  }

  if (fullOccupancy != previousFullOccupancy) {

    if (fullOccupancy) {

      Serial.println();
      Serial.println("FULL OCCUPANCY DETECTED");
      Serial.println("All three zones are occupied.");

    } else {

      Serial.println("Full occupancy condition cleared.");
    }

    previousFullOccupancy = fullOccupancy;
  }
}

// =====================================================
// 9. FAN STARTUP AND AUTOMATIC SHUTDOWN
// =====================================================

void updateFanState() {

  // Startup: motor runs immediately at normal speed
  // even when no object is detected.

  if (startupGracePeriod) {

    fanOn = true;

    if (anyDetection) {

      startupGracePeriod = false;
      lastDetectionTime = millis();

      Serial.println("Occupancy detected.");
      Serial.println("Adaptive operation activated.");

    }

    else if (millis() - lastDetectionTime >= NO_MOTION_TIME) {

      fanOn = false;
      startupGracePeriod = false;

      Serial.println();
      Serial.println("No occupancy detected for 15 seconds.");
      Serial.println("Automatic shutdown activated.");
      Serial.println("Fan OFF");
    }

    return;
  }

  // Normal operation after startup

  if (anyDetection) {

    lastDetectionTime = millis();

    if (!fanOn) {

      fanOn = true;

      Serial.println("Occupancy detected: Fan ON");
    }

  } else {

    if (fanOn &&
        millis() - lastDetectionTime >= NO_MOTION_TIME) {

      fanOn = false;

      Serial.println();
      Serial.println("No occupancy for 15 seconds.");
      Serial.println("Automatic shutdown activated.");
      Serial.println("Fan OFF");
    }
  }
}

// =====================================================
// 10. CHECK OCCUPANCY IN CURRENT SERVO ZONE
// =====================================================

bool isOccupied(int angle) {

  if (fullOccupancy) {
    return true;
  }

  // Left zone: 0 to 60 degrees
  if (angle <= 60 && sensorState[0]) {
    return true;
  }

  // Center zone: 61 to 120 degrees
  if (angle > 60 && angle <= 120 && sensorState[1]) {
    return true;
  }

  // Right zone: 121 to 180 degrees
  if (angle > 120 && sensorState[2]) {
    return true;
  }

  return false;
}

// =====================================================
// 11. SERVO SWING CONTROL
// =====================================================

void updateServo() {

  // Return to center when fan is OFF

  if (!fanOn) {

    if (currentAngle != CENTER_ANGLE) {

      currentAngle = CENTER_ANGLE;

      fanServo.write(CENTER_ANGLE);

      Serial.println("Servo returned to center.");
    }

    return;
  }

  unsigned long movementDelay = NORMAL_SWING;

  // Slow swing only in occupied zones in adaptive mode

  if (mode == ADAPTIVE && isOccupied(currentAngle)) {

    movementDelay = SLOW_SWING;
  }

  // Constant-speed sweep in baseline mode

  if (millis() - lastServoStepTime >= movementDelay) {

    lastServoStepTime = millis();

    currentAngle += swingDirection;

    if (currentAngle >= MAX_ANGLE) {

      currentAngle = MAX_ANGLE;
      swingDirection = -1;

    } else if (currentAngle <= MIN_ANGLE) {

      currentAngle = MIN_ANGLE;
      swingDirection = 1;
    }

    fanServo.write(currentAngle);
  }
}

// =====================================================
// 12. MOTOR PWM CONTROL
// =====================================================

void setMotorPWM(int pwmValue) {

  pwmValue = constrain(pwmValue, 0, MAX_PWM);

  if (pwmValue != currentPWM) {

    currentPWM = pwmValue;

    analogWrite(MOTOR_ENA, currentPWM);

    Serial.print("Motor PWM: ");
    Serial.println(currentPWM);
  }
}

void updateMotor() {

  // Fan OFF
  if (!fanOn) {

    setMotorPWM(0);
    return;
  }

  // During startup, run at normal speed
  // until occupancy is detected.

  if (startupGracePeriod) {

    setMotorPWM(NORMAL_PWM);
    return;
  }

  // Baseline mode: fixed normal speed

  if (mode == BASELINE) {

    setMotorPWM(NORMAL_PWM);
    return;
  }

  // Full occupancy

  if (fullOccupancy) {

    setMotorPWM(MAX_PWM);
    return;
  }

  // Adaptive mode

  if (isOccupied(currentAngle)) {

    setMotorPWM(MAX_PWM);

  } else {

    setMotorPWM(NORMAL_PWM);
  }
}

// =====================================================
// 13. SERIAL COMMANDS
// =====================================================

void handleSerial() {

  if (!Serial.available()) {
    return;
  }

  char command = Serial.read();

  switch (command) {

    case 'A':
    case 'a':

      mode = ADAPTIVE;

      Serial.println("ADAPTIVE MODE ACTIVATED");
      break;

    case 'B':
    case 'b':

      mode = BASELINE;

      Serial.println("BASELINE MODE ACTIVATED");
      break;

    case 'S':
    case 's':

      Serial.println();
      Serial.println("------- SYSTEM STATUS -------");

      Serial.print("Operating Mode: ");
      Serial.println(
        mode == ADAPTIVE ? "ADAPTIVE" : "BASELINE"
      );

      Serial.print("Fan Status: ");
      Serial.println(fanOn ? "ON" : "OFF");

      Serial.print("Startup Grace Period: ");
      Serial.println(startupGracePeriod ? "ACTIVE" : "INACTIVE");

      Serial.print("Left Sensor: ");
      Serial.println(sensorState[0] ? "DETECTED" : "CLEAR");

      Serial.print("Center Sensor: ");
      Serial.println(sensorState[1] ? "DETECTED" : "CLEAR");

      Serial.print("Right Sensor: ");
      Serial.println(sensorState[2] ? "DETECTED" : "CLEAR");

      Serial.print("Full Occupancy: ");
      Serial.println(fullOccupancy ? "YES" : "NO");

      Serial.print("Servo Angle: ");
      Serial.println(currentAngle);

      Serial.print("Motor PWM: ");
      Serial.println(currentPWM);

      Serial.println("-----------------------------");
      break;

    default:
      break;
  }
}

// =====================================================
// 14. SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // Configure IR sensors

  for (int i = 0; i < SENSOR_COUNT; i++) {

    pinMode(sensorPins[i], INPUT_PULLUP);

    bool initialState =
      (digitalRead(sensorPins[i]) == DETECTED_STATE);

    sensorState[i] = initialState;
    lastRawState[i] = initialState;

    lastDebounceTime[i] = millis();
  }

  // Configure L298N motor driver

  pinMode(MOTOR_ENA, OUTPUT);

  analogWriteRange(MAX_PWM);
  analogWriteFreq(1000);

  analogWrite(MOTOR_ENA, 0);

  // Initialize servo

  fanServo.attach(SERVO_PIN);

  currentAngle = CENTER_ANGLE;
  fanServo.write(CENTER_ANGLE);

  // Initialize startup timer

  lastDetectionTime = millis();
  lastServoStepTime = millis();

  startupGracePeriod = true;

  // Start fan immediately after power ON

  fanOn = true;

  Serial.println();
  Serial.println("================================");
  Serial.println("       PedOAS SYSTEM v3.2");
  Serial.println("================================");

  Serial.println("NodeMCU ESP8266 Initialized");
  Serial.println("Three IR Sensors Initialized");
  Serial.println("Servo Initialized");
  Serial.println("L298N Motor Driver Initialized");

  Serial.println();
  Serial.println("STARTUP MODE:");
  Serial.println("Fan ON at normal PWM 400.");
  Serial.println("Normal servo sweep started.");
  Serial.println("Waiting for occupancy.");

  Serial.println();
  Serial.println("Automatic shutdown: 15 seconds");
  Serial.println("Adaptive mode: ENABLED");

  Serial.println();
  Serial.println("Commands:");
  Serial.println("A - Adaptive Mode");
  Serial.println("B - Baseline Mode");
  Serial.println("S - System Status");

  Serial.println("================================");
}

// =====================================================
// 15. MAIN LOOP
// =====================================================

void loop() {

  handleSerial();

  readSensors();

  updateFullOccupancy();

  updateFanState();

  updateServo();

  updateMotor();
}
