#include <QTRSensors.h>
#include <Servo.h>

// ==========================================
// 1. PIN DEFINITIONS & CONFIGURATIONS
// ==========================================

// Left Motor Driver Pins (IBT-2)
#define L_RPWM 5
#define L_LPWM 4
#define L_REN  24
#define L_LEN  25

// Right Motor Driver Pins (IBT-2)
#define R_RPWM 8
#define R_LPWM 9
#define R_REN  28
#define R_LEN  29

// IR Junction Sensor Pins
#define IR_LEFT  30
#define IR_RIGHT 31

// TCS3200 S1 Color Sensor Pins
#define S0_PIN  22
#define S1_PIN  23
#define S2_PIN  26
#define S3_PIN  27
#define OUT_PIN 28

// Target Square Color Sensor OE Pins (S2 - S5)
#define TCS2_OE 34
#define TCS3_OE 35
#define TCS4_OE 36
#define TCS5_OE 37

// Target Square Color Sensor OUT Pins (S2 - S5)
#define TCS2_OUT 38
#define TCS3_OUT 39
#define TCS4_OUT 40
#define TCS5_OUT 41

// Shooting Mechanism Pins & Calibration Angles
#define SERVO1_PIN  7
#define SERVO2_PIN  11
#define RELAY_PIN   26

#define SERVO1_TOP_ANGLE 100
#define SERVO1_BOT_ANGLE  70

#define SERVO_ANGLE_S2  120
#define SERVO_ANGLE_S3   75
#define SERVO_ANGLE_S4  120
#define SERVO_ANGLE_S5   75

// Speed Parameters (PWM 0 - 255)
#define BaseSpeed 90
#define MaxSpeed  250
#define SpeedTurn 100

// QTR Sensor Constants
#define LINE_THRESHOLD       600
#define QTR_CENTER_POSITION 3500

// PD Controller Constants
#define Kp 1
#define Kd 3

// Timing Delays
#define JUNCTION_LOCK_MIN_TIME 600
#define SCAN_NUM_READINGS        3

// ==========================================
// 2. GLOBAL OBJECTS & VARIABLES
// ==========================================

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

Servo servo1; // Up/Down aim
Servo servo2; // Horizontal aim

int lastError = 0;
uint16_t position = QTR_CENTER_POSITION;

int junctionCount = 0;
bool junctionLock = false;
unsigned long junctionLockTime = 0;
unsigned long ignoreIRUntil = 0;

struct ColorThresholds {
  float redToBlue;
  float greenToBlue;
  float redToGreen;
};

ColorThresholds s1blueThreshold  = {1.483, 1.441, 1.029};
ColorThresholds s1whiteThreshold = {1.113, 1.118, 0.996};
#define s1COLOR_TOLERANCE 0.2
bool isBlueDetected = false;

// State Machine Navigation States
enum RobotState {
  FOLLOW,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_RIGHT2,
  LEFT_DELAY_RIGHT,
  LEFT_DELAY_LEFT,
  DELAY_RIGHT,
  REVERSE_3_JUNCTIONS,
  TURN_RIGHT_AFTER_REVERSE,
  FORWARD_4_JUNCTIONS_NO_LINE,
  STRAIGHT_UNTIL_JUNCTION,
  TURN_RIGHT_AND_REVERSE,
  PARKED
};

RobotState state = FOLLOW;

// Detection Flags
bool blueSeenAtJunction6  = false;
bool blueSeenAtJunction8  = false;
bool blueSeenAtJunction10 = false;
bool blueSeenAtJunction12 = false;
bool blueSeenBefore9      = false;
bool blueSeenBefore11     = false;

// ==========================================
// 3. LOW-LEVEL MOTOR CONTROL FUNCTIONS
// ==========================================

void leftMotorForward(int pwm) {
  pwm = constrain(pwm, -255, 255);
  analogWrite(L_RPWM, pwm > 0 ? pwm : 0);
  analogWrite(L_LPWM, pwm < 0 ? -pwm : 0);
}

void rightMotorForward(int pwm) {
  pwm = constrain(pwm, -255, 255);
  // Reverse logic to match physical motor mounting
  analogWrite(R_RPWM, pwm < 0 ? -pwm : 0);
  analogWrite(R_LPWM, pwm > 0 ? pwm : 0);
}

void moveForward(int pwm) {
  leftMotorForward(pwm);
  rightMotorForward(pwm);
}

void moveBackward(int pwm) {
  leftMotorForward(-pwm);
  rightMotorForward(-pwm);
}

void stopMotors() {
  leftMotorForward(0);
  rightMotorForward(0);
}

// ==========================================
// 4. QTR SENSOR & PD LINE FOLLOWING
// ==========================================

void setupQTR() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  
  for (int i = 0; i < 50; i++) { 
    qtr.calibrate(); 
    delay(40); 
  }
}

bool isLineDetected() {
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > LINE_THRESHOLD) return true;
  }
  return false;
}

void executePDLineFollowing() {
  position = qtr.readLineBlack(sensorValues);
  int error = (int)position - QTR_CENTER_POSITION;
  
  int motorSpeed = (Kp * error) + (Kd * (error - lastError));
  motorSpeed = constrain(motorSpeed, -50, 50);
  lastError = error;
  
  leftMotorForward(constrain(BaseSpeed + motorSpeed, 0, MaxSpeed));
  rightMotorForward(constrain(BaseSpeed - motorSpeed, 0, MaxSpeed));
}

// ==========================================
// 5. JUNCTION DETECTION LOGIC
// ==========================================

void setupIRSensors() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
}

bool detectIRJunction() {
  if (millis() < ignoreIRUntil) return false;
  return (digitalRead(IR_LEFT) == HIGH || digitalRead(IR_RIGHT) == HIGH);
}

void updateJunctionLock() {
  if (junctionLock && (millis() - junctionLockTime > JUNCTION_LOCK_MIN_TIME)) {
    if (digitalRead(IR_LEFT) == LOW && digitalRead(IR_RIGHT) == LOW) {
      junctionLock = false;
    }
  }
}

// ==========================================
// 6. S1 COLOR SENSOR READING
// ==========================================

long readColor(bool s2State, bool s3State) {
  digitalWrite(S2_PIN, s2State);
  digitalWrite(S3_PIN, s3State);
  return pulseIn(OUT_PIN, LOW);
}

void readColorSensor() {
  long r_sum = 0, g_sum = 0, b_sum = 0;
  
  for (int i = 0; i < 6; i++) {
    r_sum += readColor(LOW,  LOW );
    g_sum += readColor(HIGH, HIGH);
    b_sum += readColor(LOW,  HIGH);
    delay(5);
  }
  
  float r = r_sum / 6.0;
  float g = g_sum / 6.0;
  float b = b_sum / 6.0;

  if (b == 0) return;
  
  float rToB = r / b;
  float gToB = g / b;
  float rToG = r / g;
  
  isBlueDetected = (
    abs(rToB - s1blueThreshold.redToBlue)   < (s1blueThreshold.redToBlue   * s1COLOR_TOLERANCE) &&
    abs(gToB - s1blueThreshold.greenToBlue) < (s1blueThreshold.greenToBlue * s1COLOR_TOLERANCE) &&
    abs(rToG - s1blueThreshold.redToGreen) < (s1blueThreshold.redToGreen * s1COLOR_TOLERANCE)
  );
}

// ==========================================
// 7. SHOOTING & TARGET SCANNING SYSTEM
// ==========================================

void setupShootingMechanism() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Active LOW relay
  
  servo1.write(90);
  servo2.write(90);

  pinMode(TCS2_OE, OUTPUT);
  pinMode(TCS3_OE, OUTPUT);
  pinMode(TCS4_OE, OUTPUT);
  pinMode(TCS5_OE, OUTPUT);
  disableAllSquare();
}

void servoMoveTo(Servo &srv, int targetAngle, int stepDelay = 15) {
  int current = srv.read();
  if (current < targetAngle) {
    for (int a = current; a <= targetAngle; a++) { srv.write(a); delay(stepDelay); }
  } else {
    for (int a = current; a >= targetAngle; a--) { srv.write(a); delay(stepDelay); }
  }
}

void relayOn()  { digitalWrite(RELAY_PIN, LOW);  }
void relayOff() { digitalWrite(RELAY_PIN, HIGH); }

void aimAndFire(int sensorNum) {
  int servo1Angle, servo2Angle;
  
  switch (sensorNum) {
    case 2: servo1Angle = SERVO1_TOP_ANGLE; servo2Angle = SERVO_ANGLE_S2; break;
    case 3: servo1Angle = SERVO1_TOP_ANGLE; servo2Angle = SERVO_ANGLE_S3; break;
    case 4: servo1Angle = SERVO1_BOT_ANGLE; servo2Angle = SERVO_ANGLE_S4; break;
    case 5: servo1Angle = SERVO1_BOT_ANGLE; servo2Angle = SERVO_ANGLE_S5; break;
    default: return;
  }
  
  servoMoveTo(servo1, servo1Angle);
  servoMoveTo(servo2, servo2Angle);
  delay(200);
  
  relayOn();
  delay(100);
  relayOff();
  delay(200);
}

void disableAllSquare() {
  digitalWrite(TCS2_OE, HIGH);
  digitalWrite(TCS3_OE, HIGH);
  digitalWrite(TCS4_OE, HIGH);
  digitalWrite(TCS5_OE, HIGH);
}

void enableSquareSensor(int sensorNum) {
  disableAllSquare();
  switch (sensorNum) {
    case 2: digitalWrite(TCS2_OE, LOW); break;
    case 3: digitalWrite(TCS3_OE, LOW); break;
    case 4: digitalWrite(TCS4_OE, LOW); break;
    case 5: digitalWrite(TCS5_OE, LOW); break;
  }
  delay(10);
}

bool scanSensorAveraged(int outPin, int oePin, int sensorNum) {
  enableSquareSensor(sensorNum);
  long pulseSum = 0;
  for (int i = 0; i < SCAN_NUM_READINGS; i++) {
    pulseSum += pulseIn(outPin, LOW);
    delay(20);
  }
  disableAllSquare();
  long avgPulse = pulseSum / SCAN_NUM_READINGS;
  return (avgPulse < 1000); 
}

void scanAndFireBlueTargets() {
  bool sensorIsBlue[6] = {false};
  
  sensorIsBlue[2] = scanSensorAveraged(TCS2_OUT, TCS2_OE, 2);
  sensorIsBlue[3] = scanSensorAveraged(TCS3_OUT, TCS3_OE, 3);
  sensorIsBlue[4] = scanSensorAveraged(TCS4_OUT, TCS4_OE, 4);
  sensorIsBlue[5] = scanSensorAveraged(TCS5_OUT, TCS5_OE, 5);
  
  disableAllSquare();
  
  moveBackward(50);
  delay(800);
  stopMotors();
  delay(200);
  
  for (int s = 2; s <= 5; s++) {
    if (sensorIsBlue[s]) {
      aimAndFire(s);
    }
  }
  
  servoMoveTo(servo1, 90);
  servoMoveTo(servo2, 90);
}

void blueDetectedAction() {
  scanAndFireBlueTargets();
  delay(6000); 
}

// ==========================================
// 8. NAVIGATION STATE MACHINE EXECUTION
// ==========================================

void executeNavigationState() {
  switch (state) {
    case FOLLOW:
      executePDLineFollowing();
      break;

    case TURN_LEFT:
      leftMotorForward(-100);
      rightMotorForward(100);
      delay(400);
      state = FOLLOW;
      break;

    case TURN_RIGHT:
      leftMotorForward(100);
      rightMotorForward(-100);
      delay(520);
      state = FOLLOW;
      break;

    case LEFT_DELAY_RIGHT:
      blueDetectedAction();
      state = TURN_RIGHT;
      break;

    case LEFT_DELAY_LEFT:
      blueDetectedAction();
      state = TURN_LEFT;
      break;

    case PARKED:
      stopMotors();
      break;

    default:
      executePDLineFollowing();
      break;
  }
}

// ==========================================
// 9. MAIN ARDUINO SETUP & LOOP
// ==========================================

void setup() {
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(L_REN,  OUTPUT); pinMode(L_LEN,  OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(R_REN,  OUTPUT); pinMode(R_LEN,  OUTPUT);

  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);

  setupQTR();
  setupIRSensors();
  setupShootingMechanism();
}

void loop() {
  updateJunctionLock();

  if (detectIRJunction() && !junctionLock && state == FOLLOW) {
    junctionCount++;
    junctionLock = true;
    junctionLockTime = millis();
    stopMotors();

    delay(150); 
    readColorSensor();

    switch (junctionCount) {
      case 3:
        state = TURN_LEFT;
        break;

      case 6:
        if (isBlueDetected) {
          blueSeenAtJunction6 = true;
          blueSeenBefore9 = true;
          blueSeenBefore11 = true;
          state = LEFT_DELAY_RIGHT;
        }
        break;

      case 8:
        if (isBlueDetected) {
          blueSeenAtJunction8 = true;
          blueSeenBefore9 = true;
          blueSeenBefore11 = true;
          state = LEFT_DELAY_RIGHT;
        }
        break;

      case 9:
        if (blueSeenBefore9) {
          state = TURN_RIGHT;
        } else {
          state = TURN_LEFT; 
        }
        break;

      case 10:
        if (isBlueDetected) {
          blueSeenAtJunction10 = true;
          blueSeenBefore11 = true;
          junctionCount -= 2;
          state = LEFT_DELAY_LEFT;
        }
        break;

      case 12:
        if (!blueSeenBefore11 && isBlueDetected) {
          blueSeenAtJunction12 = true;
          junctionCount -= 6;
          state = LEFT_DELAY_LEFT;
        } else {
          state = TURN_LEFT;
        }
        break;

      case 13:
        state = PARKED;
        break;

      default:
        state = FOLLOW;
        break;
    }
  }

  executeNavigationState();
}
