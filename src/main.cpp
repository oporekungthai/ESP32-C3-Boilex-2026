#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <esp_wifi.h>  // FIX 2: needed for esp_wifi_set_max_tx_power()

// --- PCA9685 Setup ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SDA_PIN 8
#define SCL_PIN 9
#define SERVO_FREQ 50
#define SERVOMIN 150
#define SERVOMAX 600

// --- PCA9685 Channel Map ---
// Ch 0 = Gimbal left/right
// Ch 1 = Gimbal up/down
// Ch 2 = Front steering
// Ch 3 = Back steering
#define CH_BASE_GIMBAL 0
#define CH_TILT_GIMBAL 1
#define CH_STEER_FRONT 2
#define CH_STEER_BACK  3

// --- ESP32-C3 Direct Pins (Drive servos only) ---
#define DRIVE_SERVO_1_PIN 0  // Front continuous servo
#define DRIVE_SERVO_2_PIN 1  // Back continuous servo

// --- Wi-Fi Settings ---
const char* ssid     = "EIC_Dawg";
const char* password = "123456789_Password";

WebSocketsServer webSocket = WebSocketsServer(81);

// --- Servo Objects (drive only) ---
Servo driveServo1;
Servo driveServo2;

// --- Helper: write angle to PCA9685 channel ---
void pca_writeAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

// --- ADJUSTABLE SPEED / CALIBRATION VARIABLES ---
int driveForwardSpeed = 180;
int driveReverseSpeed = 0;
int driveStopSpeed    = 90;

int baseCenter  = 90;
int tiltCenter  = 90;
int frontCenter = 65;
int backCenter  = 80;

const int MIN_BASE   = 1;
const int MAX_BASE   = 179;
const int MIN_TILT   = 1;
const int MAX_TILT   = 179;
const int MIN_STEER  = 1;
const int MAX_STEER  = 179;
const int MIN_GIMBAL = 1;
const int MAX_GIMBAL = 179;


// --- STATE MACHINES ---
enum GimbalState { GIMBAL_HOLD, GIMBAL_LEFT, GIMBAL_RIGHT, GIMBAL_UP, GIMBAL_DOWN, GIMBAL_CENTER };
enum SteerState  { STATE_HOLD, STATE_LEFT, STATE_RIGHT, STATE_CENTER };
enum DriveState  { DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACKWARD };

GimbalState baseState         = GIMBAL_HOLD;
GimbalState tiltState         = GIMBAL_HOLD;
SteerState  frontState        = STATE_HOLD;
SteerState  backState         = STATE_HOLD;
DriveState  currentDriveState = DRIVE_STOP;

// --- Current real-time positions/speeds ---
int currentBaseAngle  = baseCenter;
int currentTiltAngle  = tiltCenter;
int currentFrontAngle = frontCenter;
int currentBackAngle  = backCenter;
int currentDriveSpeed = driveStopSpeed;

// --- TIMING / ACCELERATION SETTINGS ---
unsigned long lastGimbalUpdate = 0;
const unsigned long gimbalInterval = 15;
const int gimbalStep = 2;

unsigned long lastSteerUpdate = 0;
const unsigned long steerInterval = 12;
const int steerStep = 2;

unsigned long lastDriveUpdate = 0;
const unsigned long driveInterval = 18;
const int driveStep = 1;

// --- Setup ---
void setupServos() {
  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);

  driveServo1.attach(DRIVE_SERVO_1_PIN, 500, 2400);
  driveServo2.attach(DRIVE_SERVO_2_PIN, 500, 2400);

  currentBaseAngle  = baseCenter;
  currentTiltAngle  = tiltCenter;
  currentFrontAngle = frontCenter;
  currentBackAngle  = backCenter;
  currentDriveSpeed = driveStopSpeed;

  pca_writeAngle(CH_BASE_GIMBAL, currentBaseAngle);
  pca_writeAngle(CH_TILT_GIMBAL, currentTiltAngle);
  pca_writeAngle(CH_STEER_FRONT, currentFrontAngle);
  pca_writeAngle(CH_STEER_BACK,  currentBackAngle);
  driveServo1.write(currentDriveSpeed);
  driveServo2.write(currentDriveSpeed);
}

void homeServos() {
  currentBaseAngle  = baseCenter;
  currentTiltAngle  = tiltCenter;
  currentFrontAngle = frontCenter;
  currentBackAngle  = backCenter;
  currentDriveSpeed = driveStopSpeed;

  pca_writeAngle(CH_BASE_GIMBAL, currentBaseAngle);
  pca_writeAngle(CH_TILT_GIMBAL, currentTiltAngle);
  pca_writeAngle(CH_STEER_FRONT, currentFrontAngle);
  pca_writeAngle(CH_STEER_BACK,  currentBackAngle);
  driveServo1.write(currentDriveSpeed);
  driveServo2.write(currentDriveSpeed);

  // Reset all state machines
  currentDriveState = DRIVE_STOP;
  baseState  = GIMBAL_HOLD;
  tiltState  = GIMBAL_HOLD;
  frontState = STATE_HOLD;
  backState  = STATE_HOLD;
}

// --- Non-Blocking Drive Engine ---
void handleGradualDrive() {
  if (millis() - lastDriveUpdate >= driveInterval) {
    lastDriveUpdate = millis();

    int targetSpeed = driveStopSpeed;
    if (currentDriveState == DRIVE_FORWARD)  targetSpeed = driveForwardSpeed;
    if (currentDriveState == DRIVE_BACKWARD) targetSpeed = driveReverseSpeed;

    if      (currentDriveSpeed < targetSpeed) currentDriveSpeed += driveStep;
    else if (currentDriveSpeed > targetSpeed) currentDriveSpeed -= driveStep;

    if (abs(currentDriveSpeed - targetSpeed) < driveStep) currentDriveSpeed = targetSpeed;

    // Servo 2 is physically mirrored — invert around stop point so both wheels go same direction
    int speed1 = currentDriveSpeed;
    int speed2 = driveStopSpeed - (currentDriveSpeed - driveStopSpeed);

    driveServo1.write(speed1);
    driveServo2.write(speed2);
  }
}

// --- Non-Blocking Steering Engine ---
void handleGradualSteering() {
  if (millis() - lastSteerUpdate >= steerInterval) {
    lastSteerUpdate = millis();

    // FRONT
    if (frontState != STATE_HOLD) {
      int targetFront = currentFrontAngle;
      if (frontState == STATE_LEFT)   targetFront = MAX_STEER;
      if (frontState == STATE_RIGHT)  targetFront = MIN_STEER;
      if (frontState == STATE_CENTER) targetFront = frontCenter;

      if      (currentFrontAngle < targetFront) currentFrontAngle += steerStep;
      else if (currentFrontAngle > targetFront) currentFrontAngle -= steerStep;
      if (abs(currentFrontAngle - targetFront) < steerStep) currentFrontAngle = targetFront;

      pca_writeAngle(CH_STEER_FRONT, currentFrontAngle);

      if (frontState == STATE_CENTER && currentFrontAngle == targetFront)
        frontState = STATE_HOLD;
    }

    // BACK
    if (backState != STATE_HOLD) {
      int targetBack = currentBackAngle;
      if (backState == STATE_LEFT)   targetBack = MAX_STEER;
      if (backState == STATE_RIGHT)  targetBack = MIN_STEER;
      if (backState == STATE_CENTER) targetBack = backCenter;

      if      (currentBackAngle < targetBack) currentBackAngle += steerStep;
      else if (currentBackAngle > targetBack) currentBackAngle -= steerStep;
      if (abs(currentBackAngle - targetBack) < steerStep) currentBackAngle = targetBack;

      pca_writeAngle(CH_STEER_BACK, currentBackAngle);

      if (backState == STATE_CENTER && currentBackAngle == targetBack)
        backState = STATE_HOLD;
    }
  }
}

// --- Non-Blocking Gimbal Engine ---
void handleGradualGimbal() {
  if (millis() - lastGimbalUpdate >= gimbalInterval) {
    lastGimbalUpdate = millis();

    // BASE (left/right pan)
    if (baseState != GIMBAL_HOLD) {
      int targetBase = currentBaseAngle;
      if (baseState == GIMBAL_LEFT)   targetBase = MAX_GIMBAL;
      if (baseState == GIMBAL_RIGHT)  targetBase = MIN_GIMBAL;
      if (baseState == GIMBAL_CENTER) targetBase = baseCenter;

      if      (currentBaseAngle < targetBase) currentBaseAngle += gimbalStep;
      else if (currentBaseAngle > targetBase) currentBaseAngle -= gimbalStep;
      if (abs(currentBaseAngle - targetBase) < gimbalStep) currentBaseAngle = targetBase;

      pca_writeAngle(CH_BASE_GIMBAL, currentBaseAngle);

      if (baseState == GIMBAL_CENTER && currentBaseAngle == targetBase)
        baseState = GIMBAL_HOLD;
    }

    // TILT (up/down)
    if (tiltState != GIMBAL_HOLD) {
      int targetTilt = currentTiltAngle;
      if (tiltState == GIMBAL_UP)     targetTilt = MAX_TILT;
      if (tiltState == GIMBAL_DOWN)   targetTilt = MIN_TILT;
      if (tiltState == GIMBAL_CENTER) targetTilt = tiltCenter;

      if      (currentTiltAngle < targetTilt) currentTiltAngle += gimbalStep;
      else if (currentTiltAngle > targetTilt) currentTiltAngle -= gimbalStep;
      if (abs(currentTiltAngle - targetTilt) < gimbalStep) currentTiltAngle = targetTilt;

      pca_writeAngle(CH_TILT_GIMBAL, currentTiltAngle);

      if (tiltState == GIMBAL_CENTER && currentTiltAngle == targetTilt)
        tiltState = GIMBAL_HOLD;
    }
  }
}

// --- WebSocket Event Handler ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {

  // FIX 5: Safe-stop on disconnect — robot won't keep moving if connection drops
  if (type == WStype_DISCONNECTED) {
    homeServos();
    return;
  }

  if (type == WStype_TEXT) {
    String command = (char*)payload;

    // FIX 4: Ignore PING keepalives sent by the browser, reply with PONG
    if (command == "PING") {
      webSocket.sendTXT(num, "PONG");
      return;
    }

    // --- Drive ---
    if      (command == "FORWARD")  currentDriveState = DRIVE_FORWARD;
    else if (command == "BACKWARD") currentDriveState = DRIVE_BACKWARD;
    else if (command == "STOP") {
      currentDriveState = DRIVE_STOP;
      currentDriveSpeed = driveStopSpeed;
      driveServo1.write(driveStopSpeed);
      driveServo2.write(driveStopSpeed);
    }

    // --- Front Steering ---
    else if (command == "F_LEFT")   frontState = STATE_LEFT;
    else if (command == "F_RIGHT")  frontState = STATE_RIGHT;
    else if (command == "F_CENTER") frontState = STATE_CENTER;
    else if (command == "STOP_F")   frontState = STATE_HOLD;

    // --- Back Steering ---
    else if (command == "B_LEFT")   backState = STATE_LEFT;
    else if (command == "B_RIGHT")  backState = STATE_RIGHT;
    else if (command == "B_CENTER") backState = STATE_CENTER;
    else if (command == "STOP_B")   backState = STATE_HOLD;

    // --- Gimbal ---
    else if (command == "BASE_LEFT")   baseState = GIMBAL_LEFT;
    else if (command == "BASE_RIGHT")  baseState = GIMBAL_RIGHT;
    else if (command == "BASE_CENTER") baseState = GIMBAL_CENTER;
    else if (command == "TILT_UP")     tiltState = GIMBAL_UP;
    else if (command == "TILT_DOWN")   tiltState = GIMBAL_DOWN;
    else if (command == "TILT_CENTER") tiltState = GIMBAL_CENTER;
    else if (command == "STOP_GIMBAL") { baseState = GIMBAL_HOLD; tiltState = GIMBAL_HOLD; }

    // --- Home ---
    else if (command == "HOME") homeServos();
  }
}

void setup() {
  Serial.begin(115200);
  setupServos();
  Serial.println("Setup complete. Starting Wi-Fi...");

  // FIX 1: Lock to channel 6 (least-congested non-overlapping channel in most venues)
  //        Try channel 1 or 11 if 6 is still noisy in your specific environment
  WiFi.softAP(ssid, password, 6);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // FIX 2: Boost TX power to maximum (21 dBm) for better range through crowds
  esp_wifi_set_max_tx_power(84);  // 84 units = 21 dBm
  Serial.println("Wi-Fi started (ch6, max TX power).");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // FIX 4: Library-level heartbeat — pings client every 1s,
  //        disconnects if no pong within 2s, tolerates up to 3 missed pongs
  webSocket.enableHeartbeat(1000, 2000, 3);

  Serial.println("WebSocket server started on port 81.");
}

void loop() {
  webSocket.loop();
  handleGradualGimbal();
  handleGradualSteering();
  handleGradualDrive();
}