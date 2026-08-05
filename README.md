# Mobile-blooth-car-
/*
  ESP32 Follower Car – 4 wheels / 1 L298N (two motors parallel per channel)
  Hardware: L298N + MPU-6050
*/

#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

#define PI 3.14159265358979323846

BluetoothSerial BT;
const char* BT_NAME = "ESP32-CAR";

// Motor pins (L298N)
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define ENA 32
#define ENB 33

MPU6050 mpu;
float pitch = 0, roll = 0;
float gyroXoffset = 0, gyroYoffset = 0;
bool imuCalibrated = false;

int carSpeed = 180;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 3000;
bool emergencyStop = false;

int currentLeft = 0, currentRight = 0;
int targetLeft = 0, targetRight = 0;

void motorRaw(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(IN1, left >= 0 ? HIGH : LOW);
  digitalWrite(IN2, left >= 0 ? LOW : HIGH);
  digitalWrite(IN3, right >= 0 ? HIGH : LOW);
  digitalWrite(IN4, right >= 0 ? LOW : HIGH);
  analogWrite(ENA, abs(left));
  analogWrite(ENB, abs(right));
}

void updateMotors() {
  int step = 30;
  if (currentLeft < targetLeft) currentLeft = min(targetLeft, currentLeft + step);
  if (currentLeft > targetLeft) currentLeft = max(targetLeft, currentLeft - step);
  if (currentRight < targetRight) currentRight = min(targetRight, currentRight + step);
  if (currentRight > targetRight) currentRight = max(targetRight, currentRight - step);
  motorRaw(currentLeft, currentRight);
}

void forward()  { targetLeft = carSpeed; targetRight = carSpeed; }
void backward() { targetLeft = -carSpeed; targetRight = -carSpeed; }
void turnLeft() { targetLeft = -carSpeed; targetRight = carSpeed; }
void turnRight(){ targetLeft = carSpeed; targetRight = -carSpeed; }
void stopCar()  { targetLeft = 0; targetRight = 0; currentLeft = 0; currentRight = 0; motorRaw(0, 0); }

void calibrateIMU() {
  Serial.println("Calibrating IMU...");
  if (BT.hasClient()) BT.println("CALIBRATING IMU...");
  int16_t ax, ay, az, gx, gy, gz;
  long gxSum = 0, gySum = 0;
  for (int i = 0; i < 500; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gxSum += gx;
    gySum += gy;
    delay(2);
  }
  gyroXoffset = gxSum / 500;
  gyroYoffset = gySum / 500;
  imuCalibrated = true;
  pitch = 0; roll = 0;
  if (BT.hasClient()) BT.println("IMU CALIBRATED");
}

void readIMU() {
  if (!imuCalibrated) return;
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gx -= gyroXoffset;
  gy -= gyroYoffset;
  float dt = 0.02;
  float accPitch = atan2(ay, az) * 180.0 / PI;
  float accRoll = atan2(ax, az) * 180.0 / PI;
  pitch = 0.98 * (pitch + (gx / 131.0) * dt) + 0.02 * accPitch;
  roll  = 0.98 * (roll  + (gy / 131.0) * dt) + 0.02 * accRoll;
}

void handleCommand(char cmd) {
  lastCommandTime = millis();
  if (emergencyStop && cmd != 'E') return;
  switch (cmd) {
    case 'F': case 'f': forward();   if (BT.hasClient()) BT.println("OK:FWD"); break;
    case 'B': case 'b': backward();  if (BT.hasClient()) BT.println("OK:BWD"); break;
    case 'L': case 'l': turnLeft();  if (BT.hasClient()) BT.println("OK:LEFT"); break;
    case 'R': case 'r': turnRight(); if (BT.hasClient()) BT.println("OK:RIGHT"); break;
    case 'S': case 's': stopCar();   if (BT.hasClient()) BT.println("OK:STOP"); break;
    case 'E': case 'e': emergencyStop = false; stopCar(); BT.println("OK:EMERGENCY RESET"); break;
    case '+': carSpeed = min(255, carSpeed + 25); BT.printf("SPD:%d\n", carSpeed); break;
    case '-': carSpeed = max(0, carSpeed - 25); BT.printf("SPD:%d\n", carSpeed); break;
    case '0'...'9': carSpeed = map(cmd - '0', 0, 9, 0, 255); BT.printf("SPD:%d\n", carSpeed); break;
    case 'T': case 't': BT.printf("PITCH:%.1f,ROLL:%.1f,SPD:%d\n", pitch, roll, carSpeed); break;
    case 'C': case 'c': calibrateIMU(); BT.println("CALIBRATION DONE"); break;
  }
}

void checkEmergencyButton() {
  static bool lastButtonState = HIGH;
  bool buttonState = digitalRead(0);
  if (buttonState == LOW && lastButtonState == HIGH) {
    emergencyStop = true;
    stopCar();
    if (BT.hasClient()) BT.println("EMERGENCY STOP!");
  }
  lastButtonState = buttonState;
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(0, INPUT_PULLUP);
  stopCar();

  Wire.begin(21, 22);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU-6050 OK");
    calibrateIMU();
  } else {
    Serial.println("MPU-6050 FAIL");
    imuCalibrated = false;
  }

  BT.begin(BT_NAME);
  Serial.println("Bluetooth ready. Waiting for commands...");
}

void loop() {
  checkEmergencyButton();
  if (BT.hasClient()) {
    while (BT.available()) handleCommand((char)BT.read());
    if (lastCommandTime > 0 && millis() - lastCommandTime > COMMAND_TIMEOUT) {
      if (targetLeft != 0 || targetRight != 0) stopCar();
      lastCommandTime = 0;
    }
    updateMotors();
    static unsigned long lastImu = 0;
    if (millis() - lastImu >= 20) { lastImu = millis(); readIMU(); }
  } else {
    if (targetLeft != 0 || targetRight != 0) stopCar();
    delay(100);
  }
  delay(10);
}
