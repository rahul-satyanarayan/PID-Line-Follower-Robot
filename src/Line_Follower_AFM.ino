#include <AFMotor.h>

// --- Pin Definitions ---
const uint8_t Sensor_L = A2;
const uint8_t Sensor_C = A1;
const uint8_t Sensor_R = A0;

// --- Speed Calibration ---
const uint8_t BASE_L = 135;  
const uint8_t BASE_R = 115; 
const int FIND_SPEED = 100;

// --- Your Preferred PID Values ---
float Kp = 4.9; 
float Ki = 0; 
float Kd = 15; 

float error = 0, lastError = 0;

AF_DCMotor motorLeft(3);
AF_DCMotor motorRight(2);

void setup() {
  pinMode(Sensor_L, INPUT);
  pinMode(Sensor_C, INPUT);
  pinMode(Sensor_R, INPUT);
  delay(2000); 
}

void loop() {
  bool L = digitalRead(Sensor_L);
  bool C = digitalRead(Sensor_C);
  bool R = digitalRead(Sensor_R);

  // --- 1. Linear Error Mapping ---
  // No jumps to "2" or "-2" logic here; keep it as a pure variable.
  if (!L && C && !R)      error = 0;
  else if (L && C && !R)  error = 1;
  else if (L && !C && !R) error = 2;
  else if (!L && C && R)  error = -1;
  else if (!L && !C && R) error = -2;
  else if (!L && !C && !R) {
    // Line Lost: Stay in current rotation until line found
    if (lastError > 0) pivot(BACKWARD, FORWARD, FIND_SPEED);
    else pivot(FORWARD, BACKWARD, FIND_SPEED);
    return;
  }

  // --- 2. The PID Math ---
  float derivative = error - lastError;
  float correction = (Kp * error) + (Kd * derivative);
  lastError = error;

  // --- 3. Full-Bridge Motor Logic ---
  // Instead of fixed turn speeds, we let the correction 
  // push the motors into negative values (reverse).
  int pwrL = BASE_L - (int)correction;
  int pwrR = BASE_R + (int)correction;

  applyMotorPower(motorLeft, pwrL);
  applyMotorPower(motorRight, pwrR);
}

// Helper function to handle direction and speed smoothly
void applyMotorPower(AF_DCMotor &motor, int speed) {
  if (speed >= 0) {
    motor.run(FORWARD);
    motor.setSpeed(constrain(speed, 0, 255));
  } else {
    motor.run(BACKWARD);
    motor.setSpeed(constrain(abs(speed), 0, 255));
  }
}

// Dedicated pivot for line searching
void pivot(int dirL, int dirR, int speed) {
  motorLeft.run(dirL);
  motorRight.run(dirR);
  motorLeft.setSpeed(speed);
  motorRight.setSpeed(speed);
}