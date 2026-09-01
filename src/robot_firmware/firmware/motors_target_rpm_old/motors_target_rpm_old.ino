// ================================================================
// PIN DEFINITIONS
// ================================================================
// Left Motor Pins
const int LEFT_PWM_PIN = 11;
const int LEFT_DIR1_PIN = 7;
const int LEFT_DIR2_PIN = 8;
const int LEFT_ENC_A = 2;  // Hardware Interrupt Pin
const int LEFT_ENC_B = 3;

// Right Motor Pins
const int RIGHT_PWM_PIN = 6;
const int RIGHT_DIR1_PIN = 5;
const int RIGHT_DIR2_PIN = 4;
const int RIGHT_ENC_A = 9;  // Note: Check board interrupt support for Pin 9
const int RIGHT_ENC_B = 10;

// ================================================================
// MOTOR ENCODER & TIMING CONFIGURATION
// ================================================================
// 2X Decoding PPR: Ensure 896.0 represents total 2X counts per revolution
const float ENCODER_PPR = 896.0;
const unsigned long SAMPLE_TIME_MS = 20;   // 20 ms sample rate (50 Hz)
const float DT = SAMPLE_TIME_MS / 1000.0;  // Time step in seconds

// Setpoints (Target RPMs - Can now be positive or negative)
float targetRPM_Left = 30.0;
float targetRPM_Right = 30.0;

// ================================================================
// PI GAINS (From MATLAB)
// ================================================================
// const float Kp_Left = 1.6311, Ki_Left = 7.3915;
// const float Kp_Right = 1.4808, Ki_Right = 6.3112;
const float Kp_Left = 2.3, Ki_Left = 1.7;
const float Kp_Right = 2.15, Ki_Right = 1.7;

// ================================================================
// GLOBAL VARIABLES
// ================================================================
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

float integral_Left = 0.0;
float integral_Right = 0.0;

unsigned long lastSampleTime = 0;

// ================================================================
// INTERRUPT SERVICE ROUTINES (2X Quadrature Decoding)
// ================================================================
void readLeftEncoder() {
  int a = digitalRead(LEFT_ENC_A);
  int b = digitalRead(LEFT_ENC_B);
  if (a != b) {
    leftEncoderTicks++;
  } else {
    leftEncoderTicks--;
  }
}

void readRightEncoder() {
  int a = digitalRead(RIGHT_ENC_A);
  int b = digitalRead(RIGHT_ENC_B);
  if (a != b) {
    rightEncoderTicks--;
  } else {
    rightEncoderTicks++;
  }
}

void setup() {
  Serial.begin(115200);

  // Motor Driver Outputs
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR1_PIN, OUTPUT);
  pinMode(LEFT_DIR2_PIN, OUTPUT);

  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR1_PIN, OUTPUT);
  pinMode(RIGHT_DIR2_PIN, OUTPUT);

  // Explicitly set safe initial states (0 PWM, safe directions)
  analogWrite(LEFT_PWM_PIN, 0);
  digitalWrite(LEFT_DIR1_PIN, LOW);
  digitalWrite(LEFT_DIR2_PIN, LOW);

  analogWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(RIGHT_DIR1_PIN, LOW);
  digitalWrite(RIGHT_DIR2_PIN, LOW);

  // Encoder Inputs with Internal Pull-Ups
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Attach CHANGE Interrupts to Channel A for 2X Decoding
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), readRightEncoder, CHANGE);
  delay(5000);

  // 2. Clear any random ticks that accumulated during the 20 seconds
  noInterrupts();
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
  interrupts();
  lastSampleTime = millis();
}

void loop() {
  unsigned long now = millis();

  // Run PI Loop at fixed intervals (DT)
  if (now - lastSampleTime >= SAMPLE_TIME_MS) {
    lastSampleTime = now;

    // 1. Read signed encoder count & reset atomically
    noInterrupts();
    long countsLeft = leftEncoderTicks;
    long countsRight = rightEncoderTicks;
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();

    // 2. Calculate Signed Measured RPM
    float measuredRPM_Left = (countsLeft / ENCODER_PPR) * (60.0 / DT);
    float measuredRPM_Right = (countsRight / ENCODER_PPR) * (60.0 / DT);

    // 3. Compute PI Output - Left Motor
    float error_L = targetRPM_Left - measuredRPM_Left;
    integral_Left += error_L * DT;
    integral_Left = constrain(integral_Left, -255.0 / Ki_Left, 255.0 / Ki_Left);  // Anti-windup
    float control_L = (Kp_Left * error_L) + (Ki_Left * integral_Left);

    // 4. Compute PI Output - Right Motor
    float error_R = targetRPM_Right - measuredRPM_Right;
    integral_Right += error_R * DT;
    integral_Right = constrain(integral_Right, -255.0 / Ki_Right, 255.0 / Ki_Right);  // Anti-windup
    float control_R = (Kp_Right * error_R) + (Ki_Right * integral_Right);

    // 5. Drive Motors
    driveMotor(LEFT_PWM_PIN, LEFT_DIR1_PIN, LEFT_DIR2_PIN, control_L);
    driveMotor(RIGHT_PWM_PIN, RIGHT_DIR1_PIN, RIGHT_DIR2_PIN, control_R);

    // 6. Output to Serial Plotter
    Serial.print("Target:");
    Serial.print(targetRPM_Left);
    Serial.print(",");
    Serial.print("Left_RPM:");
    Serial.print(measuredRPM_Left);
    Serial.print(",");
    Serial.print("Right_RPM:");
    Serial.println(measuredRPM_Right);
  }
}

// Drive Motor Helper Function
void driveMotor(int pwmPin, int dir1Pin, int dir2Pin, float controlVal) {
  int pwmVal = constrain(abs((int)controlVal), 0, 255);

  if (controlVal >= 0) {
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
  } else {
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
  }

  analogWrite(pwmPin, pwmVal);
}