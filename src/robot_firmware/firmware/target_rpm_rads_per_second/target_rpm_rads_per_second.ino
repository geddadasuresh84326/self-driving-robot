// ================================================================
// PIN DEFINITIONS
// ================================================================
// Left Motor Pins
const int LEFT_PWM_PIN = 11;
const int LEFT_DIR1_PIN = 7;
const int LEFT_DIR2_PIN = 8;
const int LEFT_ENC_A = 2;  
const int LEFT_ENC_B = 3;

// Right Motor Pins
const int RIGHT_PWM_PIN = 6;
const int RIGHT_DIR1_PIN = 5;
const int RIGHT_DIR2_PIN = 4;
const int RIGHT_ENC_A = 9;  
const int RIGHT_ENC_B = 10;

// ================================================================
// MOTOR ENCODER & TIMING CONFIGURATION
// ================================================================
const float ENCODER_PPR = 896.0;
const unsigned long SAMPLE_TIME_MS = 20;   
const float DT = SAMPLE_TIME_MS / 1000.0;  

// Setpoints (Target RPMs for internal loop)
float targetRPM_Left = 0.0;  
float targetRPM_Right = 0.0;

// ================================================================
// PI GAINS (Tuned for RPM)
// ================================================================
const float Kp_Left = 2.3, Ki_Left = 1.7;
const float Kp_Right = 2.25, Ki_Right = 1.7;

// ================================================================
// GLOBAL VARIABLES
// ================================================================
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

float integral_Left = 0.0;
float integral_Right = 0.0;

unsigned long lastSampleTime = 0;

// ================================================================
// SERIAL COMMUNICATION VARIABLES
// ================================================================
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// ================================================================
// INTERRUPT SERVICE ROUTINES
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

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR1_PIN, OUTPUT);
  pinMode(LEFT_DIR2_PIN, OUTPUT);

  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR1_PIN, OUTPUT);
  pinMode(RIGHT_DIR2_PIN, OUTPUT);

  analogWrite(LEFT_PWM_PIN, 0);
  digitalWrite(LEFT_DIR1_PIN, LOW);
  digitalWrite(LEFT_DIR2_PIN, LOW);

  analogWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(RIGHT_DIR1_PIN, LOW);
  digitalWrite(RIGHT_DIR2_PIN, LOW);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), readRightEncoder, CHANGE);
  
  delay(5000);

  noInterrupts();
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
  interrupts();
  
  lastSampleTime = millis();
}

void loop() {
  // 1. Constantly check for incoming Python commands
  recvWithEndMarker();
  parseData();

  unsigned long now = millis();

  // 2. Run PI Loop at fixed intervals (DT)
  if (now - lastSampleTime >= SAMPLE_TIME_MS) {
    lastSampleTime = now;

    noInterrupts();
    long countsLeft = leftEncoderTicks;
    long countsRight = rightEncoderTicks;
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();

    // Calculate RPM for internal PID usage
    float measuredRPM_Left = (countsLeft / ENCODER_PPR) * (60.0 / DT);
    float measuredRPM_Right = (countsRight / ENCODER_PPR) * (60.0 / DT);

    // PI Loop - Left
    float error_L = targetRPM_Left - measuredRPM_Left;
    integral_Left += error_L * DT;
    integral_Left = constrain(integral_Left, -255.0 / Ki_Left, 255.0 / Ki_Left);  
    float control_L = (Kp_Left * error_L) + (Ki_Left * integral_Left);

    // PI Loop - Right
    float error_R = targetRPM_Right - measuredRPM_Right;
    integral_Right += error_R * DT;
    integral_Right = constrain(integral_Right, -255.0 / Ki_Right, 255.0 / Ki_Right); 
    float control_R = (Kp_Right * error_R) + (Ki_Right * integral_Right);

    // Drive Motors
    driveMotor(LEFT_PWM_PIN, LEFT_DIR1_PIN, LEFT_DIR2_PIN, control_L);
    driveMotor(RIGHT_PWM_PIN, RIGHT_DIR1_PIN, RIGHT_DIR2_PIN, control_R);

    // 3. Convert measured RPM back to rad/s for Python feedback
    float measuredRadS_Right = measuredRPM_Right * ( (2.0 * PI) / 60.0 );
    float measuredRadS_Left = measuredRPM_Left * ( (2.0 * PI) / 60.0 );

    // 4. Send Feedback
    Serial.print("rp");
    Serial.print(measuredRadS_Right);
    Serial.print(",ln");
    Serial.println(measuredRadS_Left);
  }
}

// ================================================================
// HELPER FUNCTIONS
// ================================================================
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

// Reads serial data without blocking the PI loop
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n'; 
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1; 
      }
    } else {
      receivedChars[ndx] = '\0'; 
      ndx = 0;
      newData = true;
    }
  }
}

// Parses the protocol string "rpX.XX,lnX.XX" (Values are in rad/s)
void parseData() {
  if (newData == true) {
    char *rpPtr = strstr(receivedChars, "rp");
    char *lnPtr = strstr(receivedChars, "ln");

    if (rpPtr != NULL && lnPtr != NULL) {
      // 1. Extract the raw rad/s float values from the serial string
      float targetRadS_Right = atof(rpPtr + 2);
      float targetRadS_Left = atof(lnPtr + 2);
      
      // 2. Convert rad/s directly to RPM for the internal PI controller
      targetRPM_Right = targetRadS_Right * (60.0 / (2.0 * PI));
      targetRPM_Left = targetRadS_Left * (60.0 / (2.0 * PI));
    }
    
    newData = false; 
  }
}