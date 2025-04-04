// Straight-Line Maze Navigation with Dual PID Motor Control,
// Ultrasonic Wall Avoidance, and Integral Wind-Up Handling
//5th of April, 2025, 1:11am, Abeer Masroor.
// -------------------------

// ---- Encoder Pins and Variables ----
const int encoder_L = 37;  // Left encoder
const int encoder_R = 36;  // Right encoder
volatile int leftCount = 0;
volatile int rightCount = 0;

unsigned long lastTime = 0;
unsigned long interval = 1;  // PID update interval

// ---- Motor Pins ----
int enA = 4;   // Right PWM
int in1 = 5;
int in2 = 6;

int enB = 14;  // Left PWM
int in3 = 13;
int in4 = 12;

// ---- Ultrasonic Sensor Pins ----
const int trigL = 8;
const int trigC = 9;
const int trigR = 10;

const int echoL = 27;
const int echoC = 28;
const int echoR = 29;

// ---- PID Parameters ----
float Kp = 3.0;
float Ki = 1;    
float Kd = 0.01;

float integral = 0;
float previousError = 0;
int basePWM = 70;

float maxIntegral = 100.0;

// ---- Wall Avoidance ----
int wallThreshold = 20;
int wallCorrection = 20;

// ---- Encoder ISRs ----
void leftEncoderISR() { leftCount++; }
void rightEncoderISR() { rightCount++; }

// ---- Ultrasonic Functions ----
int getDistance(int trigPin, int echoPin) {
  long duration = 0;
 
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 30000);
  return (duration) * 0.034 / 2;
}


int leftDistance()   { return getDistance(trigL, echoL); }
int centerDistance() { return getDistance(trigC, echoC); }
int rightDistance()  { return getDistance(trigR, echoR); }

// ---- Motor Control ----
void forward(int LPWM, int RPWM) {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  analogWrite(enA, RPWM);
  analogWrite(enB, LPWM);
}



void backward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  analogWrite(enA, basePWM);
  analogWrite(enB, basePWM);
}



void Stop() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0); analogWrite(enB, 0);
}

// ---- PID Control with Wall Avoidance ----
void adjustMotorSpeeds(int distL, int distR) {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    float leftSpeed = leftCount;
    float rightSpeed = rightCount;
    float error = leftSpeed - rightSpeed;

    integral += error;
    integral = constrain(integral, -maxIntegral, maxIntegral);

    float derivative = error - previousError;
    float adjustment = Kp * error + Ki * integral + Kd * derivative;

    Serial.print("integral: ");
    Serial.println(integral);

    int leftPWM = basePWM - adjustment;
    int rightPWM = basePWM + adjustment;

     if (distL < wallThreshold) {
      rightPWM -= wallCorrection;
      leftPWM += wallCorrection;
    }
    else if (distR < wallThreshold) {
      rightPWM += wallCorrection;
      leftPWM -= wallCorrection;
    }
 
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    
//       Serial.print("leftPWM: ");
//    Serial.println(leftPWM);
//     Serial.print("rightPWM: ");
//    Serial.println(rightPWM);

    forward(leftPWM, rightPWM);

    previousError = error;
    leftCount = 0;
    rightCount = 0;
    lastTime = currentTime;
  }
}

// ---- Setup ----
void setup() {
  Serial.begin(9600);  // Ensure Serial Monitor is also set to 9600
  Serial.println("Setup started...");

  pinMode(encoder_L, INPUT_PULLUP);
  pinMode(encoder_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_L), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_R), rightEncoderISR, CHANGE);

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  pinMode(trigL, OUTPUT); pinMode(echoL, INPUT);
  pinMode(trigC, OUTPUT); pinMode(echoC, INPUT);
  pinMode(trigR, OUTPUT); pinMode(echoR, INPUT);

  lastTime = millis();
  Serial.println("Motors started.");
}

// ---- Loop ----
void loop() {


int count = 10;
int TLD = 0;  //Total Left Distance
int TRD = 0;  //Total Right Distance

for(int i = 0; i < count; i++){
  TLD += leftDistance();
  TRD += rightDistance();
  }

  int distL = TLD / count;
  
  int distR = TRD / count;
//
//  Serial.print(distL);
//  Serial.println("left cm");
//  Serial.print(distR);
//  Serial.println("right cm");

  adjustMotorSpeeds(distL, distR);

millis();
//delay(2000);
}
