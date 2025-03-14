// Ultrasonic Sensor Pins
const int trigL = 8;
const int trigC = 9;
const int trigR = 10;

const int echoL = 27;
const int echoC = 28;
const int echoR = 29;

// Motor A (Right Motor)
int enA = 4;
int in1 = 5;
int in2 = 6;

// Motor B (Left Motor)
int enB = 14;
int in3 = 13;
int in4 = 12;

#define TARGET_DISTANCE 20
#define TOLERANCE 2

void setup() {
  Serial.begin(9600);
  
  // Motor pin setups
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Ultrasonic sensor setups
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  pinMode(trigC, OUTPUT);
  pinMode(echoC, INPUT);
  pinMode(trigR, OUTPUT);
  pinMode(echoR, INPUT);
}

int leftDistance() {
  long duration;
  digitalWrite(trigL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigL, LOW);
  duration = pulseIn(echoL, HIGH);
  return duration * 0.034 / 2;
}

int centerDistance() {
  long duration;
  digitalWrite(trigC, LOW);
  delayMicroseconds(2);
  digitalWrite(trigC, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigC, LOW);
  duration = pulseIn(echoC, HIGH);
  return duration * 0.034 / 2;
}

int rightDistance() {
  long duration;
  digitalWrite(trigR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigR, LOW);
  duration = pulseIn(echoR, HIGH);
  return duration * 0.034 / 2;
}

void forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 230);
  analogWrite(enB, 255);
}

void backward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void leftTurn() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void rightTurn() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void wallFollowMotion() {
  int distL = leftDistance();
  int distR = rightDistance();

  // Debug output for sensor readings
  Serial.print("L: ");
  Serial.print(distL);
  Serial.print("  R: ");
  Serial.println(distR);
  
  bool adjustLeft = false;
  bool adjustRight = false;
  
  // Left sensor logic
  if (distL < TARGET_DISTANCE - TOLERANCE) {
    adjustRight = true;
  } else if (distL > TARGET_DISTANCE + TOLERANCE) {
    adjustLeft = true;
  }
  
  // Right sensor logic
  if (distR < TARGET_DISTANCE - TOLERANCE) {
    adjustLeft = true;
  } else if (distR > TARGET_DISTANCE + TOLERANCE) {
    adjustRight = true;
  }
  
  // Act based solely on side sensor comparisons
  if (adjustLeft && !adjustRight) {
    leftTurn();
    delay(100);
  } else if (adjustRight && !adjustLeft) {
    rightTurn();
    delay(100);
  } else {
    forward();
  }
}

void loop() {
  wallFollowMotion();
}
