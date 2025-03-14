// Ultrasonic Sensor Pins
const int trigL = 8;
const int trigC = 9;
const int trigR = 10;

const int echoL = 27;
const int echoC = 28;
const int echoR = 29;

// Motor A (Right Motor) - TivaC and L298N connections
int enA = 4;   // ENABLE pin (PB1) for Motor A to provide PWM 
int in1 = 5;   // IN1 pin (PE4) for Motor A direction
int in2 = 6;   // IN2 pin (PE5) for Motor A direction

// Motor B (Left Motor)
int enB = 14;  // ENABLE pin (PB6) for Motor B to provide PWM 
int in3 = 13;  // IN1 pin (PA4) for Motor B direction
int in4 = 12;  // IN2 pin (PA3) for Motor B direction

// Define target distances and thresholds
#define TARGET_DISTANCE 20     // Desired distance from each side wall
#define FRONT_THRESHOLD 15     // Obstacle threshold for center sensor
#define TOLERANCE 2            // Allowable error range

void setup() {
  Serial.begin(9600); // Initialize serial for debugging
  
  // Set pin modes for Motor A
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Set pin modes for Motor B
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors initially
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Set up ultrasonic sensor pins
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  pinMode(trigC, OUTPUT);
  pinMode(echoC, INPUT);
  pinMode(trigR, OUTPUT);
  pinMode(echoR, INPUT);
}

int leftDistance() {
  long durationL;
  int distanceL;
  
  digitalWrite(trigL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigL, LOW);
  
  durationL = pulseIn(echoL, HIGH);
  distanceL = durationL * 0.034 / 2;
  return distanceL;
}

int centerDistance() {
  long durationC;
  int distanceC;
  
  digitalWrite(trigC, LOW);
  delayMicroseconds(2);
  digitalWrite(trigC, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigC, LOW);
  
  durationC = pulseIn(echoC, HIGH);
  distanceC = durationC * 0.034 / 2;
  return distanceC;
}

int rightDistance() {
  long durationR;
  int distanceR;
  
  digitalWrite(trigR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigR, LOW);
  
  durationR = pulseIn(echoR, HIGH);
  distanceR = durationR * 0.034 / 2;
  return distanceR;
}

void forward() {
  // Both motors forward
  digitalWrite(in1, LOW);   // Right motor forward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);   // Left motor forward
  digitalWrite(in4, HIGH);
  
  analogWrite(enA, 230);    // Set speed for right motor
  analogWrite(enB, 255);    // Set speed for left motor
}

void backward() {
  // Both motors backward
  digitalWrite(in1, HIGH);  // Right motor backward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);  // Left motor backward
  digitalWrite(in4, LOW);
  
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void leftTurn() {
  // Right motor forward, Left motor backward (rotate left)
  digitalWrite(in1, LOW);   // Right motor forward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);  // Left motor backward
  digitalWrite(in4, LOW);
  
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void rightTurn() {
  // Right motor backward, Left motor forward (rotate right)
  digitalWrite(in1, HIGH);  // Right motor backward
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);   // Left motor forward
  digitalWrite(in4, HIGH);
  
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void Stop() {
  // Stop both motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

// Wall-following motion function using our established discrete logic
void wallFollowMotion() {
  int distL = leftDistance();
  int distC = centerDistance();
  int distR = rightDistance();

  // Print sensor readings for debugging
  Serial.print("Left Distance: ");
  Serial.println(distL);
  Serial.print("Center Distance: ");
  Serial.println(distC);
  Serial.print("Right Distance: ");
  Serial.println(distR);

  // Obstacle Avoidance: If an object is too close in front (using the center sensor)
  if (distC < FRONT_THRESHOLD) {
    Stop();
    backward();
    delay(400);  // Reverse for 0.4 seconds
    // Choose turn direction based on which side has more space:
    if (distL > distR) {
      leftTurn();
    } else {
      rightTurn();
    }
    delay(200);  // Execute turn for 0.2 seconds
    return;      // Skip further processing in this cycle
  }

  // Side adjustments: Use discrete threshold comparisons (with tolerance)
  bool adjustLeft = false;
  bool adjustRight = false;
  
  // Left sensor logic:
  // - Too close to left wall? (reading is less than target minus tolerance) => steer right.
  // - Too far from left wall? (reading is greater than target plus tolerance) => steer left.
  if (distL < TARGET_DISTANCE - TOLERANCE) {
    adjustRight = true;
  } else if (distL > TARGET_DISTANCE + TOLERANCE) {
    adjustLeft = true;
  }
  
  // Right sensor logic:
  // - Too close to right wall? => steer left.
  // - Too far from right wall? => steer right.
  if (distR < TARGET_DISTANCE - TOLERANCE) {
    adjustLeft = true;
  } else if (distR > TARGET_DISTANCE + TOLERANCE) {
    adjustRight = true;
  }
  
  // Conflict resolution:
  // If both adjustments are signaled, we assume the corrections cancel out and go straight.
  if (adjustLeft && !adjustRight) {
    leftTurn();
    delay(100);  // Brief adjustment delay
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
