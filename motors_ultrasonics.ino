// Ultrasonic Sensor Pins
const int trigL = 8;
//const int trigC = 9;
//const int trigR = 10;

const int echoL = 27;
//const int echoC = 28;
//const int echoR = 29;

// Motor A (Right Motor) - TivaC and L298N connections
int enA = 4;   // ENABLE pin (PB1) for Motor A to provide PWM 
int in1 = 5;   // IN1 pin (PE4) for Motor A direction
int in2 = 6;   // IN2 pin (PE5) for Motor A direction

// Motor B (Left Motor)
int enB = 14;    // ENABLE pin (PB6) for Motor B to provide PWM 
int in3 = 13;    // IN1 pin (PA4) for Motor B direction
int in4 = 12;    // IN2 pin (PA3) for Motor B direction

void setup() {
  Serial.begin(9600); // Serial initialization for debugging
  
  // Set pin modes for Motor A
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Set pin modes for Motor B
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Set up Ultrasonic sensor
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
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

void forward() {
  digitalWrite(in1, LOW); // Right motor forward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); // Left motor forward
  digitalWrite(in4, HIGH);
  
  analogWrite(enA, 230);  // Set speed for right motor
  analogWrite(enB, 255);  // Set speed for left motor
}

void backward() {
  digitalWrite(in1, HIGH); // Right motor backward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); // Left motor backward
  digitalWrite(in4, LOW);
  
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void leftTurn() {
  digitalWrite(in1, LOW); // Right motor forward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); // Left motor backward
  digitalWrite(in4, LOW);
  
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void rightTurn() {
  digitalWrite(in1, HIGH); // Right motor backward
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); // Left motor forward
  digitalWrite(in4, HIGH);
  
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void Stop() {
  digitalWrite(in1, LOW); // Right motor stop
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); // Left motor stop
  digitalWrite(in4, LOW);
  
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void loop() {
  int distance = leftDistance();
  Serial.print("Left Distance: ");
  Serial.println(distance);

  if (distance < 20) {
    Stop();
    backward();
    delay(400);  // Move backward for 0.4 seconds
    rightTurn();
    delay(200);  // Turn right for 0.2 seconds
  } else {
    forward();
  }
}
