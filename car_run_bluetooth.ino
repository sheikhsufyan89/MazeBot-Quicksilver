 #include <Servo.h>   //code shown to Ms Abeera for Milestone 4 evaluation on 7th of April, 2025.
 const int trigL = 8;
 const int trigC = 9;
 const int trigR = 10;
 const int echoL = 27;
 const int echoC = 28;
 const int echoR = 29;
 int enA = 4;
 int in1 = 5;
 int in2 = 6;
 int enB = 14;
 int in3 = 13;
 int in4 = 12;
 #define TARGET_DISTANCE 40
 #define FRONT_THRESHOLD 40
 #define TOLERANCE 2
 bool isRunning = false;
 void setup() {
    Serial.begin(9600);
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
    pinMode(trigL, OUTPUT);
    pinMode(echoL, INPUT);
    pinMode(trigC, OUTPUT);
    pinMode(echoC, INPUT);
    pinMode(trigR, OUTPUT);
    pinMode(echoR, INPUT);
 }
 int measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    return pulseIn(echoPin, HIGH) * 0.034 / 2;
 }
 void forward() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, 80);
    analogWrite(enB, 80);
 }
 void leftTurn() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, 80);
    analogWrite(enB, 80);
 }

  void rightTurn() {
 digitalWrite(in1, HIGH);
 digitalWrite(in2, LOW);
 digitalWrite(in3, LOW);
 digitalWrite(in4, HIGH);
 analogWrite(enA, 80);
 analogWrite(enB, 80);
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
 int distL = measureDistance(trigL, echoL);
 int distC = measureDistance(trigC, echoC);
 int distR = measureDistance(trigR, echoR);
 Serial.print("Left Distance: ");
 Serial.println(distL);
 Serial.print("Center Distance: ");
 Serial.println(distC);
 Serial.print("Right Distance: ");
 Serial.println(distR);
 if (distC < FRONT_THRESHOLD) {
 Stop();
 if (distL > distR) {
 leftTurn();
 } else {
 rightTurn();
 }
 delay(100);
 return;
 }
 bool adjustLeft = false;
 bool adjustRight = false;
 if (distL < TARGET_DISTANCE - TOLERANCE) adjustRight = true;
 else if (distL > TARGET_DISTANCE + TOLERANCE) adjustLeft = true;
 if (distR < TARGET_DISTANCE - TOLERANCE) adjustLeft = true;
 else if (distR > TARGET_DISTANCE + TOLERANCE) adjustRight = true;
 if (adjustLeft && !adjustRight) {
 leftTurn();
 delay(50);
 } else if (adjustRight && !adjustLeft) {
 rightTurn();
 delay(50);
 } else {
 forward();
 }
 }
 void loop() {
 if (Serial.available() > 0) {
 char received = Serial.read();
 Serial.println(received);
 if (received == 'A') {
 isRunning = !isRunning;
 }

 }
 
 if (isRunning) {
 wallFollowMotion();
 } else {
 Stop();
 }
 }
