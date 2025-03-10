
const int trigL = 8;
//const int trigC = 9;
//const int trigR = 10;

const int echoL = 27;
//const int echoC = 28;
//const int echoR = 29;

// defines variables
long durationL;
//long durationC;
//long durationR;

int distanceL;
//int distanceC;
//int distanceR;
void setup() {
  pinMode(trigL, OUTPUT); 
//  pinMode(trigC, OUTPUT); 
//  pinMode(trigR, OUTPUT); 

  
  pinMode(echoL, INPUT); // Sets the echoPin as an Input
//  pinMode(echoC, INPUT);
//  pinMode(echoR, INPUT);

  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  // Clears the trigPin
  digitalWrite(trigL, LOW);
//  digitalWrite(trigC, LOW);
//  digitalWrite(trigR, LOW);
  
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigL, HIGH);
//  digitalWrite(trigC, HIGH);
//  digitalWrite(trigR, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(trigL, LOW);
//  digitalWrite(trigC, LOW);
//  digitalWrite(trigR, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationL = pulseIn(echoL, HIGH);
//  durationC = pulseIn(echoC, HIGH);
//  durationR = pulseIn(echoR, HIGH);

  
  // Calculating the distance
    distanceL = durationL * 0.034 / 2;
//  distanceC = durationC * 0.034 / 2;
//  distanceR = durationR * 0.034 / 2;
  
  // Prints the distance on the Serial Monitor
  Serial.print("Left Distance: ");
  Serial.println(distanceL);
//  Serial.print("Center Distance: ");
//  Serial.println(distanceC);
//  Serial.print("Right Distance: ");
//  Serial.println(distanceR);
//  
}
