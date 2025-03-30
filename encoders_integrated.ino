//This code allows the car to make 90 degree turns independent of battery charge etc. This code makes use of optical encoders and Tiva-C interrupts to precisely rotate the car.
//Author: Abeer Masroor, am08734, 31st of March, 1:11am, Eid Day 1!! :)

// Encoder Pins
const int encoder_L = 37;  // Left encoder to PC4
const int encoder_R = 36;  // Right encoder to PC5

// Motor A (Right Motor)
int enA = 4;   // PB1 (PWM)
int in1 = 5;   // PE4
int in2 = 6;   // PE5

// Motor B (Left Motor)
int enB = 14;  // PB6 (PWM)
int in3 = 13;  // PA4
int in4 = 12;  // PA3

// Encoder variables
volatile int leftCount = 0;
volatile int rightCount = 0;

// Interrupt Service Routines (ISRs) for encoders
void leftEncoderISR() { leftCount++; }
void rightEncoderISR() { rightCount++; }

void setup() {
    Serial.begin(9600);

    // Set up encoder pins
    pinMode(encoder_L, INPUT_PULLUP);
    pinMode(encoder_R, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(encoder_L), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder_R), rightEncoderISR, CHANGE);

    // Set up motor control pins
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Execute a single left turn and stop
//    preciseTurn(true);
}

void preciseTurn(bool isLeft) {
    leftCount = 0;
    rightCount = 0;
    int targetTicks = 12;  // 11 or 12 works fine when speed is 70 at moderate charge

    if (isLeft) {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH);  // Right motor forward
        digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  // Left motor backward
    }

    while (leftCount < targetTicks || rightCount < targetTicks) {
        analogWrite(enA, 255);  // Right motor slow forward
        analogWrite(enB, 255);  // Left motor slow backward
        delay(50);             // Small delay for controlled movement
    }

    Stop();
}

// Stop both motors
void Stop() {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
    analogWrite(enA, 0); analogWrite(enB, 0);
}

void loop() {
    preciseTurn(true); // First of four 90 degree turns (supposed to be accurate!)
    delay(500);
    preciseTurn(true);
    delay(500);
    preciseTurn(true);
    delay(500);
    preciseTurn(true);
    while(1);
}
