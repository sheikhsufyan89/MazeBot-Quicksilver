// ---- BASELINE SETTINGS ----
float baseline_PWM = 80;  // Base speed for motors (was initially 100)

// ---- Motor Pins ----
int enA = 4, in1 = 5, in2 = 6;         // Motor A: PWM and direction control pins
int enB = 33, in3 = 32, in4 = 31;      // Motor B: PWM and direction control pins

// ---- Ultrasonic Sensor Pins ----
const int trigL = 2, trigC = 9, trigR = 10;    // Trigger pins (Left, Center, Right)
const int echoL = 23, echoC = 28, echoR = 29;  // Echo pins (Left, Center, Right)

// ---- PID Variables ----
float error = 0;         // Current error between left and right distances
float lastError = 0;     // Previous error (for derivative calculation)
float deltaError = 0;    // Change in error
const float Kp = 1.343;  // Proportional gain constant
const float Kd = 10;     // Derivative gain constant
bool isRunning = false;  // Robot movement state (true = moving, false = stopped)


// ---- Function to Move Forward with PD Control ----
void moveForward(float distR, float distL) {
    // Calculate error (difference between right and left wall distances)
    error = distR - distL;
    
    // Limit error to prevent sudden sharp corrections
    error = constrain(error, -35, 35);

    // Calculate change in error
    deltaError = error - lastError;

    // Calculate derivative term
    float derivative = Kd * deltaError;

    // Debug: Print error to Serial Monitor
    Serial.print("Error: ");
    Serial.println(error);

    // Calculate individual motor speeds based on PD correction
    float right_PWM = baseline_PWM + Kp * error + derivative;
    float left_PWM  = baseline_PWM - Kp * error - derivative;

    // Update lastError for next iteration
    lastError = error;

    // Set both motors to move forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    // Limit PWM signals to safe range
    right_PWM = constrain(right_PWM, 0, 200);
    left_PWM  = constrain(left_PWM, 0, 200);

    // Send PWM signals to motors
    analogWrite(enA, right_PWM);
    analogWrite(enB, left_PWM);
}


// ---- Function to Measure Distance using Ultrasonic Sensor ----
float getDistance(int trigPin, int echoPin) {
    long duration;

    // Send a 10us HIGH pulse to trigger the ultrasonic sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the duration of the HIGH pulse (echo time)
    duration = pulseIn(echoPin, HIGH, 30000);  // Timeout at 30ms

    // Calculate distance (speed of sound = 0.034 cm/us, divide by 2 for round trip)
    float distance = duration * 0.034 / 2;

    // If invalid or out of range, set distance to 100 cm
    if (distance <= 0 || distance > 100.0) return 100.0;
    return distance;
}


// ---- Function to Stop the Robot ----
void Stop() {
    // Stop motor movement by setting motor pins LOW
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    // Set motor speeds to zero
    analogWrite(enA, 0);
    analogWrite(enB, 0);
}


// ---- Arduino Setup Function ----
void setup() {
    delay(1500);  // Small delay before starting
    Serial.begin(9600);  // Initialize serial communication
    Serial.println("Starting...");

    // Set motor pins as OUTPUT
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(trigL, OUTPUT);
    pinMode(trigC, OUTPUT);
    pinMode(trigR, OUTPUT);
    pinMode(echoL, INPUT);
    pinMode(echoC, INPUT);
    pinMode(echoR, INPUT);
}


// ---- Arduino Main Loop ----
void loop() {
    // Read distances from all three ultrasonic sensors
    float distL = getDistance(trigL, echoL);
    float distC = getDistance(trigC, echoC);
    float distR = getDistance(trigR, echoR);

    // Uncomment these lines if you want to see distance readings
    /*
    Serial.print("L: "); Serial.print(distL);
    Serial.print("  C: "); Serial.print(distC);
    Serial.print("  R: "); Serial.println(distR);
    */

    // Check for user input over Serial
    if (Serial.available() > 0) {
        char received = Serial.read();
        Serial.println(received);

        // Toggle robot running state when 'A' is received
        if (received == 'A') {
            isRunning = !isRunning;
        }
    }

    // Control robot movement based on running state
    if (isRunning) {
        moveForward(distR, distL);
    } else {
        Stop();
    }
}
