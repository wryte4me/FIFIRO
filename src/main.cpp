#include <Arduino.h>
#include <NewPing.h> // Library for ultrasonic sensor

// Global variable for motor speed (default to 255 for maximum speed)
int motorSpeed = 255;

const int sonarMaxDistance  = 300;   // Maximum distance for the sonar sensor (in centimeters)
const byte sonarSampling    = 5;     // Number of samples to take for averaging sonar readings

byte command = 0;  // Initialize command variable
bool inAutoMode = false;
int sonarInterval = 200; 

int frontDistance = 0;
int rightDistance = 0;
int leftDistance = 0;

// PINS ------------------------------------------------------------------
const byte flame1   = 26;   // Pin for the front flame sensor
const byte flame2   = 27;   // Pin for the right flame sensor
const byte flame3    = 28;   // Pin for the left flame sensor
const byte flame4 = 29;
const byte flame5 = 30;
const byte flame6 = 31;
const byte flameNumRead = 10;

const byte sonarFront_pin   = 8;    // Pin for the front sonar sensor
const byte sonarRight_pin   = 9;    // Pin for the right sonar sensor
const byte sonarLeft_pin    = 10;   // Pin for the left sonar sensor

// Right motor pins
const byte rightEna_pin     = 2;    // Enable pin for the right motor
const byte rightForward_pin = 3;    // Forward pin for the right motor
const byte rightReverse_pin = 4;    // Reverse pin for the right motor

// Left motor pins
const byte leftEna_pin      = 7;    // Enable pin for the left motor
const byte leftForward_pin  = 6;    // Forward pin for the left motor
const byte leftReverse_pin  = 5;    // Reverse pin for the left motor

// Linear Actuator Driver Pins
const byte linearCwEn_pin   = 22;   // Clockwise enable pin for the linear actuator driver
const byte linearCcwEn_pin  = 23;   // Counterclockwise enable pin for the linear actuator driver
const byte linearCwPwm_pin  = 10;   // Clockwise PWM pin for controlling the linear actuator speed
const byte linearCcwPwm_pin = 11;   // Counterclockwise PWM pin for controlling the linear actuator speed

// OBJECTS ------------------------------------------------------------------
// Create NewPing objects for sonar sensors
NewPing sonarFront(sonarFront_pin, sonarFront_pin, sonarMaxDistance); // Front sonar
NewPing sonarRight(sonarRight_pin, sonarRight_pin, sonarMaxDistance); // Right sonar
NewPing sonarLeft(sonarLeft_pin, sonarLeft_pin, sonarMaxDistance);    // Left sonar


// PIN MODE ------------------------------------------------------------------
void setPinModes() {
    // Set right motor pins
    pinMode(rightEna_pin, OUTPUT);      // Enable pin for the right motor (controls power to the motor)
    pinMode(rightForward_pin, OUTPUT);  // Forward control pin for the right motor (sets direction of rotation)
    pinMode(rightReverse_pin, OUTPUT);  // Reverse control pin for the right motor (sets direction of rotation)

    // Set left motor pins
    pinMode(leftEna_pin, OUTPUT);       // Enable pin for the left motor (controls power to the motor)
    pinMode(leftForward_pin, OUTPUT);   // Forward control pin for the left motor (sets direction of rotation)
    pinMode(leftReverse_pin, OUTPUT);   // Reverse control pin for the left motor (sets direction of rotation)

    // Set linear actuator driver pins
    pinMode(linearCwEn_pin, OUTPUT);    // Clockwise enable pin for the linear actuator (controls operation direction)
    pinMode(linearCcwEn_pin, OUTPUT);   // Counterclockwise enable pin for the linear actuator (controls operation direction)
    pinMode(linearCwPwm_pin, OUTPUT);   // Clockwise PWM pin for controlling the linear actuator speed (modulates speed)
    pinMode(linearCcwPwm_pin, OUTPUT);  // Counterclockwise PWM pin for controlling the linear actuator speed (modulates speed)

    // Set flame sensor pins    
    pinMode(flame1, INPUT);
    pinMode(flame2, INPUT);
    pinMode(flame3, INPUT);
    pinMode(flame4, INPUT);
    pinMode(flame5, INPUT);
    pinMode(flame6, INPUT);
}


void enableLinear(bool status) {
    if (status) {
        digitalWrite(linearCwEn_pin, HIGH);
        digitalWrite(linearCcwEn_pin, HIGH);
        Serial.println("Linear actuator enabled.");
    } else {
        digitalWrite(linearCwEn_pin, LOW);
        digitalWrite(linearCcwEn_pin, LOW);
        Serial.println("Linear actuator disabled.");
    }
}

void moveLinear(String direction) {
    if (direction == "down") {
        // Move linear actuator clockwise
        digitalWrite(linearCcwPwm_pin, LOW); // Set counterclockwise PWM pin to LOW
        analogWrite(linearCwPwm_pin, 255);   // Set clockwise PWM pin to max value (255)
        Serial.println("Moving linear actuator down.");
    } else if (direction == "up") {
        // Move linear actuator counterclockwise
        digitalWrite(linearCwPwm_pin, LOW);  // Set clockwise PWM pin to LOW
        analogWrite(linearCcwPwm_pin, 255);  // Set counterclockwise PWM pin to max value (255)
        Serial.println("Moving linear actuator up.");
    } else {
        Serial.println("Invalid direction. Please use 'up' or 'down'.");
    }
}

void squeeze (){
    moveLinear("up");
}

void release (){
    moveLinear("down");
}

void stopSqueeze (){
    analogWrite(linearCwPwm_pin, 0);  // Set clockwise PWM pin to LOW
    analogWrite(linearCcwPwm_pin, 0);  // Set counterclockwise PWM pin to max value (255)
    Serial.println("Stopping");
}






int sonarDistance (NewPing sonar){
    return sonar.ping_cm();
}

// Function to test sonar sensors
void testSonar() {
    // Measure distances from the front, left, and right sonar sensors
    frontDistance = sonarDistance(sonarFront);
    delay (sonarInterval);
    rightDistance = sonarDistance(sonarRight);
    delay (sonarInterval);
    leftDistance = sonarDistance(sonarLeft);
    delay (sonarInterval);


    // Print the measured distances to the Serial Monitor
    Serial.print("Front distance: ");
    Serial.print(frontDistance);
    Serial.println(" cm");
    delay (1000);
    Serial.print("Right distance: ");
    Serial.print(rightDistance);
    Serial.println(" cm");
    delay (1000);
    Serial.print("Left distance: ");
    Serial.print(leftDistance);
    Serial.println(" cm");
    delay (1000);
}

bool onFire (byte flameSensor_pin){
    return digitalRead(flameSensor_pin) == LOW;
}

// Function to determine flame detection using majority voting
bool irPositive(byte flameSensor_pin) {
    int flameDetectedCount = 0;   // Counter for flame detections

    // Read the sensor multiple times
    for (int i = 0; i < flameNumRead; i++) {
        if (onFire(flameSensor_pin)) {
            flameDetectedCount++;
        }
        delay(50); // Short delay between readings to allow for sensor stability
    }

    // Return true if the majority of readings indicate flame detection
    return (flameDetectedCount > flameNumRead / 2);
}

byte fireDetected() {
    int flame1DetectedCount,flame2DetectedCount, flame3DetectedCount;

    // Read the sensor multiple times
    for (int i = 0; i < allFlameNumRead; i++) {
        if (irPositive (flame1)) {
            flame1DetectedCount++;
        }
        if (irPositive (flame2)) {
            flame2DetectedCount++;
        }
        if (irPositive (flame3)) {
            flame3DetectedCount++;
        }

        delay(50); // Short delay between readings to allow for sensor stability
    } 
    
    if 
    
}



// Function to set motor speed
void setSpeed(int speed) {
    // Ensure the speed is within the valid PWM range (0 to 255)
    if (speed < 0) {
        speed = 0; // Minimum speed
    } else if (speed > 255) {
        speed = 255; // Maximum speed
    }

    // Set the motor speed (PWM value) for both left and right motors
    analogWrite(rightEna_pin, speed);  // Set speed for right motor
    analogWrite(leftEna_pin, speed);   // Set speed for left motor

    // Store the current speed for future use
    motorSpeed = speed;
}

// Move forward (both motors forward)
void forward() {
    Serial.println("Moving forward");
    setSpeed(motorSpeed); // Set the speed before moving forward

    digitalWrite(rightForward_pin, HIGH);
    digitalWrite(rightReverse_pin, LOW);

    digitalWrite(leftForward_pin, HIGH);
    digitalWrite(leftReverse_pin, LOW);
}

// Move backward (both motors reverse)
void reverse() {
    Serial.println("Reversing");
    setSpeed(motorSpeed); // Set the speed before reversing

    digitalWrite(rightForward_pin, LOW);
    digitalWrite(rightReverse_pin, HIGH);

    digitalWrite(leftForward_pin, LOW);
    digitalWrite(leftReverse_pin, HIGH);
}

// Turn left (left motor reverse, right motor forward)
void turnLeft() {
    Serial.println("Turning left");
    setSpeed(motorSpeed); // Set the speed before turning

    digitalWrite(rightForward_pin, HIGH);
    digitalWrite(rightReverse_pin, LOW);

    digitalWrite(leftForward_pin, LOW);
    digitalWrite(leftReverse_pin, HIGH);
}

// Turn right (right motor reverse, left motor forward)
void turnRight() {
    Serial.println("Turning right");
    setSpeed(motorSpeed); // Set the speed before turning

    digitalWrite(rightForward_pin, LOW);
    digitalWrite(rightReverse_pin, HIGH);

    digitalWrite(leftForward_pin, HIGH);
    digitalWrite(leftReverse_pin, LOW);
}

// Stop the motors
void stop() {
    Serial.println("Stopping");
    //setSpeed(0); // Stop both motors by setting speed to 0
    digitalWrite(rightForward_pin, LOW);
    digitalWrite(rightReverse_pin, LOW);

    digitalWrite(leftForward_pin, LOW);
    digitalWrite(leftReverse_pin, LOW);


    stopSqueeze ();

    command = 0;


}

void toggleAutoMode(bool enable) {
    inAutoMode = enable;
    Serial.print("Auto mode ");
    Serial.println(enable ? "enabled" : "disabled");
}



void toolCommand() {
    Serial.println("Executing tool command");
    //squeeze();
    delay(2500);
    stop();
    delay(1000);
    //swing();
    release();
    delay(3000);
    stop();
    // Add your tool command code here
}

// Function to get a command from the bluetooth module
void getCommand() {
    if (Serial3.available() > 0) {
        command = Serial3.read(); // Read the command byte from the Bluetooth serial
        Serial.print("Command received: ");
        Serial.println(command); // Print the received command for debugging use serial printf command is byte
        motorSpeed = 255;
    }
}

// Function to process commands
void processCommand() {
    switch (command) {
        case 0:
            if (!inAutoMode) {
                Serial.println("Waiting for command");
            }
            break;
        case 1:
            forward();
            //release();
            break;
        case 2:
            reverse();
            //squeeze();
            break;
        case 3:
            turnLeft();
            break;
        case 4:
            turnRight();
            break;
        case 5:
            stop();
            break;
        case 6:
            toggleAutoMode(true);
            break;
        case 7:
            toggleAutoMode(false);
            break;
        case 8:
            toolCommand();
            break;
        default:
            Serial.println("Unknown command");
            break;
    }
}

void setup() {
    Serial.begin(9600);
    Serial3.begin(9600);

    setPinModes();
}

void loop() {
    getCommand();
    processCommand();
    delay(50); // Optional: add a small delay

    //fireDetected();

    //testSonar();

    //enableLinear (true);
    //delay (500);
    //squeeze();
    //moveLinear ("up");
    //delay (5000);
    //release();
    //moveLinear ("down");
    //delay (5000);
    //enableLinear (false);
    //delay (2000);

}
