//  TODO - WIRING LIMIT SWITCH FOR SQUEEZER
//  TODO - FIX ISSUE WITH ULTRASONIC SENSOR FALSE READINGS
//  TODO - PROGRAM AUTONOMOUS NAVIGATION AND FIRE SUPPRESSION
//  TODO - 3D PRINT COVER OF THE ELECTRONICS CASING



#include <Arduino.h>
#include <NewPing.h> // Library for ultrasonic sensor
#include <LibPrintf.h>

// Global variable for motor speed (default to 255 for maximum speed)
int motorSpeed = 0;

const int sonarMaxDistance  = 400;   // Maximum distance for the sonar sensor (in centimeters)
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
const byte allFlameNumRead = 10;
const byte maxLimit_pin = A1;   
const byte minLimit_pin = A0;

const byte sonarFront_pin   = 8;    // Pin for the front sonar sensor
const byte sonarRight_pin   = 10;    // Pin for the right sonar sensor
const byte sonarLeft_pin    = 9;   // Pin for the left sonar sensor

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
const byte linearCwPwm_pin  = 12;   // Clockwise PWM pin for controlling the linear actuator speed
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

    // Set limit switch pins
    pinMode(maxLimit_pin, INPUT);
    pinMode(minLimit_pin, INPUT);
}

void moveLinear(String direction) {
    if (direction == "down") {
        // Move linear actuator down (clockwise)
        digitalWrite(linearCcwPwm_pin, LOW); // Set counterclockwise PWM pin to LOW
        analogWrite(linearCwPwm_pin, 255);   // Set clockwise PWM pin to max value (255)
        Serial.println("Moving linear actuator down.");
    } else if (direction == "up") {
        // Move linear actuator up (counterclockwise)
        digitalWrite(linearCwPwm_pin, LOW);  // Set clockwise PWM pin to LOW
        analogWrite(linearCcwPwm_pin, 255);  // Set counterclockwise PWM pin to max value (255)
        Serial.println("Moving linear actuator up.");
    } else if (direction == "stop") {
        // Stop the linear actuator
        analogWrite(linearCwPwm_pin, 0);     // Set clockwise PWM pin to LOW
        analogWrite(linearCcwPwm_pin, 0);    // Set counterclockwise PWM pin to LOW
        Serial.println("Stopping linear actuator.");
    } else {
        Serial.println("Invalid direction. Please use 'up', 'down', or 'stop'.");
    }
}

void stopSqueeze (){
    moveLinear("stop");
}

void squeeze (){
    moveLinear("down");
        while (digitalRead(minLimit_pin)==LOW){
        Serial.println ("Moving down");
    }
    stopSqueeze();
}

void release (){
    moveLinear("up");
    while (digitalRead(maxLimit_pin)==LOW){
        Serial.println ("Moving up");
    }
    stopSqueeze();
    
}

void measureDistance() {
    // Measure distances from each sonar sensor
    leftDistance = sonarLeft.ping_cm();
    delay(10);
    frontDistance = sonarFront.ping_cm();
    delay(10);
    rightDistance = sonarRight.ping_cm();
    delay(10);

    // Print the values in the format: "Left: XX cm | Front: XX cm | Right: XX cm"
    printf("Left: %d cm | Front: %d cm | Right: %d cm\n", leftDistance, frontDistance, rightDistance);
}


bool onFire (byte flameSensor_pin){
    return digitalRead(flameSensor_pin) == LOW;
}

// Function to determine flame detection using majority voting
bool irPositive(byte flameSensor_pin) {
    int flameDetectedCount = 0;   // Counter for flame detections
    bool detected = false;
    bool detectedResult = false;
    // Read the sensor multiple times
    for (int i = 0; i < flameNumRead; i++) {
        detected = onFire(flameSensor_pin);
        Serial.print(detected ? 1:0 );
        if (detected) {
            flameDetectedCount++;
        }
        delay(50); // Short delay between readings to allow for sensor stability
    }

    // Return true if the majority of readings indicate flame detection
    detectedResult = (flameDetectedCount > flameNumRead / 2);
    Serial.print ("\t");
    Serial.println (detectedResult);
    return detectedResult;
}

// Function to check if at least two sensors detect flame
bool fireDetected() {
    int flame1DetectedCount = 0;
    int flame2DetectedCount = 0;
    int flame3DetectedCount = 0;

    // Perform multiple readings for each sensor and count positive detections
    for (int i = 0; i < allFlameNumRead; i++) {
        if (irPositive(flame1)) flame1DetectedCount++;
        if (irPositive(flame2)) flame2DetectedCount++;
        if (irPositive(flame3)) flame3DetectedCount++;

        delay(50); // Delay between readings for sensor stability
    }

    // Check if at least two of the sensors have a majority of positive detections
    int positiveCount = 0;
    if (flame1DetectedCount > allFlameNumRead / 2) positiveCount++;
    if (flame2DetectedCount > allFlameNumRead / 2) positiveCount++;
    if (flame3DetectedCount > allFlameNumRead / 2) positiveCount++;

    return (positiveCount >= 2);
}

// Function to set motor speed
void setSpeed() {
    // Ensure the speed is within the valid PWM range (0 to 255)
    if (inAutoMode) {
        motorSpeed = 200; // Minimum speed
    } else {
        motorSpeed = 255; // Maximum speed
    }

    // Set the motor speed (PWM value) for both left and right motors
    analogWrite(rightEna_pin, motorSpeed);  // Set speed for right motor
    analogWrite(leftEna_pin, motorSpeed);   // Set speed for left motor
}

// Function to move the robot forward
void forward() {
    Serial.println("Moving forward");
    setSpeed(); // Set the speed before moving forward

    digitalWrite(rightReverse_pin, LOW);
    digitalWrite(leftReverse_pin, LOW);
    digitalWrite(rightForward_pin, HIGH);
    digitalWrite(leftForward_pin, HIGH);
}

// Function to move the robot backwards
void reverse() {
    Serial.println("Reversing");
    setSpeed(); // Set the speed before reversing

    digitalWrite(rightForward_pin, LOW);
    digitalWrite(leftForward_pin, LOW);
    digitalWrite(rightReverse_pin, HIGH);
    digitalWrite(leftReverse_pin, HIGH);
}

// Function to turn the robot to the left
void turnLeft() {
    Serial.println("Turning left");
    setSpeed(); // Set the speed before turning

    digitalWrite(leftForward_pin, LOW);
    digitalWrite(rightReverse_pin, LOW);
    digitalWrite(rightForward_pin, HIGH);
    digitalWrite(leftReverse_pin, HIGH);
}

// Function to turn the robot to the right
void turnRight() {
    Serial.println("Turning right");
    setSpeed(); // Set the speed before turning

    digitalWrite(rightForward_pin, LOW);
    digitalWrite(leftReverse_pin, LOW);
    digitalWrite(leftForward_pin, HIGH);        
    digitalWrite(rightReverse_pin, HIGH); 
}

// Function to stop the robot
void stop() {
    Serial.println("Stopping");
    digitalWrite(rightForward_pin, LOW);        //--------------------------------------->      Pull the motor driver control pins low
    digitalWrite(rightReverse_pin, LOW);
    digitalWrite(leftForward_pin, LOW);
    digitalWrite(leftReverse_pin, LOW);


    //stopSqueeze ();
    command = 0;                                //--------------------------------------->      Reset command variable
}

void toggleAutoMode(bool enable) {
    inAutoMode = enable;
    Serial.print("Auto mode ");
    Serial.println(enable ? "enabled" : "disabled");

    command = 0;
}



void toolCommand() {
    Serial.println("Executing tool command");
    squeeze();
    stopSqueeze();
    delay (1000);
    release();
    stopSqueeze();
    command = 0;

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

// Function to test the limit switch
void testLimitSwitch (){
    Serial.print(digitalRead(minLimit_pin));
    Serial.println(digitalRead(maxLimit_pin));
    delay(1000);

    //00 no limit switch is triggered
    //01 max limit switch is triggered
    //10 min limit switch is triggered
    //11 both limit switch is triggered
}

void testIrFlameSensor (){
    // Print the status of each flame sensor in sequence
    Serial.print("Flame sensors status: ");
    Serial.print(irPositive(flame1) ? "1" : "0");
    Serial.print(irPositive(flame2) ? "1" : "0");
    Serial.print(irPositive(flame3) ? "1" : "0");
    Serial.print(irPositive(flame4) ? "1" : "0");
    Serial.print(irPositive(flame5) ? "1" : "0");
    Serial.print(irPositive(flame6) ? "1" : "0");
    Serial.println();  // Move to the next line for clarity in output

    delay(1000); // Delay before next check
}

// Function to process commands
void processCommand() {
    switch (command) {
        case 0:
            if (!inAutoMode) {
                Serial.print("Waiting for command \t | ");
            }
            break;
        case 1:
            Serial.println("Forward");
            forward();
            break;
        case 2:
            Serial.println("Reverse");
            reverse();
            break;
        case 3:
            Serial.println("Left");
            turnLeft();
            break;
        case 4:
            Serial.println("Right");
            turnRight();
            break;
        case 5:
            Serial.println("Stop");
            stop();
            break;
        case 6:
            Serial.println("Auto mode");
            toggleAutoMode(true);
            break;
        case 7:
            Serial.println("Manual mode");
            toggleAutoMode(false);
            break;
        case 8:
            Serial.println("Tool command");
            toolCommand();
            break;
        default:
            Serial.println("Unknown command");
            break;
    }
}

void setupLinear (){
    release();
    stopSqueeze();
}

void setup() {
    Serial.begin(9600);
    Serial3.begin(9600);
    printf_init(Serial);  // Initialize printf with Serial output

    setPinModes();
    setupLinear();
    
}

void loop() {
    printf("Mode: %s \t | ", inAutoMode ? "AUTO":"MANUAL");
    getCommand();
    processCommand();
    delay (500);

    if (inAutoMode){
        Serial.println("RUNNING AUTO MODE");
    } else{
        Serial.println("RUNNING MANUAL MODE");
    }

    //testLimitSwitch ();
}
