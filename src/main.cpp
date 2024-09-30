#include <Arduino.h>
#include <NewPing.h> // Library for ultrasonic sensor

const int sonarMaxDistance  = 300;   // Maximum distance for the sonar sensor (in centimeters)
const byte sonarSampling    = 5;     // Number of samples to take for averaging sonar readings

byte command = 0;  // Initialize command variable
bool inAutoMode = false;

// PINS ------------------------------------------------------------------
const byte flameFront_pin   = 26;   // Pin for the front flame sensor
const byte flameRight_pin   = 27;   // Pin for the right flame sensor
const byte flameLeft_pin    = 28;   // Pin for the left flame sensor

const byte sonarFront_pin   = 8;    // Pin for the front sonar sensor
const byte sonarRight_pin   = 9;    // Pin for the right sonar sensor
const byte sonarLeft_pin    = 10;   // Pin for the left sonar sensor

// Right motor pins
const byte rightEna_pin     = 3;    // Enable pin for the right motor
const byte rightForward_pin  = 4;    // Forward pin for the right motor
const byte rightReverse_pin  = 5;    // Reverse pin for the right motor

// Left motor pins
const byte leftEna_pin      = 6;    // Enable pin for the left motor
const byte leftForward_pin   = 7;    // Forward pin for the left motor
const byte leftReverse_pin   = 8;    // Reverse pin for the left motor

// Linear Actuator Driver Pins
const byte linearCwEn_pin   = 22;   // Clockwise enable pin for the linear actuator driver
const byte linearCcwEn_pin   = 23;   // Counterclockwise enable pin for the linear actuator driver
const byte linearCwPwm_pin   = 11;   // Clockwise PWM pin for controlling the linear actuator speed
const byte linearCcwPwm_pin  = 12;   // Counterclockwise PWM pin for controlling the linear actuator speed

// OBJECTS ------------------------------------------------------------------
// Create NewPing objects for sonar sensors
NewPing sonarFront(sonarFront_pin, sonarFront_pin, sonarMaxDistance); // Front sonar
NewPing sonarRight(sonarRight_pin, sonarRight_pin, sonarMaxDistance); // Right sonar
NewPing sonarLeft(sonarLeft_pin, sonarLeft_pin, sonarMaxDistance);    // Left sonar


// PINMODE ------------------------------------------------------------------
void setPinModes() {
    // Set right motor pins
    pinMode(rightEna_pin, OUTPUT);      // Enable pin for right motor
    pinMode(rightForward_pin, OUTPUT);  // Forward control pin for right motor
    pinMode(rightReverse_pin, OUTPUT);  // Reverse control pin for right motor

    // Set left motor pins
    pinMode(leftEna_pin, OUTPUT);       // Enable pin for left motor
    pinMode(leftForward_pin, OUTPUT);   // Forward control pin for left motor
    pinMode(leftReverse_pin, OUTPUT);   // Reverse control pin for left motor

    // Set linear actuator driver pins
    pinMode(linearCwEn_pin, OUTPUT);    // Clockwise enable pin
    pinMode(linearCcwEn_pin, OUTPUT);   // Counterclockwise enable pin
    pinMode(linearCwPwm_pin, OUTPUT);   // Clockwise PWM pin
    pinMode(linearCcwPwm_pin, OUTPUT);  // Counterclockwise PWM pin
}

// Blank function stubs
void forward() {
    Serial.println("Moving forward");
    // Add your motor control code here
}

void reverse() {
    Serial.println("Reversing");
    // Add your motor control code here
}

void turnLeft() {
    Serial.println("Turning left");
    // Add your motor control code here
}

void turnRight() {
    Serial.println("Turning right");
    // Add your motor control code here
}

void stop() {
    Serial.println("Stopping");
    // Add your motor control code here
    command = 0;
}

void toggleAutoMode(bool enable) {
    inAutoMode = enable;
    Serial.print("Auto mode ");
    Serial.println(enable ? "enabled" : "disabled");
}

void toolCommand() {
    Serial.println("Executing tool command");
    // Add your tool command code here
}

// Function to get a command from the bluetooth module
void getCommand() {
    if (Serial3.available() > 0) {
        command = Serial3.read(); // Read the command byte from the Bluetooth serial
        Serial.print("Command received: ");
        Serial.println(command); // Print the received command for debugging
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
            break;
        case 2:
            reverse();
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
    delay(100); // Optional: add a small delay
}
