#include <Arduino.h>
#include <NewPing.h> //Library for ultrasonic sensor

const int sonarMaxDistance  = 300;   // Maximum distance for the sonar sensor (in centimeters)
const byte sonarSampling    = 5;     // Number of samples to take for averaging sonar readings

//PINS ------------------------------------------------------------------

const byte flameFront_pin   = 26;   // Pin for the front flame sensor
const byte flameRight_pin   = 27;   // Pin for the right flame sensor
const byte flameLeft_pin    = 28;   // Pin for the left flame sensor

const byte sonarFront_pin   = 8;    // Pin for the front sonar sensor
const byte sonarRight_pin   = 9;    // Pin for the right sonar sensor
const byte sonarLeft_pin    = 10;   // Pin for the left sonar sensor

// Right motor pins
const byte rightEna_pin     = 3;    // Enable pin for the right motor
const byte rightForward_pin = 4;    // Forward pin for the right motor
const byte rightReverse_pin = 5;    // Reverse pin for the right motor

// Left motor pins
const byte leftEna_pin      = 6;     // Enable pin for the left motor
const byte leftForward_pin  = 7;     // Forward pin for the left motor
const byte leftReverse_pin  = 8;     // Reverse pin for the left motor

// Linear Actuator Driver Pins
const byte linearCwEn_pin   = 22;   // Clockwise enable pin for the linear actuator driver
const byte linearCcwEn_pin  = 23;   // Counterclockwise enable pin for the linear actuator driver
const byte linearCwPwm_pin  = 11;   // Clockwise PWM pin for controlling the linear actuator speed
const byte linearCcwPwm_pin = 12;   // Counterclockwise PWM pin for controlling the linear actuator speed

//OBJECTS ------------------------------------------------------------------


// Create NewPing objects for sonar sensors
NewPing sonarFront (sonarFront_pin, sonarFront_pin, sonarMaxDistance); // Front sonar
NewPing sonarRight (sonarRight_pin, sonarRight_pin, sonarMaxDistance); // Right sonar
NewPing sonarLeft (sonarLeft_pin, sonarLeft_pin, sonarMaxDistance);    // Left sonar

HardwareSerial bluetoothSerial (Serial3);

//PINMODE ------------------------------------------------------------------
void setPinModes (){
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


void setup (){
    Serial.begin (9600);
    bluetoothSerial.begin(9600);

    setPinModes();

}

void loop() {
    
}