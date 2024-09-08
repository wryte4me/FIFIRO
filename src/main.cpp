#include <Arduino.h>
#include "Servo.h"

// Pin assignments
const byte rightMotorsForward_pin = 2;  /** @brief Pin for right motors forward */
const byte rightMotorsReverse_pin = 3;  /** @brief Pin for right motors reverse */
const byte leftMotorsForward_pin = 4;   /** @brief Pin for left motors forward */
const byte leftMotorsReverse_pin = 5;   /** @brief Pin for left motors reverse */

const byte servo_pin = 6;               /** @brief Pin for the servo motor */

// Duration in milliseconds to run the motors
const int runTime = 0;  /**< @brief 2 seconds (adjust as needed) */

Servo servoMotor;  // Create a servo object
int command = 0;
/**
 * @brief Turn off all motor control pins.
 */
void stopAllMotors() {
  digitalWrite(rightMotorsForward_pin, LOW);
  digitalWrite(rightMotorsReverse_pin, LOW);
  digitalWrite(leftMotorsForward_pin, LOW);
  digitalWrite(leftMotorsReverse_pin, LOW);
}

/**
 * @brief Set right and left motors to move forward.
 */
void moveForward() {
  digitalWrite(rightMotorsReverse_pin, LOW);
  digitalWrite(leftMotorsReverse_pin, LOW);
  digitalWrite(rightMotorsForward_pin, HIGH);
  digitalWrite(leftMotorsForward_pin, HIGH);
  delay(runTime);
  stopAllMotors();
}

/**
 * @brief Set right and left motors to move in reverse.
 */
void moveReverse() {
  digitalWrite(rightMotorsForward_pin, LOW);
  digitalWrite(leftMotorsForward_pin, LOW);
  digitalWrite(rightMotorsReverse_pin, HIGH);
  digitalWrite(leftMotorsReverse_pin, HIGH);
  delay(runTime);
  stopAllMotors();
}

/**
 * @brief Set left motors to reverse and right motors to move forward.
 */
void turnLeft() {
  digitalWrite(rightMotorsReverse_pin, LOW);
  digitalWrite(leftMotorsForward_pin, LOW);
  digitalWrite(rightMotorsForward_pin, HIGH);
  digitalWrite(leftMotorsReverse_pin, HIGH);
  delay(runTime);
  stopAllMotors();
}

/**
 * @brief Set right motors to reverse and left motors to move forward.
 */
void turnRight() {
  digitalWrite(leftMotorsReverse_pin, LOW);
  digitalWrite(rightMotorsForward_pin, LOW);
  digitalWrite(rightMotorsReverse_pin, HIGH);
  digitalWrite(leftMotorsForward_pin, HIGH);
  delay(runTime);
  stopAllMotors();
}

/**
 * @brief Process remote commands and control motors and servo.
 * @param command The command byte to process.
 */
void processRemoteCommand(byte command) {
  
}

void setup() {
  // Set motor control pins as outputs
  pinMode(rightMotorsForward_pin, OUTPUT);
  pinMode(rightMotorsReverse_pin, OUTPUT);
  pinMode(leftMotorsForward_pin, OUTPUT);
  pinMode(leftMotorsReverse_pin, OUTPUT);
  

  // Initialize the servo motor
  //servoMotor.attach(servo_pin);

  // Initialize serial communication
  Serial.begin(9600);
  Serial3.begin (9600);
}

void loop() {
  // Example to simulate receiving commands via serial (change as per your input method)
  if (Serial3.available()) {
    command = Serial3.read();
    Serial.println(command);
    //processRemoteCommand(command);  // Process the received command
  } else {
    //Serial.println ("Waiting for command");
  }

  delay(10);  // Small delay to avoid overwhelming serial input
}
