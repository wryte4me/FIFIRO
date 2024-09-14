#include <Arduino.h>

const byte forwardRight_pin = 5;
const byte reverseRight_pin = 4;
const byte forwardLeft_pin = 3;
const byte reverseLeft_pin = 2;

/*
const byte squeezeForward_pin = 6;
const byte squeezeBackward_pin = 7;

const byte minLimitSwitch_pin = 8;
const byte maxLimitSwitch_pin = 9;
*/
const int runPulse = 50;

const byte irFront_pin = 10;
const byte irRight_pin = 11;
const byte irLeft_pin = 12;

byte command; // The command byte to process
bool fireDetected = false;
bool inAutoMode = false;

// Function to check if a specific IR sensor detects fire
bool onFire(byte irFlameSensor_pin) {
  return digitalRead(irFlameSensor_pin) == LOW; // IR sensor returns LOW when fire is detected
}

// Function to move the robot forward
void forward() {
  Serial.println("Moving forward");
  digitalWrite(forwardRight_pin, LOW);
  digitalWrite(forwardLeft_pin, LOW);
  delay(runPulse);
}

// Function to move the robot backward
void reverse() {
  Serial.println("Moving backward");
  digitalWrite(reverseRight_pin, LOW);
  digitalWrite(reverseLeft_pin, LOW);
  delay(runPulse);
}

// Function to turn the robot to the right
void turnRight() {
  Serial.println("Turning right");
  digitalWrite(forwardLeft_pin, LOW); // Left motor moves forward
  digitalWrite(reverseRight_pin, LOW); // Right motor moves backward
  delay(runPulse);
}

// Function to turn the robot to the left
void turnLeft() {
  Serial.println("Turning left");
  digitalWrite(forwardRight_pin, LOW); // Right motor moves forward
  digitalWrite(reverseLeft_pin, LOW); // Left motor moves backward
  delay(runPulse);
}

// Function to stop the robot
void stop() {
  Serial.println("Stopping");
  digitalWrite(forwardRight_pin, HIGH);
  digitalWrite(reverseRight_pin, HIGH);
  digitalWrite(forwardLeft_pin, HIGH);
  digitalWrite(reverseLeft_pin, HIGH);
  command = 0; // Reset the command after stopping
}

void toggleAutoMode (bool isAuto){
  if (isAuto) {
    inAutoMode = true;
    Serial.println("Entering auto mode");
  } else {
    inAutoMode = false;
    stop();
  }
  command = 0;
}

// Function to handle auto mode with fire detection
void runAutoMode() {
  if (onFire(irFront_pin)) {
    Serial.println("Fire detected in front!");
    reverse();  // Move back if fire is detected in front
  } else if (onFire(irLeft_pin)) {
    Serial.println("Fire detected on the left!");
    turnRight();  // Turn right if fire is detected on the left
  } else if (onFire(irRight_pin)) {
    Serial.println("Fire detected on the right!");
    turnLeft();  // Turn left if fire is detected on the right
  } else {
    forward();  // If no fire is detected, move forward
  }

  delay (300);
}

void runManualMode (){

}



// Function to handle tool command
void toolCommand() {
  Serial.println("Executing tool command");
  command = 0;
  // Add code to execute a tool command
}

// Function to process commands
void processCommand() {
  switch (command) {
    case 0:
      !inAutoMode && Serial.println("Waiting for command");
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

// Function to get a command from the serial communication
void getCommand() {
  if (Serial3.available() > 0) {
    command = Serial3.read(); // Read the command byte
  }
}

void setupPinMode() {
  pinMode(forwardRight_pin, OUTPUT);
  pinMode(reverseRight_pin, OUTPUT);
  pinMode(forwardLeft_pin, OUTPUT);
  pinMode(reverseLeft_pin, OUTPUT);

  pinMode(irFront_pin, INPUT);
  pinMode(irRight_pin, INPUT);
  pinMode(irLeft_pin, INPUT);

  // Initialize all pins to HIGH (motors off)
  digitalWrite(forwardRight_pin, HIGH);
  digitalWrite(reverseRight_pin, HIGH);
  digitalWrite(forwardLeft_pin, HIGH);
  digitalWrite(reverseLeft_pin, HIGH);
}

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  setupPinMode();
}

void loop() {
  getCommand();
  processCommand();       // Process the command

  if (inAutoMode){
    runAutoMode ();
  } else {
    runManualMode();
  }
}
