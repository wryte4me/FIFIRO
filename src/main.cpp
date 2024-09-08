#include <Arduino.h>

byte command; // The command byte to process

// Function to move the robot forward
void forward() {
  Serial.println("Moving forward");
  // Add code to move forward
}

// Function to move the robot backward
void reverse() {
  Serial.println("Moving backward");
  // Add code to move backward
}

// Function to turn the robot to the right
void turnRight() {
  Serial.println("Moving right");
  // Add code to turn right
}

// Function to turn the robot to the left
void turnLeft() {
  Serial.println("Moving left");
  // Add code to turn left
}

// Function to stop the robot
void stop() {
  Serial.println("Stopping");
  // Add code to stop the robot
  command = 0;
}

// Function to handle manual mode
void manualMode() {
  Serial.println("Entering manual mode");
  command = 0;
  // Add code for manual mode
}

// Function to handle auto mode
void autoMode() {
  Serial.println("Entering auto mode");
  // Add code for auto mode
  command = 0;
}

// Function to handle tool command
void toolCommand() {
  Serial.println("Executing tool command");
  // Add code for tool command
  command = 0;
}

// Function to process commands
void processCommand() {  
  switch (command) {
    case 0:
      Serial.println ("Waiting for command");
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
      manualMode();
      break;
    case 7:
      autoMode();
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

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop() {
  getCommand();
  processCommand();       // Process the command
}
