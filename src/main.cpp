#include<Arduino.h>

byte command;

// Function to get a command from the serial communication
byte getCommand() {
  byte _command;
  if (Serial3.available() > 0) {
    _command = Serial3.read();

  } else {
    _command = ' ';
  }
  return _command;
}

// Function to handle remote-controlled mode
void remoteControlled() {
  // Get remote control command from the user
  command = getCommand();
  Serial.print("Received command: ");
  Serial.println(command);

  /*/ Execute corresponding action based on the remote command
  switch (command) {
    case 'F':
      forward();
      break;
    case 'B':
      reverse();
      break;
    case 'R':
      turnRight();
      break;
    case 'L':
      turnLeft();
      break;
    case 'V':
      //vacuumMode();
    default:
      stop();
      break;
  }*/
}




// Setup function to initialize the serial communication
void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
}
void loop() {
 
 if (Serial.available() > 0){
    Serial.println (Serial3.read());
 }
}
