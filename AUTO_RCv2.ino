#include <NewPing.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// Motor control pins
const byte leftForward = 9;
const byte leftBackward = 8;
const byte rightForward = 10;
const byte rightBackward = 11;
const byte brushForward = 6;
const byte brushBackward = 7;
const byte brushEna = A5;
const byte wheelEna = A4;

const byte vacuum_Pin = 5;
const int vacuumSpeed = 1600;

const int brushSpeed = 130;
const int wheelSpeed = 130;

const byte left_pin = 4;
const byte right_pin = 12;
const byte front_pin = 13;
const int maxSonarDistance = 400;

NewPing sonarRight(right_pin, right_pin, maxSonarDistance);
NewPing sonarLeft(left_pin, left_pin, maxSonarDistance);
NewPing sonarFront(front_pin, front_pin, maxSonarDistance);

SoftwareSerial BT_Controller(3, 2);  // RX, TX

Servo vacuum;

int front_distance;
int right_distance;
int left_distance;
int avoid_distance = 20;

byte mode = 0;  //0 for RC, 1 for Auto
byte remoteInput;
bool brushOn = false;
bool vacuumOn = false;

void setup() {
  Serial.begin(9600);
  BT_Controller.begin(9600);
  vacuum.attach(vacuum_Pin);
  while (!vacuum.attached()) {
    Serial.print("xx");
    //delay(20000);
  }  //set speed of motors
  Serial.println("Vacuum initiated");

  //setupMotors
  pinMode(leftForward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(brushEna, OUTPUT);
  pinMode(wheelEna, OUTPUT);

  analogWrite(brushEna, brushSpeed);
  analogWrite(wheelEna, wheelSpeed);
}

void loop() {
  remoteInput = getCommand();
  Serial.print("Received command: ");
  Serial.println(remoteInput);

  setMode();

  if (remoteInput == 8) {
    
    updateBrushStatus();
    updateVacuumStatus();
  }

  if (mode == 0) {
    remoteControl();
  } else if (mode == 1) {
    autoRun();
  }
}

void setMode() {
  if (remoteInput == 6) {
    // Set MODE to 1 for AUTO
    mode = 1;
    Serial.println("Auto pilot mode!");
  } else if (remoteInput == 7) {
    // Set MODE to 0 for RC
    mode = 0;
    robotMove('S');
    Serial.println("Remote Control MOde");
  }
}

void printDistances() {
  Serial.print(left_distance);
  Serial.print(" | ");
  Serial.print(front_distance);
  Serial.print(" | ");
  Serial.println(right_distance);
}

//modify this function (use char F, B, R, L, S for forward, backward, right, left, and stop respectively).
//algorithm
//if all distance is more than avoid distance. move forward
// if front distance is less than avoid distance and if right distance is greater than avoid distance turn right
// if front distance is less than avoid distance and if left distance is greater than avoid distance turn left
// if all distance is less than avoid distance move backwards for 1 second
void autoRun() {
  measureDistance();
  printDistances();

  if (front_distance < avoid_distance && left_distance < avoid_distance && front_distance < avoid_distance) {
    // Left distance is less than avoid distance and front distance is less than avoid distance,
    // no need to turn, just move forward
    robotMove('B');
  } else if (front_distance < avoid_distance && right_distance >= avoid_distance) {
    // Front distance is less than avoid distance and right distance is greater than or equal to avoid distance, turn right
    robotMove('R');
    delay(1000);
  } else if (front_distance < avoid_distance && left_distance >= avoid_distance) {
    // Front distance is less than avoid distance and left distance is greater than or equal to avoid distance, turn left
    robotMove('L');
    delay(1000);
  } else if (left_distance < avoid_distance && right_distance < avoid_distance && front_distance >= avoid_distance) {
    // Left distance is less than avoid distance, right distance is less than avoid distance,
    // and front distance is greater than or equal to avoid distance, move forward
    robotMove('F');
  } else {
    // Default case, move forward
    robotMove('F');
  }
}

void measureDistance() {
  front_distance = sonarFront.ping_cm();
  delay(10);
  right_distance = sonarRight.ping_cm();
  delay(10);
  left_distance = sonarLeft.ping_cm();
  delay(10);
}

void robotMove(char dir) {
  switch (dir) {
    case 'F':
      motorRun(HIGH, HIGH, LOW, LOW);  //forward
      break;
    case 'B':
      motorRun(LOW, LOW, HIGH, HIGH);  //reverse
      break;
    case 'L':
      motorRun(LOW, HIGH, HIGH, LOW);  //right
      break;
    case 'R':
      motorRun(HIGH, LOW, LOW, HIGH);  //left
      break;
    case 'S':
      motorRun(LOW, LOW, LOW, LOW);  //stop
      break;
    default:
      break;
  }
}


// Function to control the motors based on input commands
void motorRun(bool rForward, bool lForward, bool rBackward, bool lBackward) {
  digitalWrite(rightForward, rForward);
  digitalWrite(leftForward, lForward);
  digitalWrite(rightBackward, rBackward);
  digitalWrite(leftBackward, lBackward);
}
void remoteControl() {
  // Execute corresponding action based on the remote command
  switch (remoteInput) {
    case 1:
      robotMove('F');
      break;
    case 2:
      robotMove('B');
      break;
    case 3:
      robotMove('R');
      break;
    case 4:
      robotMove('L');
      break;
    case 5:
      robotMove('S');
      break;
    default:
      break;
  }
}

// Function to get a command from the serial communication
byte getCommand() {
  byte command;
  if (BT_Controller.available() > 0) {  //if some date is sent, reads it and saves in state
    command = BT_Controller.read();
  }
  return command;
}
void updateBrushStatus() {
  brushOn = !brushOn;  // Toggle brush status
  runBrush(brushOn);   // Turn brush on or off based on brushOn variable
  if (brushOn) {
    Serial.println("Brush On");
  } else {
    Serial.println("Brush Off");
  }
}

void updateVacuumStatus() {
  vacuumOn = !vacuumOn;  // Toggle vacuum status
    // Turn vacuum on or off based on vacuumOn variable
  if (vacuumOn) {
    vacuum.write (vacuumSpeed);
    Serial.println("Vacuum On");
  } else {
    vacuum.write (0);
    Serial.println("Vacuum Off");
  }
}
void runBrush(bool runBr) {

  if (runBr) {
    digitalWrite(brushForward, HIGH);
    digitalWrite(brushBackward, LOW);
    Serial.println("Brush turned on");
  }
  if (!runBr) {
    digitalWrite(brushForward, LOW);
    digitalWrite(brushBackward, LOW);
    Serial.println("Brush turned off");
  }
}

