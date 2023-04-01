#include <Servo.h>
#include <math.h>

// Define the lengths of the two links
const double L1 = 10.0; // length of the first link
const double L2 = 8.0; // length of the second link

// Define the servo pins
const int servo1Pin = 10; // servo 1 pin
const int servo2Pin = 1; // servo 2 pin

// Create servo objects
Servo servo1;
Servo servo2;

// Define the current and desired (x, y) coordinates of the end effector
double xCurrent = 0.0;
double yCurrent = 0.0;
double xDesired = 5.0;
double yDesired = 5.0;

// Define the tolerance for the error in the end effector position
const double tolerance = 0.1;

void setup() {
  // Attach the servos to their pins
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Move the end effector to the desired position
  moveToEndEffectorPosition(xDesired, yDesired);
}

void loop() {
  // Do nothing
}

// Function to move the end effector to the desired (x, y) position
void moveToEndEffectorPosition(double xDesired, double yDesired) {
  // Calculate the initial error in the end effector position
  double error = sqrt(pow(xDesired - xCurrent, 2) + pow(yDesired - yCurrent, 2));
  
  // Loop until the error is less than the tolerance
  while (error > tolerance) {
    // Calculate the joint angles using inverse kinematics
    double theta1 = atan2(yDesired, xDesired) - acos((xDesired*xDesired + yDesired*yDesired + L1*L1 - L2*L2) / (2 * L1 * sqrt(xDesired*xDesired + yDesired*yDesired)));
    double theta2 = acos((xDesired*xDesired + yDesired*yDesired - L1*L1 - L2*L2) / (2 * L1 * L2));

    // Convert the angles to degrees
    theta1 = theta1 * 180 / PI;
    theta2 = theta2 * 180 / PI;

    // Set the servo angles to the calculated joint angles
    servo1.write(theta1);
    servo2.write(theta2);

    // Update the current position of the end effector
    xCurrent = L1*cos(theta1*PI/180) + L2*cos((theta1 + theta2)*PI/180);
    yCurrent = L1*sin(theta1*PI/180) + L2*sin((theta1 + theta2)*PI/180);

    // Calculate the error in the end effector position
    error = sqrt(pow(xDesired - xCurrent, 2) + pow(yDesired - yCurrent, 2));

    // Wait for the servos to reach the new position
    delay(10);
  }
}
