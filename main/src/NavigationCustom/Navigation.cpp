#include "Navigation.h"
#include "../MovementCustom/Movement.h"
#include "../PayloadCustom/Payload.h"
#include "Arduino.h"
#include "../pinLayout.h"
#include "../params.h"

Movement move;
Payload payload;
// Initialize the Navigation System
void Navigation::initNav() {
  //Set the Trig pins as output pins
  pinMode(S1Trig, OUTPUT);
  pinMode(S2Trig, OUTPUT);
  pinMode(S3Trig, OUTPUT);
  //Set the Echo pins as input pins
  pinMode(S1Echo, INPUT);
  pinMode(S2Echo, INPUT);
  pinMode(S3Echo, INPUT);

  bool mode1 = true;
  bool mode2 = false;

}

// Navigation to opposite Start area then measure object widths to locate pylon
int Navigation::pylonSearch(int position[]) {
  int i = 0, success = 0;
  int centerSensor, leftSensor, rightSensor;
  while (success == 0 && i < 10) {
    // Reset the sensors for each loop
    centerSensor = sensorTwo();
    leftSensor = sensorOne();
    rightSensor = sensorThree();

    if ((centerSensor > 0 && leftSensor > 0 && rightSensor > 0) && 
        (centerSensor < 200 && leftSensor < 200 && rightSensor < 200)) {
      success = 1;
    }
    delay(100);
    i++;
  }
  // If the sensors do not initialize, exit the function with an error
  if (success == 0) {
    return 0;
  }
  // Check the distance using the IF condition
  // Detect and Navigate to Pylon
  // Initialize measured object width
  int width = 0, payloadFound = 0;
  // Insert logic to detect pylon
  // Determine a way to measure width of objects based on sensor positions
  // if (width > 2*PYLON_WIDTH) { // Since the pylon will be the only object other than the walls, we can be very coarse in our measurements
  //   // Determine azimuth
  //   payloadFound = 1;
  //   // Navigate to center azimuth of measurement
  // }
  // Insert logic to navigate to pylon, missile guidance may work?

  if (8 >= centerSensor) {
    move.stop();
    Serial.println("stop");
    // We may want to reduce speed here
    delay(500);
    Serial.println("forward");
    move.forward();
    if (2 >= centerSensor) {
      move.stop();
      Serial.println("stop");

      // This functionality should be brought into main.ino
      int success = payload.deployPayload(); // Will need to create deployPayload() function

      if (success == true) {
        move.reverse();
        Serial.println("Reverse");
      
        if (8 >= centerSensor) {
          move.stop();
          Serial.println("stop");
          int azimuth = position[2];
          int targetAzimuth = 90;
          // Adjust to face towards the end goal zone
          while (5 <= abs(azimuth - targetAzimuth)) {
            move.left();
            delay(200);
          }
        }
      }

      }
    }
  return 1;
}

// Home in on payload once found through width measurement
int Navigation::pylonHoming(int position[]) {
  int i = 0, success = 0;
  while (success == 0 && i < 10) {
    // Reset the sensors for each loop
    int centerSensor = sensorTwo();
    int leftSensor = sensorOne();
    int rightSensor = sensorThree();

    if ((centerSensor > 0 && leftSensor > 0 && rightSensor > 0) && 
        (centerSensor < 200 && leftSensor < 200 && rightSensor < 200)) {
      success = 1;
    }
    delay(100);
    i++;
  }
    // If the sensors do not initialize, exit the function with an error
  if (success == 0) {
    return 0;
  }

  // Implement all of this

}

// Obstacle Avoidance to the Goal Zone (other end of Arena)
int Navigation::obstacleAvoidance(int position[]) {
  int i = 0, success = 0;
  int centerSensor, leftSensor, rightSensor;
  while (success == 0 && i < 10) {
    // Reset the sensors for each loop
    centerSensor = sensorTwo();
    leftSensor = sensorOne();
    rightSensor = sensorThree();

    if ((centerSensor > 0 && leftSensor > 0 && rightSensor > 0) && 
        (centerSensor < 200 && leftSensor < 200 && rightSensor < 200)) {
      success = 1;
    }
    delay(100);
    i++;
  }
    // If the sensors do not initialize, exit the function with an error
  if (success == 0) {
    return 0;
  }
  // Current implementation is jerky, requiring stops and turns
  // We can upgrade this to flow smoothly by implementing PID or something similar
  if (8 >= centerSensor) {
    move.stop();
    Serial.println("stop");
    delay(1000);
    if (leftSensor > rightSensor) {
      move.left();
      Serial.println("Left");
      delay(500);
    } else {
      move.right();
      Serial.println("Right");
      delay(500);
    }
  }
  Serial.println("Forward");
  move.forward();
  return 1;
}

//Get the sensor values
int Navigation::sensorOne() {
  //pulse output
  digitalWrite(S1Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S1Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S1Trig, LOW);

  long t = pulseIn(S1Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  Serial.println(cm);
  return cm; // Return the values from the sensor
}

//Get the sensor values
int Navigation::sensorTwo() {
  //pulse output
  digitalWrite(S2Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S2Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S2Trig, LOW);

  long t = pulseIn(S2Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  Serial.println(cm);
  return cm; // Return the values from the sensor
}

//Get the sensor values
int Navigation::sensorThree() {
  //pulse output
  digitalWrite(S3Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(S3Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(S3Trig, LOW);

  long t = pulseIn(S3Echo, HIGH);//Get the pulse
  int cm = t / 29 / 2; //Convert time to the distance
  Serial.println(cm);
  return cm; // Return the values from the sensor
}
