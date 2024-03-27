// Include parameter and pin layout files
#include <params.h>
#include <pinLayout.h>

// Initialize the Navigation System
void initNav() {
  //Set the Trig pins as output pins
  pinMode(S1Trig, OUTPUT);
  pinMode(S2Trig, OUTPUT);
  pinMode(S3Trig, OUTPUT);
  //Set the Echo pins as input pins
  pinMode(S1Echo, INPUT);
  pinMode(S2Echo, INPUT);
  pinMode(S3Echo, INPUT);
  //Set the speed of the motors
  // motorLeft.setSpeed(Speed);
  // motorRight.setSpeed(Speed);
  int mode1 = true;
  int mode2 = false;

}

void navMode1() {
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
  // Check the distance using the IF condition
  // Query the Overhead Vision System
  int mode = 1; // queryOVS(); // queryOVS will probably be an int array return?
  // Detect and Navigate to Pylon
  if (mode == 1) {

    // Initialize measured object width
    int width = 0;
    // Insert logic to detect pylon
    // Determine a way to measure width of objects based on sensor positions
    if (width > 2*PYLON_WIDTH) { // Since the pylon will be the only object other than the walls, we can be very coarse in our measurements
      // Determine azimuth
      // Navigate to center azimuth of measurement
    }
    // Insert logic to navigate to pylon, missile guidance may work?

    if (8 >= centerSensor) {
      Stop();
      Serial.println("Stop");
      // motorLeft.setSpeed(Speed/4);
      // motorRight.setSpeed(Speed/4);
      delay(500);
      Serial.println("forward");
      forward();
      if (2 >= centerSensor) {
        Stop();
        Serial.println("Stop");

        int success = deployPayload(); // Will need to create deployPayload() function

        if (success == True) {
          reverse();
          Serial.println("Reverse");
        
          if (8 >= centerSensor) {
            Stop();
            Serial.println("Stop");
            int azimuth = queryOVS(); // If it's an array, extract just the azimuth
            int targetAzimuth = 90;
            // Adjust to face towards the end goal zone
            while (5 <= abs(azimuth - targetAzimuth)) {
              left();
              delay(200);
              
            }
          }
        }

      }
    }

    continue;
  }
}
int navMode2() {
  // Obstacle Avoidance
  if (mode == 2) { 
    if (8 >= centerSensor) {
      Stop();
      Serial.println("Stop");
      delay(1000);
      if (leftSensor > rightSensor) {
        left();
        Serial.println("Left");
        delay(500);
      } else {
        right();
        Serial.println("Right");
        delay(500);
      }
    }
  }
    Serial.println("Forward");
    forward();
    // Serial.print(leftSensor);
    // Serial.print(", ");
    // Serial.print(centerSensor);
    // Serial.print(", ");
    // Serial.print(rightSensor);
    // Serial.print('\n');
  }
  return 1;

}

// Query OVS for position
int queryOVS() {
  //int[] position = {150, 50};
  int mode;

  // Insert logic to query OVS through the ESP32 WiFi Modem

  if (position[0] < 100) {
    mode = 1;
  } else {
    mode = 2;
  }
  return mode;
}

//Get the sensor values
int sensorOne() {
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
int sensorTwo() {
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
int sensorThree() {
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

/*******************Motor functions**********************/
void forward() {
  // motorLeft.run(FORWARD);
  // motorRight.run(FORWARD);
}
void left() {
  // motorLeft.run(BACKWARD);
  // motorRight.run(FORWARD);
}
void right() {
  // motorLeft.run(FORWARD);
  // motorRight.run(BACKWARD);
}
void Stop() {
  // motorLeft.run(RELEASE);
  // motorRight.run(RELEASE);
}
void reverse() {
  // motorLeft.run(BACKWARD);
  // motorRight.run(BACKWARD);
}