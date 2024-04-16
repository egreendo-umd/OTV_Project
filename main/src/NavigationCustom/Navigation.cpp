#include "Navigation.h"
#include "Arduino.h"
#include "../ParamsCustom/pinLayout.h"
#include "../ParamsCustom/params.h"

int centerSensor, leftSensor, rightSensor;

// Initialize the Navigation System
Navigation::Navigation(IMovement& movementController, VisionSystemClient& Enes100) : movement(movementController) {
  //Set the Trig pins as output pins
  pinMode(S1Trig, OUTPUT);
  pinMode(S2Trig, OUTPUT);
  pinMode(S3Trig, OUTPUT);
  //Set the Echo pins as input pins
  pinMode(S1Echo, INPUT);
  pinMode(S2Echo, INPUT);
  pinMode(S3Echo, INPUT);

  this->Enes100 = &Enes100;
}

// Navigation to opposite Start area then measure object widths to locate pylon
int Navigation::pylonSearch(int position[]) {
  // Initialize the sensors
  SensorReadings readings = initializeSensors();
  if (!readings.success) {
      Enes100->println("Failed to initialize sensors. Exiting...");
      return 0;  // or handle the failure appropriately
  }

  int turnQuant;
  bool turnDir = 0;
  int currentAzimuth = position[2];
  turnQuant = abs(currentAzimuth) + 90;
  if(position[1] < 1.0) // pos A
  {
    turnQuant *= -1; 
    turnDir = 1;
  }
  else if(position[1] < 0) // Vision Error 
  {
    Enes100->println("Error in starting position");
    return 0;
  }

  movement.turn(turnQuant, turnDir*90); 
  // pos B's turn direction is already correct direction;
  // using turnOverride to make sure that the OTV doesn't overturn.
  // buffer should be tuned
  movement.forward();
  movement.move();
  pylonHoming(position);

  return 1;
}

// Conduct width measurement and home in on pylon
int Navigation::pylonHoming(int position[]) {
  // Initialize the sensors
  SensorReadings readings = initializeSensors();
  if (!readings.success) {
      Enes100->println("Failed to initialize sensors. Exiting...");
      return 0;  // or handle the failure appropriately
  }

  bool success = false;
  if (OBSTACLE_CLOSE_THRESHOLD >= readings.center) {
    movement.stop();
    Enes100->println("stop");
    // We may want to reduce speed here
    delay(500);
    Enes100->println("forward");
    movement.forward();
    movement.move(SPEED);
    if (2 >= readings.center) {
      movement.stop();
      Enes100->println("stop");

      // This functionality should be brought into main.ino
      // int success = payload.deployPayload(); // Will need to create deployPayload() function

      if (success) {
        movement.reverse();
        movement.move(SPEED);
        Enes100->println("Reverse");
      
        if (OBSTACLE_CLOSE_THRESHOLD >= readings.center) {
          movement.stop();
          Enes100->println("stop");
          int currentAzimuth = position[2];
          int targetAzimuth = 90;
          // Adjust to face towards the end goal zone
          while (5 <= abs(currentAzimuth - targetAzimuth)) {
            movement.left();
            movement.move(SPEED);
            delay(200);
          }
        }
      }

      }
    }
  return 1;
}

// Obstacle Avoidance to the Goal Zone (other end of Arena)
int Navigation::obstacleAvoidance(int currentPosition[], int targetPosition[]) {
  // Initialize the sensors
  SensorReadings readings = initializeSensors();
  if (!readings.success) {
      Enes100->println("Failed to initialize sensors. Exiting...");
      return 0;  // or handle the failure appropriately
  }

  int currentAzimuth = calculateAzimuth(currentPosition, targetPosition);
  // Current implementation is jerky, requiring stops and turns
  // We can upgrade this to flow smoothly by implementing PID or something similar
  if (OBSTACLE_CLOSE_THRESHOLD >= readings.center) {
    movement.stop();
    Enes100->println("Stop, obstacle too close");
    delay(1000);
    if (readings.left > readings.right) {
      movement.left();
      movement.move(SPEED);
      Enes100->println("Left");
      delay(500);
    } else {
      movement.right();
      movement.move(SPEED);
      Enes100->println("Right");
      delay(500);
    }
  }

  int newAzimuth = calculateAzimuth(currentPosition, targetPosition);
  if (abs(currentAzimuth - newAzimuth) > AZIMUTH_TOLERANCE*2) {
    adjustHeading(currentAzimuth, newAzimuth);
  }

  Enes100->println("Forward");
  movement.forward();
  movement.move(SPEED);
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
  Enes100->println(cm);
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
  Enes100->println(cm);
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
  Enes100->println(cm);
  return cm; // Return the values from the sensor
}

Navigation::SensorReadings Navigation::initializeSensors() {
    SensorReadings readings;
    readings.success = false;
    int attempt = 0;

    while (!readings.success && attempt < MAX_ATTEMPTS) {
        // Reset the sensors for each loop
        readings.center = sensorTwo();
        readings.left = sensorOne();
        readings.right = sensorThree();

        if ((readings.center > 0 && readings.left > 0 && readings.right > 0) && 
            (readings.center < SENSOR_MAX_THRESHOLD && readings.left < SENSOR_MAX_THRESHOLD && readings.right < SENSOR_MAX_THRESHOLD)) {
            readings.success = true;
        } else {
            delay(100);
            attempt++;
        }
    }
    // Log if the sensors do not initialize
    if (!readings.success) {
        Enes100->println("Error initializing sensors");
    }
    return readings;
}

Navigation::SensorReadings Navigation::calibrateSensors() {
  SensorReadings readings = initializeSensors();
  if (!readings.success) {
      Enes100->println("Failed to initialize sensors. Exiting...");
      return readings;  // or handle the failure appropriately
  }

  // Figure out how to calibrate the sensors
  // We should start from a known position, and find any offsets in the sensor readings
  // Return should be the offset values for each sensor
  // How to incorporate this?

  return readings;
}


int Navigation::calculateAzimuth(int currentPosition[], int targetPosition[]) {
  int deltaX = targetPosition[0] - currentPosition[0];
  int deltaY = targetPosition[1] - currentPosition[1];
  int azimuth = atan2(deltaY, deltaX) * 180 / PI;
  return azimuth;
}

int Navigation::calculateDistance(int currentPosition[], int targetPosition[]) {
  int deltaX = targetPosition[0] - currentPosition[0];
  int deltaY = targetPosition[1] - currentPosition[1];
  int distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  return distance;
}

int Navigation::adjustHeading(int currentAzimuth, int newAzimuth) {
  while (AZIMUTH_TOLERANCE <= abs(currentAzimuth - newAzimuth)) {
    if (currentAzimuth > newAzimuth) {
      movement.left();
      movement.move(SPEED);
      delay(200);
    } else {
      movement.right();
      movement.move(SPEED);
      delay(200);
    }
  }
  return 1;
}

void Navigation::calculateTargetPosition(int currentPosition[], int currentAzimuth, int centerSensorReading, int targetPosition[]) {
    // Convert azimuth to radians for trigonometric functions
    double azimuthInRadians = currentAzimuth * (M_PI / 180.0);

    // Assuming the center sensor reading gives a straight-line distance to the obstacle,
    // and we want to set the target position just before the obstacle
    double distanceToTarget = centerSensorReading - TARGET_STOP_DISTANCE; // Stop 1 unit before the obstacle

    // Calculate the change in position
    double deltaX = distanceToTarget * cos(azimuthInRadians);
    double deltaY = distanceToTarget * sin(azimuthInRadians);

    // Calculate the new target position
    targetPosition[0] = currentPosition[0] + static_cast<int>(deltaX);
    targetPosition[1] = currentPosition[1] + static_cast<int>(deltaY);
}

// Function to move forward for a set distance at a given speed
void Navigation::moveForwardSetDistance(double speed, double distance) {
    double travelTime = distance / speed; // Time in seconds to travel the given distance

    unsigned long startTime = millis();
    while (millis() - startTime < travelTime * 1000) { // Multiply by 1000 to convert seconds to milliseconds
        SensorReadings readings = initializeSensors();
        if (!readings.success) {
            Enes100->println("Failed to initialize sensors. Exiting...");
            return 0;  // or handle the failure appropriately
        }
        if (readings.center <= OBSTACLE_CLOSE_THRESHOLD || readings.center >= SENSOR_MAX_THRESHOLD) {
            movement.stop();
            Enes100->println("Obstacle detected or reached max threshold, stopping");
            return;
        }

        movement.forward(); // set forward direction
        movement.move(speed); // set speed
        delay(100); // Small delay to allow sensor reading update, adjust as necessary
    }

    movement.stop(); // Stop after traveling the set distance
    Enes100->println("Reached target distance, stop");
}

Navigation::ScanResult Navigation::scanAndCalculateWidth(int position[]) {
    // Start by taking a measurement directly in front
    SensorReadings readings = initializeSensors();
    if (!readings.success) {
        Enes100->println("Failed to initialize sensors. Exiting...");
        return ScanResult{0, 0, 0, false};  // or handle the failure appropriately
    }
    SensorReadings measurements[SCAN_SECTORS];
    int azimuths[SCAN_SECTORS];
    bool objectInView = false;

    int widthThreshold = readings.center > OBSTACLE_CLOSE_THRESHOLD / 2 ? readings.center / 3 : 2;

    if (readings.center <= OBSTACLE_CLOSE_THRESHOLD && (abs(readings.left - readings.center) >= widthThreshold && abs(readings.right - readings.center) >= widthThreshold)) {
        double width = calculateObjectWidth(readings.left, readings.center, readings.right);

        if (width < PYLON_WIDTH * 2) {
            objectInView = true;
            Enes100->print("Detected object width is within threshold: ");
            Enes100->println(width);
        } else {
            objectInView = false; // Assuming you want to set this to false if the object is too wide
            Enes100->print("Detected width too large: ");
            Enes100->println(width);
            return ScanResult{0, 0, 0, false};
        }
    } else {
        objectInView = false; // Ensure objectInView is set correctly if no close obstacle is detected
        Enes100->println("No object detected in front");
        return ScanResult{0, 0, 0, false};
    }

    // Scan to the left
    movement.turn(SCAN_ANGLE/2);
    double leftAzimuth = position[2] - SCAN_ANGLE/2;
    double objectStartAzimuth, objectEndAzimuth;

    int idx = 0;
    for (double angle = leftAzimuth; angle <= leftAzimuth + SCAN_ANGLE; angle += SCAN_ANGLE / SCAN_SECTORS) {
      SensorReadings scanReading = initializeSensors();
      if (!scanReading.success) {
          Enes100->println("Failed to initialize sensors. Exiting...");
          return ScanResult{0, 0, 0, false};  // or handle the failure appropriately
      }
      measurements[idx] = scanReading;
      azimuths[idx] = angle;

      movement.turn(SCAN_ANGLE / SCAN_SECTORS);  // Incremental scanning step
      idx++;
    }

    // Return to initial position
    movement.turn(-SCAN_ANGLE/2);

    double leftEdgeAzimuth = 0;
    double rightEdgeAzimuth = 0;
    bool objectDetected = false;

    // Find the edges of the object based on the sensor readings
    for (int i = 0; i < SCAN_SECTORS; i++) {
        const SensorReadings& reading = measurements[i];
        double currentAzimuth = azimuths[i];

        // Check for the start of the object
        if (reading.center > 0 && reading.center <= OBSTACLE_CLOSE_THRESHOLD && !objectDetected) {
            leftEdgeAzimuth = currentAzimuth;
            objectDetected = true;  // Mark that we've started detecting an object
        }
        // If the object was previously detected and we've now moved past it
        else if (reading.center > OBSTACLE_CLOSE_THRESHOLD && objectDetected) {
            rightEdgeAzimuth = currentAzimuth;
            break;  // Stop the loop after finding the right edge of the object
        }
    }
    if (objectDetected && leftEdgeAzimuth != 0 && rightEdgeAzimuth != 0) {
        Enes100->print("Left edge of object detected at azimuth: ");
        Enes100->println(leftEdgeAzimuth);
        Enes100->print("Right edge of object detected at azimuth: ");
        Enes100->println(rightEdgeAzimuth);

        // Calculate object width or other processing based on the detected edges
        // The width calculation could be refined based on the specific geometry and scanning logic
    } else {
        Enes100->println("No object detected within the scanning range.");
        return ScanResult{0, 0, 0, false};
    }
    // Calculate the width based on the collected data
    int objectWidth = (int)calculateWidthFromMeasurements(measurements, SCAN_SECTORS);

    ScanResult result = {leftEdgeAzimuth, rightEdgeAzimuth, objectWidth, objectDetected};

    return result;
}

double Navigation::calculateWidthFromMeasurements(const SensorReadings measurements[], int numMeasurements) {
    // Logic to calculate the object's width based on multiple sensor readings
    double widthSum = 0.0;
    for (int i = 0; i < numMeasurements; i++) {
        double width = calculateObjectWidth(measurements[i].left, measurements[i].center, measurements[i].right);
        widthSum += width;
    }
    double averageWidth = numMeasurements > 0 ? widthSum / numMeasurements : 0.0;
    return averageWidth;
}

double Navigation::calculateObjectWidth(int leftSensor, int centerSensor, int rightSensor) {
    // Convert angles to radians for trigonometric calculations
    double leftAngleRadians = LEFT_ANGLE_OFFSET * (M_PI / 180.0);
    double rightAngleRadians = RIGHT_ANGLE_OFFSET * (M_PI / 180.0);

    // Calculate the base of the triangles formed by each sensor with the object
    double leftBase = leftSensor * tan(leftAngleRadians);
    double rightBase = rightSensor * tan(rightAngleRadians);

    // The approximate width of the object is the sum of the bases of the two triangles
    // This assumes the object's width is large enough that the center sensor is also aligned with it
    double objectWidth = leftBase + rightBase;

    return objectWidth;
}