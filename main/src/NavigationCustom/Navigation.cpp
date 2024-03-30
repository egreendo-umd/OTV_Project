#include "Navigation.h"
#include "Arduino.h"
#include <cmath>
#include <vector>
#include "../ParamsCustom/pinLayout.h"
#include "../ParamsCustom/params.h"

int centerSensor, leftSensor, rightSensor;

// Initialize the Navigation System
Navigation::Navigation(IMovement& movementController) : movement(movementController) {
  //Set the Trig pins as output pins
  pinMode(S1Trig, OUTPUT);
  pinMode(S2Trig, OUTPUT);
  pinMode(S3Trig, OUTPUT);
  //Set the Echo pins as input pins
  pinMode(S1Echo, INPUT);
  pinMode(S2Echo, INPUT);
  pinMode(S3Echo, INPUT);
}

// Navigation to opposite Start area then measure object widths to locate pylon
int Navigation::pylonSearch(int position[]) {
  // Initialize the sensors
  SensorReadings readings = initializeSensors();
  if (!readings.success) {
      Serial.println("Failed to initialize sensors. Exiting...");
      return 0;  // or handle the failure appropriately
  }
  // Determine starting position A or B
  bool startingPositionA = false;
  int startDistance = 0 targetDistance = 0;

  // Based on start position, calculate distance to opposite start area
  if (abs(position[0] - AREA_A_CENTER_X) < START_RADIUS_CM && abs(position[1] - AREA_A_CENTER_Y) < START_RADIUS_CM) {
    startingPositionA = true;
    startDistance = calculateDistance(position, {AREA_A_CENTER_X, AREA_A_CENTER_Y, 0});
    targetDistance = calculateDistance(position, {AREA_B_CENTER_X, AREA_B_CENTER_Y, 0});
    Serial.print("Starting Position A, distance from center: ");
    Serial.println(startDistance);
  } else if (abs(position[0] - AREA_B_CENTER_X) < START_RADIUS_CM && abs(position[1] - AREA_B_CENTER_Y) < START_RADIUS_CM) {
    startingPositionA = false;
    startDistance = calculateDistance(position, {AREA_B_CENTER_X, AREA_B_CENTER_Y, 0});
    targetDistance = calculateDistance(position, {AREA_A_CENTER_X, AREA_A_CENTER_Y, 0});
    Serial.print("Starting Position B, distance from center: ");
    Serial.println(startDistance);
  } else {
    Serial.println("Error in starting position");
    return 0;
  }

  // Turn to face the opposite start area
  int currentAzimuth = position[2];
  if (startingPositionA) {
    // Turn to face Area B
    int targetAzimuth = calculateAzimuth(position, {AREA_B_CENTER_X, AREA_B_CENTER_Y, 0});
    adjustHeading(currentAzimuth, targetAzimuth);
  } else {
    // Turn to face Area A
    int targetAzimuth = calculateAzimuth(position, {AREA_A_CENTER_X, AREA_A_CENTER_Y, 0});
    adjustHeading(currentAzimuth, targetAzimuth);
  }

  Serial.print("Moving forward to opposite start area, distance: ");
  Serial.println(abs(targetDistance-START_RADIUS_CM));

  // Move forward set distance or until obstacle detected
  moveForwardSetDistance(SPEED, abs(targetDistance-START_RADIUS_CM));

  return 1
}

// Conduct width measurement and home in on pylon
int Navigation::pylonHoming(int position[]) {
  // Initialize the sensors
  SensorReadings readings = initializeSensors();
  if (!readings.success) {
      Serial.println("Failed to initialize sensors. Exiting...");
      return 0;  // or handle the failure appropriately
  }

  if (OBSTACLE_CLOSE_THRESHOLD >= readings.center) {
    movement.stop();
    Serial.println("stop");
    // We may want to reduce speed here
    delay(500);
    Serial.println("forward");
    movement.forward();
    if (2 >= readings.center) {
      movement.stop();
      Serial.println("stop");

      // This functionality should be brought into main.ino
      // int success = payload.deployPayload(); // Will need to create deployPayload() function

      if (success) {
        movement.reverse();
        Serial.println("Reverse");
      
        if (OBSTACLE_CLOSE_THRESHOLD >= readings.center) {
          movement.stop();
          Serial.println("stop");
          int currentAzimuth = position[2];
          int targetAzimuth = 90;
          // Adjust to face towards the end goal zone
          while (5 <= abs(azimuth - targetAzimuth)) {
            movement.left();
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
      Serial.println("Failed to initialize sensors. Exiting...");
      return 0;  // or handle the failure appropriately
  }

  int currentAzimuth = calculateAzimuth(currentPosition, targetPosition);
  // Current implementation is jerky, requiring stops and turns
  // We can upgrade this to flow smoothly by implementing PID or something similar
  if (OBSTACLE_CLOSE_THRESHOLD >= readings.center) {
    movement.stop();
    Serial.println("Stop, obstacle too close");
    delay(1000);
    if (readings.left > readings.right) {
      movement.left();
      Serial.println("Left");
      delay(500);
    } else {
      movement.right();
      Serial.println("Right");
      delay(500);
    }
  }

  int newAzimuth = calculateAzimuth(currentPosition, targetPosition);
  adjustHeading(currentAzimuth, newAzimuth);

  Serial.println("Forward");
  movement.forward();
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

SensorReadings Navigation::initializeSensors() {
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
        Serial.println("Error initializing sensors");
    }
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
      delay(200);
    } else {
      movement.right();
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
            Serial.println("Failed to initialize sensors. Exiting...");
            return 0;  // or handle the failure appropriately
        }
        if (readings.center <= OBSTACLE_CLOSE_THRESHOLD || readings.center >= SENSOR_MAX_THRESHOLD) {
            movement.stop();
            Serial.println("Obstacle detected or reached max threshold, stopping");
            return;
        }

        movement.forward(speed); // Assuming forward() can take speed as an argument
        delay(100); // Small delay to allow sensor reading update, adjust as necessary
    }

    movement.stop(); // Stop after traveling the set distance
    Serial.println("Reached target distance, stop");
}

int Navigation::scanAndCalculateWidth(int position[]) {
    // Start by taking a measurement directly in front
    SensorReadings readings = initializeSensors();
    if (!readings.success) {
        Serial.println("Failed to initialize sensors. Exiting...");
        return 0;  // or handle the failure appropriately
    }
    std::vector<SensorReadings> measurements = {readings};
    std::vector<int> azimuths = {position[2]};
    bool objectInView = false;

    int widthThreshold = readings.center > OBSTACLE_CLOSE_THRESHOLD / 2 ? readings.center / 3 : 2;

    if (readings.center <= OBSTACLE_CLOSE_THRESHOLD && (abs(readings.left - readings.center) >= widthThreshold && abs(readings.right - readings.center) >= widthThreshold)) {
        double width = calculateObjectWidth(readings.left, readings.center, readings.right);

        if (width < PYLON_WIDTH * 2) {
            objectInView = true;
            Serial.print("Detected object width is within threshold: ");
            Serial.println(width);
        } else {
            objectInView = false; // Assuming you want to set this to false if the object is too wide
            Serial.print("Detected width too large: ");
            Serial.println(width);
            return 0;
        }
    } else {
        objectInView = false; // Ensure objectInView is set correctly if no close obstacle is detected
        Serial.println("No object detected in front");
        return 0;
    }

    // Scan to the left
    movement.turn(SCAN_ANGLE/2);
    double leftAzimuth = position[2] - SCAN_ANGLE/2;
    double objectStartAzimuth, objectEndAzimuth;

    for (double angle = leftAzimuth; angle <= leftAzimuth + SCAN_ANGLE; angle += SCAN_ANGLE / SCAN_SECTORS) {
      SensorReadings scanReading = initializeSensors();
      if (!scanReading.success) {
          Serial.println("Failed to initialize sensors. Exiting...");
          return 0;  // or handle the failure appropriately
      }
      measurements.push_back({scanReading});
      azimuth.push_back(angle);

      turn(SCAN_ANGLE / SCAN_SECTORS);  // Incremental scanning step
    }

    // Return to initial position
    turn(-SCAN_ANGLE/2);

    double leftEdgeAzimuth = 0;
    double rightEdgeAzimuth = 0;
    bool objectDetected = false;

    // Find the edges of the object based on the sensor readings
    for (size_t i = 0; i < measurements.size(); ++i) {
        const SensorReadings& reading = measurements[i];
        double currentAzimuth = azimuths[i];

        // Check for the start of the object
        if (reading.center <= OBSTACLE_CLOSE_THRESHOLD && !objectDetected) {
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
        Serial.print("Left edge of object detected at azimuth: ");
        Serial.println(leftEdgeAzimuth);
        Serial.print("Right edge of object detected at azimuth: ");
        Serial.println(rightEdgeAzimuth);

        // Calculate object width or other processing based on the detected edges
        // The width calculation could be refined based on the specific geometry and scanning logic
    } else {
        Serial.println("No object detected within the scanning range.");
        return 0;
    }
    // Calculate the width based on the collected data
    int objectWidth = (int)calculateWidthFromMeasurements(measurements);

    return objectWidth;
}

double Navigation::calculateWidthFromMeasurements(const std::vector<SensorReadings>& measurements) {
    // Logic to calculate the object's width based on multiple sensor readings

    double widthSum = 0.0;
    for (const auto& reading : measurements) {
        double width = calculateObjectWidth(reading.left, reading.center, reading.right);
        widthSum += width;
    }
    double averageWidth = widthSum / measurements.size();
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