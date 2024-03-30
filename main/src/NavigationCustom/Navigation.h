#ifndef Navigation_h
#define Navigation_h

#include "../MovementCustom/IMovement.h"

class Navigation {
    public:
        struct SensorReadings {
        int left;
        int center;
        int right;
        bool success;
        };
        struct ScanResult {
        double leftEdgeAzimuth;
        double rightEdgeAzimuth;
        double objectWidth;
        bool objectDetected;
        };
        Navigation(IMovement& movementController); // Constructor
        int pylonSearch(int position[]); // Determine starting position and navigate to pylon position
        int pylonHoming(int position[]); // Measure and home in on pylon
        int obstacleAvoidance(int currentPosition[3], int targetPosition[3]); // Avoid obstacles to navigate to target position
        int calculateAzimuth(int currentPosition[], int targetPosition[]);
        int calculateDistance(int currentPosition[], int targetPosition[]);
        int adjustHeading(int currentAzimuth, int targetAzimuth);
        void calculateTargetPosition(int currentPosition[], int currentAzimuth, int centerSensorReading, int targetPosition[]);
        ScanResult scanAndCalculateWidth(int position[]); // Get initial sensor readings and then scan left and right by moving OTV
        double calculateWidthFromMeasurements(const SensorReadings measurements[], int numMeasurements); // Calculate object width from scanned sensor readings
        double calculateObjectWidth(int leftSensor, int centerSensor, int rightSensor); // calculate indivual sensor reading width
        int sensorOne();
        int sensorTwo();
        int sensorThree();
        SensorReadings initializeSensors(); // Initialize the sensors
        void moveForwardSetDistance(double speed, double distance); // This should be in movement...
    private:
        IMovement& movement;
        bool mode1;
        bool mode2;
};

#endif