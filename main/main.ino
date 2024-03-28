#include "src/params.h"
#include "src/pinLayout.h"

#include "src/Navigation/Navigation.h"
#include "src/OVS/OVS.h"
#include "src/Movement/Movement.h"
#include "src/Payload/Payload.h"

// Filler values for determining mode switch boundaries from OVS query
#define MODE_ONE_X 100 // centimeters
#define MODE_ONE_Y 100 // centimeters
#define MODE_TWO_X 500 // centimeters
#define MODE_TWO_Y 500 // centimeters

void setup() {
    Serial.begin(9600); // Start communications

    Navigation.initNav(); // Initialize Navigation Pins and Variables

}

void loop() {
    int[3] position;
    int foundPylon;

    // Wifi Module OVS Position Query
    // position = OVS.queryOVS(); // Uncomment to use OVS
    position = {200, 200, 90}; // Filler position to force Obstacle Avoidance {x (cm, y (cm), az (degrees))}
    Serial.print("Current position: ");
    Serial.print(position);
    Serial.print("\n");

    // Determine if finding Payload or Avoiding Obstacles
    if (position[0] < MODE_ONE_X && position[1] < MODE_ONE_Y) 
    {
        foundPylon = Navigation.pylonSearch(position); // Object width measurement
        if (foundPylon == 1) {
            readyPayload = Navigation.pylonHoming(position); // Specific navigation to pylon and orientation for data extraction deployment
        }
    } 
    else if ((position[0] > MODE_ONE_X && position[1] > MODE_ONE_Y) &&
                (position[0] < MODE_TWO_X && position[1] < MODE_TWO_Y)) 
    { 
        Navigation.obstacleAvoidance(position);
    } else {
        Serial.print("There was an error in the position: ");
        Serial.print(position);
        Serial.print("\n");
    }
}