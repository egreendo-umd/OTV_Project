#include "src/ENES100ArduinoLibrary-master/src/Enes100.h"
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

Navigation nav;

void setup() {
    Enes100.begin("B-Team", DATA, int markerId, int wifiModuleTX, int wifiModuleRX) //TODO: Update with unkown values

    //Serial.begin(9600); // Start communications

    nav.initNav(); // Initialize Navigation Pins and Variables

}

void loop() {
    int position[3];
    int foundPylon;

    // Wifi Module OVS Position Query
    // position = OVS.queryOVS(); // Uncomment to use OVS
    position[0] = 200; // Filler x position to force Obstacle Avoidance {x (cm, y (cm), az (degrees))}
    position[1] = 200, 200; // Filler y position to force Obstacle Avoidance {x (cm, y (cm), az (degrees))}
    position[2] = 90; // Filler az position to force Obstacle Avoidance {x (cm, y (cm), az (degrees))}

    Serial.print("Current position: ");
    Serial.println(position);

    // Determine if finding Payload or Avoiding Obstacles
    if (position[0] < MODE_ONE_X && position[1] < MODE_ONE_Y) 
    {
        foundPylon = nav.pylonSearch(position); // Object width measurement
        if (foundPylon == 1) {
            readyPayload = nav.pylonHoming(position); // Specific navigation to pylon and orientation for data extraction deployment
        }
    } 
    else if ((position[0] > MODE_ONE_X && position[1] > MODE_ONE_Y) &&
                (position[0] < MODE_TWO_X && position[1] < MODE_TWO_Y)) 
    { 
        nav.obstacleAvoidance(position);
    } else {
        Serial.print("There was an error in the position: ");
        Serial.println(position);
    }
}