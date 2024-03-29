#include "src/params.h"
#include "src/pinLayout.h"

#include "src/Navigation/Navigation.h"
#include "src/ENES100/src/ENES100.h"
#include "src/Movement/Movement.h"
#include "src/Payload/Payload.h"

Navigation nav;

void setup() {
    Serial.begin(9600); // Start communications

    nav.initNav(); // Initialize Navigation Pins and Variables
    // Initialize Enes100 Library
    Enes100.begin("B-Team", DATA, 205, 3, 2);
    Serial.println("ENES100 initialized");
}

void loop() {
    int foundPylon, isVisible, readyPayload;
    int x, y;

   // Get position from Enes100 Vision System
    x = Enes100.getX();
    y = Enes100.getY();
    isVisible = Enes100.isVisible();

    if (!isVisible) {
        Serial.println("Marker not visible");
        return;
    }

    Serial.print("Current position: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.println(y);

    int position[3] = {static_cast<int>(x * 100), static_cast<int>(y * 100), 0}; // Convert meters to centimeters, assuming az is 0 for simplicity


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
        for (int i = 0; i < 3; i++) {
            Serial.print("position[");
            Serial.print(i);
            Serial.print("]: ");
            Serial.println(position[i]);
        }
    }
}