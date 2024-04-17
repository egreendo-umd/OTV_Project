#ifndef GITHUB_RUNNER
// If running locally, use the following includes for src libraries
#include "src/ParamsCustom/params.h"
#include "src/ParamsCustom/pinLayout.h"
#include "src/NavigationCustom/Navigation.h"
#include "src/VisionSystemCustom/VisionSystemClient.hpp"
#include "src/MovementCustom/Movement.h"
#include "src/PayloadCustom/Payload.h"
#else
// If running in Github Actions, use the following includes
#include <params.h>
#include <pinLayout.h>
#include <Navigation.h>
#include <VisionSystemClient.hpp>
#include <Movement.h>
#include <Payload.h>
#endif





Payload payload; // Initialize Payload Pins and Variables
VisionSystemClient Enes100; // Initialize Enes100 Vision System
Movement movement(Enes100); // Initialize Movement Pins and Variables
Navigation nav(movement, Enes100); // Initialize Navigation Pins and Variables

void setup() {
    // Serial.begin(9600);
    // Initialize Enes100 Library
    Enes100.begin("B-Team", DATA, 697, TX, RX);
    Enes100.println("Beginning...");

    delay(1000); // Wait for Enes100 to initialize (1 second delay)
    Enes100.println("Vision System initialized");
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
        if (foundPylon == 0) {
            foundPylon = nav.pylonSearch(position); // Object width measurement

        }
        if (foundPylon == 1) {
            readyPayload = nav.pylonHoming(position); // Specific navigation to pylon and orientation for data extraction deployment
        }
    } 
    else if ((position[0] > MODE_ONE_X && position[1] > MODE_ONE_Y) &&
                (position[0] < MODE_TWO_X && position[1] < MODE_TWO_Y)) 
    { 
        int goalzone[3] = {GOALZONE_X, GOALZONE_Y, 0};
        nav.obstacleAvoidance(position, goalzone); // Avoid obstacles to navigate to target position
    } else {
        Serial.print("There was an error in the position: ");
        for (int i = 0; i < 3; i++) {
            Serial.print("position[");
            Serial.print(i);
            Serial.print("]: ");
            Serial.println(position[i]);
        }
        delay(1000);
        return;
    }
}