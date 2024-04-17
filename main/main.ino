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

enum {MODE_PYLON, MODE_OBSTACLE, MODE_ENDZONE};
int mode;
float x, y, heading;
void setup() {
    // Serial.begin(9600);
    // Initialize Enes100 Library
    Enes100.begin("B-Team", DATA, 697, TX, RX);
    delay(2000); // Wait for Enes100 to initialize (1 second delay)
    x = Enes100.getX();
    y = Enes100.getY();
    heading = 180* (Enes100.getTheta()/M_PI);
    mode = MODE_PYLON;
}

void loop() {
    if (!Enes100.isVisible()) 
        return;

    // Get position from Enes100 Vision System
    int position[3] = {static_cast<int>(x * 100), static_cast<int>(y * 100), static_cast<int>(heading)}; // Convert meters to centimeters, assuming az is 0 for simplicity
    switch(mode)
    {
    case MODE_PYLON:
        while(!nav.pylonSearch(position));
        break;
    default:
        exit(0);
    }
    
    

    

}