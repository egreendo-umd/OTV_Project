#ifndef GITHUB_RUNNER
// If running locally, use the following includes for src libraries
#include "src/ParamsCustom/params.h"
#include "src/ParamsCustom/pinLayout.h"
#include "src/VisionSystemCustom/VisionSystemClient.hpp"
#else
// If running in Github Actions, use the following includes
#include <params.h>
#include <pinLayout.h>
#include <VisionSystemClient.hpp>
#endif



VisionSystemClient Enes100; // Initialize Enes100 Vision System

void setup() {
    Serial.begin(9600); // Start communications
    // Initialize Enes100 Library
    Enes100.begin("B-Team", DATA, 697, 9, 8);
    delay(1000); // Wait for Enes100 to initialize (1 second delay)
    Serial.println("Vision System initialized");
}

void loop() {
    int isVisible;
    int x, y;

   // Get position from Enes100 Vision System
    x = Enes100.getX();
    y = Enes100.getY();
    theta = Enes100.getTheta();
    isVisible = Enes100.isVisible();

    if (!isVisible) {
        Serial.println("Marker not visible");
        return;
    }

    Serial.print("Current position: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" Theta=");
    Serial.println(theta);

}