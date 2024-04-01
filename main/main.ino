#ifndef GITHUB_RUNNER
// If running locally, use the following includes for src libraries
#include "src/ParamsCustom/params.h"
#include "src/ParamsCustom/pinLayout.h"
#include "src/MovementCustom/Movement.h"
#else
// If running in Github Actions, use the following includes
#include <params.h>
#include <pinLayout.h>
#include <Movement.h>
#endif




Movement movement; // Initialize Movement Pins and Variables

void setup() {
    Serial.begin(9600); // Start communications
    Serial.println("Starting movement tests");
}

void loop() {
    // Test the movement functions
    Serial.println("Testing forward motion");
    movement.forward();
    movement.move(SPEED);
    delay(1000);

    Serial.println("Testing reverse motion");
    movement.reverse();
    movement.move(SPEED);
    delay(1000);

    Serial.println("Testing left motion");
    movement.left();
    movement.move(SPEED);
    movement.stop();
    delay(1000);

    Serial.println("Testing right motion");
    movement.right();
    movement.move(SPEED);
    movement.stop();
    delay(1000);

    Serial.println("Testing angle turn motion");
    Serial.println("Turning left 90 degrees");
    movement.turn(-90);
    delay(1000);
    Serial.println("Turning right 180 degrees");
    movement.turn(180);
    delay(1000);

}