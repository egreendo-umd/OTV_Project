#ifndef GITHUB_RUNNER
// If running locally, use the following includes for src libraries
#include "src/ParamsCustom/params.h"
//#include "src/ParamsCustom/pinLayout.h"
//#include "src/MovementCustom/Movement.h"
#include "src/VisionSystemCustom/VisionSystemClient.hpp"
//#include "src/NavigationCustom/Navigation.h"
#else
// If running in Github Actions, use the following includes
#include <params.h>
#include <pinLayout.h>
#include <Movement.h>
#endif



VisionSystemClient Enes100;
//Movement movement(Enes100); // Initialize Movement Pins and Variables
//Navigation navigation(movement, Enes100);
int mode;
void setup() {
    //Serial.begin(9600); // Start communications
    mode = MODE_PYLON;
}

void loop() {
    switch(mode)
    {
    case MODE_PYLON:
        break;

    default:
        exit(0);
    };
    

    // Test the movement functions
    // Serial.println("Testing forward motion");
    // movement.forward();
    // movement.move(SPEED);
    // delay(5000);

    // movement.stop();
    // Serial.println("Testing reverse motion");
    // movement.reverse();
    // movement.move(SPEED);
    // delay(5000);

    // movement.stop();
    // Serial.println("Testing left motion");
    // movement.left();
    // movement.move(SPEED);
    // delay(5000);
    // movement.stop();
    // delay(1000);

    // Serial.println("Testing right motion");
    // movement.right();
    // movement.move(SPEED);
    // delay(5000);
    // movement.stop();
    // delay(1000);

    // Serial.println("Testing angle turn motion");
    // Serial.println("Turning left 90 degrees");
    // movement.turn(-90);
    // delay(5000);
    // Serial.println("Turning right 180 degrees");
    // movement.turn(180);
    // delay(5000);

}