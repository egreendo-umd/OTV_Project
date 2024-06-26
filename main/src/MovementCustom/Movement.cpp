#include "Arduino.h"
#include "Movement.h"
#include "../ParamsCustom/pinLayout.h"
#include "../ParamsCustom/params.h"

Movement::Movement()
{
    pinMode(N1, OUTPUT);
    pinMode(N2, OUTPUT);
    pinMode(N3, OUTPUT);
    pinMode(N4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    this->stop();
}

void Movement::move(int setSpeed)
{
    analogWrite(ENA, setSpeed);
    analogWrite(ENB, setSpeed);
    digitalWrite(N1, in1);
    digitalWrite(N2, in2);
    digitalWrite(N3, in3);
    digitalWrite(N4, in4);
}

void Movement::forward() {
    in1 = HIGH;
    in2 = LOW;
    in3 = HIGH;
    in4 = LOW;
    //move(255); // Assuming a default speed for forward
}

void Movement::reverse() {
    in1 = LOW;
    in2 = HIGH;
    in3 = LOW;
    in4 = HIGH;
    //move(255); // Assuming a default speed for reverse
}

void Movement::left() {
    in1 = LOW;
    in2 = HIGH;
    in3 = HIGH;
    in4 = LOW;
    //move(255); // Assuming a default speed for left turn
}

void Movement::right() {
    in1 = HIGH;
    in2 = LOW;
    in3 = LOW;
    in4 = HIGH;
    //move(255); // Assuming a default speed for right turn
}

void Movement::turn(int angle) {
    int setSpeed = SPEED; // Maximum speed for turning
    int turnDuration = map(abs(angle), 0, 360, 0, SPIN_TIME); // Map angle to time (adjust 2000 as needed)

    if (angle > 0) {
        right();
    } else {
        left();
    }

    move(setSpeed);
    delay(turnDuration); // turn for a time proportional to the angle
    stop();
}

void Movement::stop() {
    digitalWrite(N1, LOW);
    digitalWrite(N2, LOW);
    digitalWrite(N3, LOW);
    digitalWrite(N4, LOW);
}