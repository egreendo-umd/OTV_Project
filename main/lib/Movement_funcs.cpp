#include "Movement_funcs.h"
#include "pinLayout.h"

Movement_funcs::Movement_funcs()
{
    pinMode(N1, OUTPUT);
    pinMode(N2, OUTPUT);
    pinMode(N3, OUTPUT);
    pinMode(N4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    stop();
}

void Movement_funcs::move(int setSpeed)
{
    analogWrite(ENA, setSpeed);
    analogWrite(ENB, setSpeed);
    digitalWrite(N1, in1);
    digitalWrite(N2, in2);
    digitalWrite(N3, in3);
    digitalWrite(N4, in4);
}

void Movement_funcs::forward()
{
    in1 = HIGH;
    in2 = LOW;
    in3 = HIGH;
    in4 = LOW;
}

void Movement_funcs::reverse()
{
    in1 = LOW;
    in2 = HIGH;
    in3 = LOW;
    in4 = HIGH;
}

void Movement_funcs::left()
{
    in1 = LOW;
    in2 = HIGH;
    in3 = HIGH;
    in4 = LOW;
}

void Movement_funcs::turn90(bool direction)
{
    direction? right(): left();
    move();
    delay(1000) //placeholder value;
    stop();
}

void right()
{
    in1 = HIGH;
    in2 = LOW;
    in3 = LOW;
    in4 = HIGH;
}

void stop()
{
    digitalWrite(N1, LOW);
    digitalWrite(N2, LOW);
    digitalWrite(N3, LOW);
    digitalWrite(N4, LOW);
}