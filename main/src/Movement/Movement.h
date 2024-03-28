
#ifndef Movement_h
#define Movement_h
#include "Arduino.h"

#define N1 6 // MAKE SURE PLEASE MAKE SURE THE PINS ARE GOING TO THE CORRECT MOTORS
#define N2 3
#define N3 5
#define N4 11
#define ENA 1 //temp
#define ENB 2 //temp

class Movement
{
public:
    void move(int setSpeed);
    void forward();
    void reverse();
    void left();
    void right();
    void turn90(bool direction); //left = 0, right = 1
    void stop();


private:
    int in1, in2, in3, in4;
}
#endif
