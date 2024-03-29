
#ifndef Movement_h
#define Movement_h
#include "Arduino.h"
#include "../ParamsCustom/pinLayout.h"

class Movement {
public:
    Movement(); // Declare the constructor
    void move(int setSpeed);
    void forward();
    void reverse();
    void left();
    void right();
    void turn90(bool direction); //left = 0, right = 1
    void stop();

private:
    bool in1, in2, in3, in4;
};

#endif
