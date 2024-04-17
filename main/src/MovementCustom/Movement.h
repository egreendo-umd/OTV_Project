
#ifndef Movement_h
#define Movement_h

#include "../VisionSystemCustom/VisionSystemClient.hpp"
#include "Movement.h"
#include "../ParamsCustom/pinLayout.h"
#include "../ParamsCustom/params.h"

class Movement{
public:
    Movement(VisionSystemClient& Enes100); // Declare the constructor
    void move(int setSpeed);
    void forward();
    void reverse();
    void left();
    void right();
    void turn(int angle, int overrideAngle = -1); //left < 0 < right
    void stop();
private:
    VisionSystemClient* Enes100;
    bool in1, in2, in3, in4;
};

#endif
