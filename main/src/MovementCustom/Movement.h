
#ifndef Movement_h
#define Movement_h

#include "../VisionSystemCustom/VisionSystemClient.hpp"

class Movement{
public:
    Movement(VisionSystemClient& Enes100); // Declare the constructor

    void move(int setSpeed);
    void forward();
    void reverse();
    void left();
    void right();
    void turn(int angle); //left = 0, right = 1
    void stop();

private:
    VisionSystemClient* Enes100;
    bool in1, in2, in3, in4;
};

#endif
