
#ifndef Movement_h
#define Movement_h

#include "IMovement.h"
#include "src/VisionSystemCustom/VisionSystemClient.hpp"

class Movement : public IMovement{
public:
    Movement(VisionSystemClient& Enes100); // Declare the constructor

    void move(int setSpeed) override;
    void forward() override;
    void reverse() override;
    void left() override;
    void right() override;
    void turn(int angle, int overrideAngle = -1); //left < 0 < right
    void stop() override;
private:
    VisionSystemClient* Enes100;
    bool in1, in2, in3, in4;
};

#endif
