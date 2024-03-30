
#ifndef Movement_h
#define Movement_h

#include "IMovement.h"

class Movement : public IMovement{
public:
    Movement(); // Declare the constructor

    void move(int setSpeed) override;
    void forward() override;
    void reverse() override;
    void left() override;
    void right() override;
    void turn(int angle) override; //left = 0, right = 1
    void stop() override;

private:
    bool in1, in2, in3, in4;
};

#endif
