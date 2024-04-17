#ifndef IMovement_h
#define IMovement_h

class IMovement {
public:
    virtual void move(int setSpeed);
    virtual void turn(int angle, int overrideAngle);
    virtual void stop();
    virtual void forward();
    virtual void reverse();
    virtual void left();
    virtual void right();
    virtual ~IMovement() {} // Include a virtual destructor
};

#endif
