#ifndef IMovement_h
#define IMovement_h

class IMovement {
public:
    virtual void stop() = 0;
    virtual void forward() = 0;
    virtual void reverse() = 0;
    virtual void left() = 0;
    virtual void right() = 0;
    virtual ~IMovement() {} // Include a virtual destructor
};

#endif
