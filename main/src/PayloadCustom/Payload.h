#ifndef Payload_h
#define Payload_h

#include "../ParamsCustom/params.h"
#include "../ParamsCustom/pinLayout.h"
#include "../MovementCustom/Movement.h"
#include <Servo.h>

// Implement header code here
class Payload {
    public:
        Payload(Movement& movement); // Declare the constructor
        int deployPayload(void);
        void collectPayload(void);
        int getCycle();
        void shuffle();
        int readOneCycle(int mils);
    private:
        Movement* movement;
        Servo myservo;
};

#endif