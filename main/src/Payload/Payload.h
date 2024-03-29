#ifndef Payload_h
#define Payload_h
#include "Arduino.h"

#include "../params.h"
#include "../pinLayout.h"

// Implement header code here
class Payload {
    public:
        void initPayload(void);
        int deployPayload(void);
        void collectPayload(void);
    private:
};

#endif