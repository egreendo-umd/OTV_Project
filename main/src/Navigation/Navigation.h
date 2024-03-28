#ifndef Navigation_h
#define Navigation_h
#include "Arduino.h"

#include "../params.h"
#include "../pinLayout.h"

class Navigation {
    public:
        void initNav(void);
        int pylonSearch(int[]);
        int pylonHoming(int[]);
        int obstacleAvoidance(int[]);
        int parseOVS();
    private:
        int sensorOne();
        int sensorTwo();
        int sensorThree();
}

#endif