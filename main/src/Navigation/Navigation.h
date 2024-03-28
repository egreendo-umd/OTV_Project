#ifndef Navigation_h
#define Navigation_h
#include "Arduino.h"

#include "../params.h"
#include "../pinLayout.h"

class Navigation {
    public:
        void initNav(void);
        int pylonSearch(int position[]);
        int pylonHoming(int position[]);
        int obstacleAvoidance(int position[]);
        int parseOVS();
    private:
        int sensorOne();
        int sensorTwo();
        int sensorThree();
        bool mode1;
        bool mode2;
};

#endif