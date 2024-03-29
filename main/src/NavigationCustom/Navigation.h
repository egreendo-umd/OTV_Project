#ifndef Navigation_h
#define Navigation_h

#include "../MovementCustom/IMovement.h"

class Navigation {
    public:
        Navigation(IMovement& movementController);
        int pylonSearch(int position[]);
        int pylonHoming(int position[]);
        int obstacleAvoidance(int position[]);

        int sensorOne();
        int sensorTwo();
        int sensorThree();
    private:
        IMovement& movement;
        bool mode1;
        bool mode2;
};

#endif