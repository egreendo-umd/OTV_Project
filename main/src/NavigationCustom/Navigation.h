#ifndef Navigation_h
#define Navigation_h

class Navigation {
    public:
        Navigation(); // Declare the constructor
        int pylonSearch(int position[]);
        int pylonHoming(int position[]);
        int obstacleAvoidance(int position[]);
    private:
        int sensorOne();
        int sensorTwo();
        int sensorThree();
        bool mode1;
        bool mode2;
};

#endif