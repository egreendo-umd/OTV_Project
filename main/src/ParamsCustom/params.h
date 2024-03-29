#ifndef Params_h
#define Params_h

// File to store and manage parameters that may be used by multiple functions/files

// Include all necessary libraries
#include "../VisionSystemCustom/VisionSystemClient.hpp"
#include "../MovementCustom/Movement.h"
#include "../NavigationCustom/Navigation.h"

// Declare all necessary objects
VisionSystemClient Enes100;
Movement move;
Navigation nav;
Payload payload;

// Payload and Mission Parameters
#define PYLON_WIDTH 12;

// Propulsion and Movement Parameters
#define SPEED 160;

// Filler values for determining mode switch boundaries from OVS query
#define MODE_ONE_X 100 // centimeters
#define MODE_ONE_Y 100 // centimeters
#define MODE_TWO_X 500 // centimeters
#define MODE_TWO_Y 500 // centimeters

// From ENES100.h
#define CRASH_SITE        0
#define DATA              1
#define MATERIAL          2
#define FIRE              3
#define WATER             4
#define SEED              5


// Crash Mission
#define DIRECTION         0
#define LENGTH            1
#define HEIGHT            2
#define NORMAL_X          0
#define MORMAL_Y          1



// Data
#define CYCLE             0
#define MAGNETISM         1
#define MAGNETIC          0
#define NOT_MAGNETIC      1


// Materials
#define WEIGHT            0
#define MATERIAL_TYPE     1
#define FOAM              0
#define PLASTIC           1
#define HEAVY             0
#define MEDIUM            1
#define LIGHT             2


// Fire
#define NUM_CANDLES       0
#define TOPOGRAPHY        1
#define TOP_A             0
#define TOP_B             1
#define TOP_C             2


// Water
#define DEPTH             0
#define WATER_TYPE        1
#define FRESH_UNPOLLUTED  0
#define FRESH_POLLUTED    1
#define SALT_UNPOLLUTED   2
#define SALT_POLLUTED     3


// Seed
#define LOCATION 0
#define PERCENTAGE 1

#endif