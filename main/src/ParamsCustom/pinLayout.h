#ifndef pinLayout_h
#define pinLayout_h

// Navigation Pins
//Define the sensor pins
#define S1Trig A0
#define S2Trig A1
#define S3Trig A2
#define S1Echo A3
#define S2Echo A4
#define S3Echo A5

//movement funcs pins here
#define N1 A3
#define N2 A2
#define N3 A1
#define N4 8
#define ENA 6 
#define ENB 5 

//payload pins here
#define PAYLOAD 0
#define C1 12 //cycle positive side (likely to also be pin 7)
#define R1 7 //reed switch positive side
#endif