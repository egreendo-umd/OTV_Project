#include "OVS.h"
#include "Arduino.h"
#include "../params.h"
#include "../pinLayout.h"

void initOVS() {
    delay(2000);
    // Assuming that the "9, 8" are pins, we'll need to edit the pinLayout
    // then use those variable names for this function
    OVS.begin("B-Team", DATA, 697, 9, 8);
    delay(2000);
    //rx=8 tx=9
}
void loop(){

    // Where is this object getting created that's used below? Is the struct in the Enes100.begin cpp code?
    OVS.updateLocation();
    // xlocation=Enes100.getX();
    //  ylocation=Enes100.getY();
    OVS.print("x= " );
    OVS.println(OVS.location.x);
    OVS.print("y= " );
    OVS.println(OVS.location.y);
delay(100);

}