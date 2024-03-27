#include <Enes100.h>
void setup() {
  delay(2000);
 Enes100.begin("B-Team", DATA, 697, 9, 8);
 delay(2000);
 //rx=8 tx=9
}
void loop(){
  Enes100.updateLocation();
// xlocation=Enes100.getX();
//  ylocation=Enes100.getY();
 Enes100.print("x= " );
 Enes100.println(Enes100.location.x);
 Enes100.print("y= " );
 Enes100.println(Enes100.location.y);
delay(100);

}