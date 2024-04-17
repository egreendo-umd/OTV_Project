#include "Payload.h"
#include "Arduino.h"



// Implement code here
Payload::Payload(Movement& movement)
{
    pinMode(C1, INPUT);
    pinMode(R1, INPUT);
    pinMode(PAYLOAD, OUTPUT);
    digitalWrite(PAYLOAD, LOW);
    this->movement = &movement;
}
void Payload::shuffle(void)
{
    while(readOneCycle(2000)==0)
    {
        movement->left();
        movement->move(100);
        delay(10);
        movement->stop();
        movement->right();
        movement->move(100);
        delay(10);
        movement->stop();
    }
}
int Payload::deployPayload(void)
{
    int map_angle;
    map(map_angle,0,270,0,180);
    for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    movement->reverse();
    delay(10);
  }
  return 1;

  /*digitalWrite(PAYLOAD, HIGH);
    delay(1000);
    digitalWrite(PAYLOAD, LOW);

    return 1; */  
  }

void Payload::collectPayload(void)
{
    /*digitalWrite(PAYLOAD, HIGH);
    delay(1000);
    digitalWrite(PAYLOAD, LOW);
    */
    for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

int Payload::getCycle()
{
    int tenCycleAvg = 0;
    for(int i = 0; i < 10; ++i)
        tenCycleAvg += readOneCycle(2000); // temp cycle time;

    tenCycleAvg /= 10;
    return tenCycleAvg;
}

int Payload::readOneCycle(int mils)
{
    unsigned int highs = 0;

    for(int i = 0; i < mils; ++i)
    {
        highs += digitalRead(C1);
    }

    Serial.println("Highs");
    Serial.println(highs);
    return 100 * float(highs)/(float)mils;
}
