#include "Payload.h"
#include "Arduino.h"

#include "../ParamsCustom/params.h"
#include "../ParamsCustom/pinLayout.h"

// Implement code here
Payload::Payload()
{
    pinMode(C1, INPUT);
    pinMode(R1, INPUT);
    pinMode(PAYLOAD, OUTPUT);
    digitalWrite(PAYLOAD, LOW);
}

int Payload::deployPayload(void)
{
    digitalWrite(PAYLOAD, HIGH);
    delay(1000);
    digitalWrite(PAYLOAD, LOW);

    return 1;
}

void Payload::collectPayload(void)
{
    digitalWrite(PAYLOAD, HIGH);
    delay(1000);
    digitalWrite(PAYLOAD, LOW);
}

int Payload::getCycle()
{
    int tenCycleAvg = 0;
    for(int i = 0; i < 10; ++i)
        tenCycleAvg += readOneCycle(2000) // temp cycle time;

    tenCycleAvg /= 10;
    return return tenCycleAvg;
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
