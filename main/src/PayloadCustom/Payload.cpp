#include "Payload.h"
#include "Arduino.h"

#include "../params.h"
#include "../pinLayout.h"

// Implement code here
void Payload::initPayload(void)
{
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