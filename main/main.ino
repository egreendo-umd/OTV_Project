#include "params.h"
#include "pinLayout.h"

#include "lib/navigation.h"
#include "lib/ovs.h"
#include "lib/propulsion.h"
#include "lib/payload.h"


void setup() {
    Serial.begin(9600); // Start communications

    initNav();

}

void loop() {

}