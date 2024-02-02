#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    buzzer.play("c32");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);

    buzzer.play("c12");
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}
