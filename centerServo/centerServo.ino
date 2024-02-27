#include <Servo.h>

Servo myServo;

void setup() {
    myServo.attach(22);
    myServo.write(90);
}

void loop() {}
