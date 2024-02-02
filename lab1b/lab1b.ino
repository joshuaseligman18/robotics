#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Motors motors;
Buzzer buzzer;

// Keeping track of the timing for checking the encoders
unsigned long currentMillis = 0;
unsigned long prevMillis = 0;
const unsigned long PERIOD = 20;

// Counts for the encoder measurements
long countsLeft = 0;
long countsRight = 0;
long prevCountsLeft = 0;
long prevCountsRight = 0;

// Keep track of the position of the robot based on distance
// travelled by each motor
float positionLeft = 0.0f;
float positionRight = 0.0f;

// Encoder resolution
const int COUNTS_PER_REVOLUTION = 12;
const float GEAR_RATIO = 75.81f;
// Total encoder clicks per revolution
const float CLICKS_PER_REVOLUTION = COUNTS_PER_REVOLUTION * GEAR_RATIO;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

void setup() {
    Serial.begin(57600);
    buzzer.play("c32");
    delay(1000);
}

void loop() {
    checkEncoders();
}

void checkEncoders() {
    currentMillis = millis();
    if (currentMillis > prevMillis + PERIOD) {
        countsLeft += encoders.getCountsAndResetLeft();
        countsRight += encoders.getCountsAndResetRight();

        positionLeft += (countsLeft - prevCountsLeft) * WHEEL_CIRCUMFERENCE / CLICKS_PER_REVOLUTION;
        positionRight += (countsRight - prevCountsRight) * WHEEL_CIRCUMFERENCE / CLICKS_PER_REVOLUTION;

        int wheelSpeed = 100;
        if (positionLeft < 30.48) {
            if (positionLeft > 20) {
                wheelSpeed *= (30 - positionLeft) / 10;
                if (wheelSpeed < 20) {
                    wheelSpeed = 20;
                }
            }
            motors.setSpeeds(wheelSpeed, wheelSpeed);
        } else {
            motors.setSpeeds(0, 0);
        }

        prevCountsLeft = countsLeft;
        prevCountsRight = countsRight;
        prevMillis = currentMillis;
    }
}
