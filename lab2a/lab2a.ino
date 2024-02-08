#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

// Pins for the ultrasonic sensor
const int ECHO_PIN = 22;
const int TRIG_PIN = 4;

// Max distance for the sensor
const int MAX_DISTANCE = 400;

// Time keeping
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long US_PERIOD = 100;

// Distance to the object in cm
float distance = 0;

Buzzer buzzer;

void setup() {
    Serial.begin(57600);

    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    delay(1000);
    buzzer.play("c32");
}

void loop() {
    usReadCm();
}

void usReadCm() {
    currentMillis = millis();
    if (currentMillis >= prevMillis + US_PERIOD) {
        // Make sure the trigger starts off as low
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);

        // Need to supply voltage to the trigger for 10 microseconds
        // for the sensor to send out the signal
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        // Echo pin is set to high when it receives a signal back
        // Wait for the echo to be received or until the timeout
        // of 38000 microseconds
        long duration = pulseIn(ECHO_PIN, HIGH, 38000);

        // Convert the duration to a distance in cm
        distance = duration / 58.0f;

        // Clean up the distance data
        if (distance > MAX_DISTANCE) {
            distance = MAX_DISTANCE;
        } else if (distance == 0) {
            distance = MAX_DISTANCE;
        }

        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");

        prevMillis = currentMillis;
    }
}
