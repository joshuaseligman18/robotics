#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

// Pins for the ultrasonic sensor
const int ECHO_PIN = 4;
const int TRIG_PIN = 5;

// Max distance for the sensor
const int MAX_DISTANCE = 50;

// Normalization factor based on MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE / 100.0f;
const float STOP_DISTANCE = 5.0f;

// Motor constants
const float MOTOR_BASE_SPEED = 100.0f;
const float MOTOR_MIN_SPEED = 25.0f;
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100.0f;

// Time keeping
unsigned long usCm;
unsigned long usPm;
const unsigned long US_PERIOD = 50;

unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 20;

// Motor compensation
const float L_MOTOR_FACTOR = 1.0;
const float L_MOTOR_FACTOR_THRESHOLD = 80;
const float R_MOTOR_FACTOR = 1.0;
const float R_MOTOR_FACTOR_THRESHOLD = 80;

// Distance to the object in cm
float distance = 0;

Buzzer buzzer;
Motors motors;

void setup() {
    Serial.begin(57600);

    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    delay(1000);
    buzzer.play("c32");
}

void loop() {
    usReadCm();
    setMotors();
}

void usReadCm() {
    usCm = millis();
    if (usCm >= usPm + US_PERIOD) {
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

        usPm = usCm;
    }
}

void setMotors() {
    motorCm = millis();
    if (motorCm > motorPm + MOTOR_PERIOD) {
        float leftSpeed = MOTOR_BASE_SPEED;
        float rightSpeed = MOTOR_BASE_SPEED;

        if (distance <= MAX_DISTANCE) {
            // How close we are to the max distance in regard on
            // a scale of 0 (far away) to 100 (distance is 0)
            float magnitude = (MAX_DISTANCE - distance) / DISTANCE_FACTOR;
            leftSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
            rightSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);

            if (leftSpeed < MOTOR_MIN_SPEED) {
                leftSpeed = MOTOR_MIN_SPEED;
            }
            if (rightSpeed < MOTOR_MIN_SPEED) {
                rightSpeed = MOTOR_MIN_SPEED;
            }

            if (distance <= STOP_DISTANCE) {
                leftSpeed = 0;
                rightSpeed = 0;
            }
        }

        // Compensation for the motors in case they are inconsistent
        if (leftSpeed < L_MOTOR_FACTOR_THRESHOLD) {
            leftSpeed *= L_MOTOR_FACTOR;
        }
        if (rightSpeed < R_MOTOR_FACTOR_THRESHOLD) {
            rightSpeed *= R_MOTOR_FACTOR;
        }

        // Left is right and right is left
        // Also the robot is moving backwards
        motors.setSpeeds(-rightSpeed, -leftSpeed);
        motorPm = motorCm;
    }
}
