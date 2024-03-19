#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
Servo headServo;

// Pins
const int HEAD_SERVO_PIN = 22;
const int ECHO_PIN = 4;
const int TRIG_PIN = 5;

// US variables
const float MAX_DISTANCE = 300.0f;

const unsigned long PERIOD = 15;
unsigned long prevMillis = 0;

const float TARGET = 30.48f;

const bool P_ON = true;
const bool I_ON = true;
const bool D_ON = true;

const float K_P = 2.85f;
const float K_I = 0.05f;
const float K_D = 0.35f;

const int BASE_SPEED = 75;
const int MIN_SPEED = 50;
const int MAX_SPEED = 100;
const int HALF_SPEED_RANGE = (MAX_SPEED - MIN_SPEED) / 2;

const int PID_THRESHOLD = 5;
const float PID_MIN = -1 * PID_THRESHOLD * TARGET;
const float PID_MAX = PID_THRESHOLD * TARGET;
const float PID_RANGE = PID_MAX;

float prevError = 0.0f;
float errorSum = 0.0f;
const float INTEGRAL_THRESHOLD = 12.0f;

void setup() {
    Serial.begin(57600);

    // Set up servo pins
    headServo.attach(HEAD_SERVO_PIN);
    headServo.write(170);
    
    // Set up US pins
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);


    delay(1000);
    buzzer.play("c32");
}

void loop() {
    unsigned long curMillis = millis();
    if (curMillis >= prevMillis + PERIOD) {
        float distance = usReadCm();
        runPid(distance);
        prevMillis = curMillis;
    }
}

float usReadCm() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Send the signal
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Get the distance
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    float distance = duration / 58.0f;

    // Clean up the data
    if (distance > MAX_DISTANCE) {
        distance = MAX_DISTANCE;
    } else if (distance == 0) {
        distance = MAX_DISTANCE;
    }

    return distance;
}

void runPid(float curDistance) {
    float error = curDistance - TARGET;

    float pidCalculation = 0;

    if (P_ON) {
        pidCalculation += K_P * error;
    }

    if (I_ON) {
        errorSum += error;
        if (errorSum > INTEGRAL_THRESHOLD) {
            errorSum = INTEGRAL_THRESHOLD;
        } else if (errorSum < -1 * INTEGRAL_THRESHOLD) {
            errorSum = -1 * INTEGRAL_THRESHOLD;
        }

        pidCalculation += K_I * errorSum;
    }

    if (D_ON) {
        float dErr = error - prevError;
        pidCalculation += K_P * dErr;
        prevError = error;
    }

    if (pidCalculation < PID_MIN) {
        pidCalculation = PID_MIN;
    } else if (pidCalculation > PID_MAX) {
        pidCalculation = PID_MAX;
    }

    float normalizedPid = pidCalculation / PID_RANGE * HALF_SPEED_RANGE;

    int rightSpeed = (int)(BASE_SPEED + normalizedPid);
    if (rightSpeed < MIN_SPEED) {
        rightSpeed = MIN_SPEED;
    } else if (rightSpeed > MAX_SPEED) {
        rightSpeed = MAX_SPEED;
    }

    int rightMag = rightSpeed - BASE_SPEED;
    int leftSpeed = BASE_SPEED - rightMag;
    if (leftSpeed < MIN_SPEED) {
        leftSpeed = MIN_SPEED;
    } else if (leftSpeed > MAX_SPEED) {
        leftSpeed = MAX_SPEED;
    }
    
    motors.setSpeeds(-rightSpeed, -leftSpeed);
}
