#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
Encoders encoders;

unsigned long prevMillis = 0;
const unsigned long PERIOD = 10;

float theta = M_PI_2;
float x = 0.0f;
float y = 0.0f;

const int COUNTS_PER_REVOLUTION = 12;
const float GEAR_RATIO = 75.81f;
// Total encoder clicks per revolution
const float CLICKS_PER_REVOLUTION = COUNTS_PER_REVOLUTION * GEAR_RATIO;
// Both measured in cm
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

const float DISTANCE_BETWEEN_WHEELS = 8.5f;

const int BASE_SPEED = 75;
const int SPEED_RANGE = 30;

const unsigned int NUM_GOALS = 4;
const float GOALS[NUM_GOALS][2] = { 
    { 38.48f, 55.88f },
    { -72.39f, 24.13f },
    {  68.58f, -50.8f },
    {  0.0f, 0.0f }
};
unsigned int goal = 0;
bool atGoal = false;

void setup() {
    Serial.begin(57600);
    delay(1000);
    buzzer.play("c32");
}

void loop() {
    unsigned long curMillis = millis();
    if (curMillis >= prevMillis + PERIOD && goal < NUM_GOALS) {
        updatePosition();
        adjustMotors();
        checkGoal();
        prevMillis = curMillis;
    }
}

void updatePosition() {
    long countsLeft = encoders.getCountsAndResetLeft();
    long countsRight = encoders.getCountsAndResetRight();
    float deltaLeft = countsLeft * WHEEL_CIRCUMFERENCE / CLICKS_PER_REVOLUTION;
    float deltaRight = countsRight * WHEEL_CIRCUMFERENCE / CLICKS_PER_REVOLUTION;

    float deltaDistance = (deltaRight + deltaLeft) / 2;
    float deltaTheta = (deltaRight - deltaLeft) / DISTANCE_BETWEEN_WHEELS;

    float deltaX = deltaDistance * cos(theta + deltaTheta / 2);
    float deltaY = deltaDistance * sin(theta + deltaTheta / 2);

    x += deltaX;
    y += deltaY;
    theta += deltaTheta;
    if (theta > M_PI) {
        theta -= 2 * M_PI;
    } else if (theta < -1 * M_PI) {
        theta += 2 * M_PI;
    }
}

void adjustMotors() {
    float targetTheta = atan2(GOALS[goal][1] - y, GOALS[goal][0] - x);

    if (theta > 0 && targetTheta < 0) {
        targetTheta += 2 * M_PI;
    } else if (theta < 0 && targetTheta > 0) {
        targetTheta -= 2 * M_PI;
    }

    float eTheta = theta - targetTheta;
    float distanceToGoal = computeDistanceToGoal();

    int motorSpeed;
    if (distanceToGoal > 0.1) {
        // Slow down the robot as it approaches the target
        if (distanceToGoal < 10) {
            motorSpeed *= distanceToGoal / 10;

            // Keep a minimum speed for the robot so it doesn't stall
            if (motorSpeed < 30) {
                motorSpeed = 30;
            }
        } else {
            motorSpeed = BASE_SPEED;
        }
    } else {
        motorSpeed = 0;
        atGoal = true;
    }

    if (motorSpeed != 0) {
        int leftSpeed = eTheta * SPEED_RANGE + motorSpeed;
        int rightSpeed = motorSpeed - (leftSpeed - motorSpeed);
        motors.setSpeeds(leftSpeed, rightSpeed);
    } else {
        motors.setSpeeds(0, 0);
    }
}

float computeDistanceToGoal() {
    return sqrt(pow(GOALS[goal][0] - x, 2) + pow(GOALS[goal][1] - y, 2));
}

void checkGoal() {
    if (atGoal) {
        Serial.print("X: ");
        Serial.print(x);
        Serial.print("; Y: ");
        Serial.println(y);

        buzzer.play("c32");
        delay(1000);
        goal++;
        atGoal = false;
    }
}
