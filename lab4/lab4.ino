#include <Pololu3piPlus32U4.h>

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

const int BASE_SPEED = 100;
const int SPEED_RANGE = 30;

const unsigned int NUM_GOALS = 4;
const float GOALS[NUM_GOALS][2] = { 
    { 80.0f, 50.0f },
    { 60.0f, 0.0f },
    { -30.0f, 0.0f },
    { 0.0f, 0.0f }
};
unsigned int goal = 0;
bool atGoal = false;

const float K_P = 2.0f;

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
    } else if (goal >= NUM_GOALS) {
        buzzer.play("g32");
        delay(1000);
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
    theta = normalizeAngle(theta + deltaTheta);
}

void adjustMotors() {
    float targetTheta = atan2(GOALS[goal][1] - y, GOALS[goal][0] - x);

    if (theta > targetTheta && theta / targetTheta < 0) {
        targetTheta += 2 * M_PI;
    } else if (theta < targetTheta && theta / targetTheta < 0) {
        targetTheta -= 2 * M_PI;
    }

    float eTheta = normalizeAngle(theta - targetTheta);
    float distanceToGoal =
        sqrt(pow(GOALS[goal][0] - x, 2) + pow(GOALS[goal][1] - y, 2));

    int motorSpeed;
    if (distanceToGoal > 0.25) {
        // Slow down the robot as it approaches the target
        if (distanceToGoal < 10) {
            motorSpeed *= distanceToGoal / 5;

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
        int leftSpeed = K_P * eTheta * SPEED_RANGE + motorSpeed;
        int rightSpeed = motorSpeed - (leftSpeed - motorSpeed);
        motors.setSpeeds(leftSpeed, rightSpeed);
    } else {
        motors.setSpeeds(0, 0);
    }
}

void checkGoal() {
    if (atGoal) {
        goal++;
        if (goal == NUM_GOALS) {
            buzzer.play("g32");
        } else {
            buzzer.play("c32");
        }
        delay(1000);
        atGoal = false;
    }
}

float normalizeAngle(float angle) {
    if (angle < -1 * M_PI) {
        angle += 2 * M_PI;
    } else if (angle > M_PI) {
        angle -= 2 * M_PI;
    }

    return angle;
}
