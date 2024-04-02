#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
Encoders encoders;

unsigned long prevMillis = 0;
const unsigned long PERIOD = 10;

float theta = (float) M_PI_2;
float x = 0.0f;
float y = 0.0f;

const int COUNTS_PER_REVOLUTION = 12;
const float GEAR_RATIO = 75.81f;
// Total encoder clicks per revolution
const float CLICKS_PER_REVOLUTION = COUNTS_PER_REVOLUTION * GEAR_RATIO;
// Both measured in cm
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

const float DISTANCE_BETWEEN_WHEELS = 8.57f;

const int BASE_SPEED = 75;
const int SPEED_RANGE = 10;

const unsigned int NUM_GOALS = 2;
const float GOALS[NUM_GOALS * 2] = { 100.0f, 100.0f, -100.0f, 25.0f };
unsigned int goal = 0;

void setup() {
    Serial.begin(57600);
    delay(1000);
    buzzer.play("c32");
}

void loop() {
    unsigned long curMillis = millis();
    if (curMillis >= prevMillis + PERIOD) {
        updatePosition();
        adjustMotors();
    }
}

void updatePosition() {
    long countsLeft = encoders.getCountsAndResetLeft();
    long countsRight = encoders.getCountsAndResetRight();
    float deltaLeft = countsLeft * WHEEL_CIRCUMFERENCE / CLICKS_PER_REVOLUTION;
    float deltaRight = countsRight * WHEEL_CIRCUMFERENCE / CLICKS_PER_REVOLUTION;

    float deltaDistance = (deltaRight + deltaLeft) / 2;
    float deltaTheta = (deltaRight - deltaLeft) / 2;

    float deltaX = deltaDistance * cos(theta + deltaTheta / 2);
    float deltaY = deltaDistance * sin(theta + deltaTheta / 2);

    x += deltaX;
    y += deltaY;
    theta += deltaTheta;
    if (theta > M_PI) {
        float overlap = theta - M_PI;
        theta = -M_PI + overlap;
    } else if (theta < M_PI) {
        float overlap = theta + M_PI;
        theta = M_PI - overlap;
    }
}

void adjustMotors() {
    float targetTheta = atan2(GOALS[goal + 1] - y, GOALS[goal] - x);

    float eTheta = theta - targetTheta;
      
    
    // float normalizedPid = finalPidCalculation / PID_RANGE * HALF_SPEED_RANGE;
    //
    // int rightSpeed = (int)(BASE_SPEED + normalizedPid);
    // if (rightSpeed < MIN_SPEED) {
    //     rightSpeed = MIN_SPEED;
    // } else if (rightSpeed > MAX_SPEED) {
    //     rightSpeed = MAX_SPEED;
    // }
    //
    // int rightMag = rightSpeed - BASE_SPEED;
    // int leftSpeed = BASE_SPEED - rightMag;
    // if (leftSpeed < MIN_SPEED) {
    //     leftSpeed = MIN_SPEED;
    // } else if (leftSpeed > MAX_SPEED) {
    //     leftSpeed = MAX_SPEED;
    // }
    // 
    // motors.setSpeeds(-rightSpeed, -leftSpeed);
}

