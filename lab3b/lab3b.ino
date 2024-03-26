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

const float MAX_DISTANCE = 300.0f;

const unsigned long PID_PERIOD = 20;
unsigned long prevPidMillis = 0;

const unsigned long HEAD_MOVE_DURATION = 200;
unsigned long headMoveStart = 0;
bool headDoneMoving = true;

const float TARGET = 30.48f;

const bool P_ON = true;
const bool I_ON = false;
const bool D_ON = true;
const bool HEAD_ON = true;

const float K_P = 2.225f;
const float K_I = 0.005f;
const float K_D = 0.225f;

const int BASE_SPEED = 100;
const int MIN_SPEED = 75;
const int MAX_SPEED = 125;
const int HALF_SPEED_RANGE = (MAX_SPEED - MIN_SPEED) / 2;

const int PID_THRESHOLD = 5;
const float PID_MIN = -1 * PID_THRESHOLD * TARGET;
const float PID_MAX = PID_THRESHOLD * TARGET;
const float PID_RANGE = PID_MAX;

float prevError = 0.0f;
float errorSum = 0.0f;
const float INTEGRAL_THRESHOLD = 10.0f;

const unsigned long HEAD_PERIOD = 750;
unsigned long prevHeadMillis = 0;
const int SIDE_POSITION = 170;
const int FRONT_POSITION = 90;
const float TARGET_FRONT = 30.48f;
const float K_P_FRONT = 4.25f;
float pidAdjustment = 0;

const int VALUES_SIZE = 5;
float pidValues[VALUES_SIZE] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
int valueIndex = 0;
int sideValueIndex = 0;

void setup() {
    Serial.begin(57600);

    // Set up servo pins
    headServo.attach(HEAD_SERVO_PIN);
    headServo.write(SIDE_POSITION);
    
    // Set up US pins
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    delay(1000);
    buzzer.play("c32");
}

void loop() {
    unsigned long curMillis = millis();
    if (curMillis >= prevPidMillis + PID_PERIOD) {
        bool ran = false;
        if (valueIndex < VALUES_SIZE - 1) {
            ran = checkSide();
        } else {
            ran = checkFront(); 
        }

        if (ran) {
            if (valueIndex < VALUES_SIZE - 1) {
                sideValueIndex = valueIndex;
            }

            valueIndex++;
            if (valueIndex == VALUES_SIZE) {
                valueIndex = 0;
            }

            if (valueIndex == VALUES_SIZE - 1) {
                if (HEAD_ON) {
                    headServo.write(FRONT_POSITION);
                    headMoveStart = curMillis;
                    headDoneMoving = false;
                } else {
                    pidValues[valueIndex] = 0.0f;
                    valueIndex = 0;
                }
            } else if (valueIndex == 0) {
                if (HEAD_ON) {
                    headServo.write(SIDE_POSITION);
                    headMoveStart = curMillis;
                    headDoneMoving = false;
                }
            }
            prevPidMillis = curMillis;
        }
        adjustMotors(pidValues[sideValueIndex] + pidValues[4]);
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

bool checkSide() {
    unsigned long curMillis = millis();
    if (headDoneMoving || (!headDoneMoving && curMillis > headMoveStart + HEAD_MOVE_DURATION)) {
        headDoneMoving = true;
        float curDistance = usReadCm();
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
            pidCalculation += K_D * dErr;
            prevError = error;
        }

        if (pidCalculation < PID_MIN) {
            pidCalculation = PID_MIN;
        } else if (pidCalculation > PID_MAX) {
            pidCalculation = PID_MAX;
        }

        pidValues[valueIndex] = pidCalculation;

        return true;
    } else {
        return false;
    }
}

float checkFront() {
    float curMillis = millis();
    if (headDoneMoving || (!headDoneMoving && curMillis > headMoveStart + HEAD_MOVE_DURATION)) {
        headDoneMoving = true;
        float distanceAhead = usReadCm();
        float error = distanceAhead - TARGET_FRONT;
        if (error < 0) {
            error *= -1;
        } else if (error > TARGET_FRONT) {
            error = TARGET_FRONT;
        }
        // We want the inverse error, so closer to the target means more action
        error = TARGET_FRONT - error;
        
        pidAdjustment = -1 * K_P_FRONT * error;        

        pidValues[valueIndex] = pidAdjustment;

        return true;
    } else {
        return false;
    }
}

void adjustMotors(float finalPidCalculation) {
    float normalizedPid = finalPidCalculation / PID_RANGE * HALF_SPEED_RANGE;

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
