#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

enum class CheckRange {
    Full,
    Left,
    Right
};

Buzzer buzzer;
Motors motors;
Encoders encoders;
Servo headServo;

// Pins
const int HEAD_SERVO_PIN = 22;
const int ECHO_PIN = 4;
const int TRIG_PIN = 5;

const bool SERVO_ON = true;

unsigned long prevPfMillis = 0;
const unsigned long PF_PERIOD = 10;
unsigned long prevServoMillis = 0;
const unsigned long SERVO_PERIOD = 175;

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

const int NUM_GOALS = 1;
const float GOALS[NUM_GOALS][2] = {
    { 100.0f, 304.8f },
};
int goalIndex = 0;
bool approachingGoal = false;
bool atGoal = false;

const float MAX_DISTANCE = 100.0f;
const int NUM_POSITIONS = 5;
float measurements[NUM_POSITIONS] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };
const float HEAD_POSITIONS[NUM_POSITIONS] = { 120.0f, 105.0f, 90.0f, 75.0f, 60.0f };
int headIndex = 2;

const float K_P = 45.0f;
const float K_SIDE = 0.029f;
const float K_MID = 0.30f;
const float K_FRONT = 0.85f;
const float POSITION_MULTIPLIERS[NUM_POSITIONS] = { K_SIDE, K_MID, K_FRONT, K_MID, K_SIDE };

CheckRange checkRange = CheckRange::Full;
const float CHECK_THRESHOLD = 30.48f * 3.5f;
const float K_SIDE_CHECK = 1.775f;

void setup() {
    Serial.begin(57600);

    // Set up servo pins
    headServo.attach(HEAD_SERVO_PIN);
    headServo.write(HEAD_POSITIONS[headIndex]);

    // Set up US pins
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    runInitialScan();

    delay(1000);
    buzzer.play("c32");
}

void loop() {
    unsigned long curMillis = millis();
    if (curMillis >= prevPfMillis + PF_PERIOD) {
        if (goalIndex < NUM_GOALS) {
            updatePosition();
            runHeadCalculation();
            runPotentialFields();
        }
        checkGoal();
        prevPfMillis = curMillis;
    }
}

void updatePosition() {
    long countsLeft = -1 * encoders.getCountsAndResetRight();
    long countsRight = -1 * encoders.getCountsAndResetLeft();
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
    
    Serial.println(distance);

    return distance;
}

void runPotentialFields() {
    float targetTheta = atan2(GOALS[goalIndex][1] - y, GOALS[goalIndex][0] - x);

    // Bring target theta to have the same sign as theta
    if (theta > targetTheta && theta / targetTheta < 0) {
        targetTheta += 2 * M_PI;
    } else if (theta < targetTheta && theta / targetTheta < 0) {
        targetTheta -= 2 * M_PI;
    }

    float eTheta = normalizeAngle(theta - targetTheta);
    float distanceToGoal =
        sqrt(pow(GOALS[goalIndex][0] - x, 2) + pow(GOALS[goalIndex][1] - y, 2));

    
    float pfCalculation = eTheta * K_P;
    float adjustmentSum = 0.0f;

    if (SERVO_ON) {
        float adjustments[NUM_POSITIONS] = { 0, 0, 0, 0, 0 };
        for (int i = 0; i < NUM_POSITIONS; i++) { 
            float adjustment = (MAX_DISTANCE - measurements[i]) * POSITION_MULTIPLIERS[i];
            if (distanceToGoal < measurements[i]) {
                // Keep adjustment at 0 because goal is closer
                continue;
            }
            adjustments[i] = adjustment;
        }

        for (int i = 0; i < NUM_POSITIONS; i++) {
            if (i < 2) {
                if (checkRange == CheckRange::Left) {
                    adjustmentSum += K_SIDE_CHECK * adjustments[i];
                } else {
                    adjustmentSum += adjustments[i];
                }
            } else if (i > 2) {
                if (checkRange == CheckRange::Right) {
                    adjustmentSum -= K_SIDE_CHECK * adjustments[i];
                } else {
                    adjustmentSum -= adjustments[i];
                }
            } else {
                switch (checkRange) {
                    case CheckRange::Left:
                        adjustmentSum += adjustments[i];
                        break;
                    case CheckRange::Right:
                        adjustmentSum -= adjustments[i];
                        break;
                    case CheckRange::Full:
                        float leftWeight = adjustments[0] + adjustments[1];
                        float rightWeight = adjustments[3] + adjustments[4];
                        if (leftWeight >= rightWeight) {
                            adjustmentSum += adjustments[i];
                        } else {
                            adjustmentSum -= adjustments[i];
                        }
                        break;
                }
            }
        }

        switch (getClosestIndex()) {
            case 0:
                adjustmentSum += adjustments[0];
            case 1:
                adjustmentSum += 0.5f * adjustments[1];
            case 3:
                adjustmentSum -= 0.5f * adjustments[3];
            case 4:
                adjustmentSum -= adjustments[4];
        }
        pfCalculation += adjustmentSum;
    }

    int leftSpeed = BASE_SPEED + pfCalculation;
    int rightSpeed = BASE_SPEED - pfCalculation;

    if (distanceToGoal > 1.5f) {
        // Slow down the robot as it approaches the target
        if (distanceToGoal < 10 && abs(adjustmentSum) < 1.0f) {
            if (!approachingGoal) {
                buzzer.play("e32");
                approachingGoal = true;
            }
            leftSpeed *= distanceToGoal / 10;
            rightSpeed *= distanceToGoal / 10;

            // Keep a minimum speed for the robot so it doesn't stall
            if (leftSpeed < 30) {
                leftSpeed = 30;
            }

            if (rightSpeed < 30) {
                rightSpeed = 30;
            }
        } else {
            approachingGoal = false;
        }
    } else {
        leftSpeed = 0;
        rightSpeed = 0;
        atGoal = true;
    }

    motors.setSpeeds(-rightSpeed, -leftSpeed);
}

void runHeadCalculation() {
    if (SERVO_ON) {
        unsigned long curMillis = millis();
        if (curMillis >= prevServoMillis + SERVO_PERIOD) {
            measurements[headIndex] = usReadCm();
            updateCheckRange();
            moveHeadToNextPosition();
            prevServoMillis = curMillis;
        }
    }
}

void moveHeadToNextPosition() {
    headIndex++;
    switch (checkRange) {
        case CheckRange::Full:
            if (headIndex == NUM_POSITIONS) {
                headIndex = 0;
            }
            break;
        case CheckRange::Left:
            if (headIndex >= 3) {
                headIndex = 0;
            }
            break;
        case CheckRange::Right:
            if (headIndex == NUM_POSITIONS) {
                headIndex = 2;
            }
            break;
    }
    headServo.write(HEAD_POSITIONS[headIndex]);
}


float normalizeAngle(float angle) {
    if (angle < -1 * M_PI) {
        angle += 2 * M_PI;
    } else if (angle > M_PI) {
        angle -= 2 * M_PI;
    }

    return angle;
}

void runInitialScan() {
    if (SERVO_ON) {
        for (int i = 0; i < NUM_POSITIONS; i++) {
            headServo.write(HEAD_POSITIONS[i]);
            delay(SERVO_PERIOD);
            measurements[i] = usReadCm();
        }
        headIndex = 2;
        headServo.write(HEAD_POSITIONS[headIndex]);
        delay(SERVO_PERIOD);
    }
}

void checkGoal() {
    if (atGoal) {
        if (goalIndex < NUM_GOALS) {
            goalIndex++;
        }

        if (goalIndex == NUM_GOALS) {
            buzzer.play("g32");
        } else {
            buzzer.play("c32");
            atGoal = false;
        }

        delay(1000);
    }
}

void updateCheckRange() {
    float leftSum = measurements[0] + measurements[1];
    float rightSum = measurements[3] + measurements[4];

    float diff = leftSum - rightSum;
    if (diff <= -1 * CHECK_THRESHOLD) {
        if (checkRange == CheckRange::Full) {
            buzzer.play("a32");
        }
        checkRange = CheckRange::Left;
    } else if (diff >= CHECK_THRESHOLD) {
        if (checkRange == CheckRange::Full) {
            buzzer.play("a32");
        }
        checkRange = CheckRange::Right;
    } else {
        checkRange = CheckRange::Full;
    }
}

int getClosestIndex() {
    int lowestIndex = -1;
    for (int i = 0; i < NUM_POSITIONS; i++) {
        if (measurements[i] < 10.0f) {
            if (lowestIndex == -1 || measurements[i] < measurements[lowestIndex]) {
                lowestIndex = i;
            }
        }
    }
    return lowestIndex;
}
