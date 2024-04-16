#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
Encoders encoders;
Servo headServo;

// Pins
const int HEAD_SERVO_PIN = 22;
const int ECHO_PIN = 4;
const int TRIG_PIN = 5;

const bool SERVO_ON = false;

unsigned long prevPfMillis = 0;
const unsigned long PF_PERIOD = 10;
unsigned long prevServoMillis = 0;
const unsigned long SERVO_PERIOD = 150;

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

const int NUM_GOALS = 2;
const float GOALS[NUM_GOALS][2] = {
    { 30.48f, 152.4f },
    { 0.0f, 0.0f }
};
int goalIndex = 0;
bool atGoal = false;

const float MAX_DISTANCE = 150.0f;
const int NUM_POSITIONS = 5;
float measurements[NUM_POSITIONS] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };
const float HEAD_POSITIONS[NUM_POSITIONS] = { 120.0f, 105.0f, 90.0f, 75.0f, 60.0f };
int headIndex = 2;
bool headMovingRight = true;

const float K_P = 100.0f;
const float POSITION_MULTIPLIERS[NUM_POSITIONS] = { 0.5f, 1.25f, 2.5f, 1.25f, 0.5f };

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

    if (SERVO_ON) {
        for (int i = 0; i < NUM_POSITIONS; i++) { 
            float adjustment = (MAX_DISTANCE - measurements[i]) * POSITION_MULTIPLIERS[i];
            if (distanceToGoal < measurements[i]) {
                adjustment /= 15;
            }
            if (i <= 2) {
                pfCalculation += adjustment;
            } else {
                pfCalculation -= adjustment;
            }
        }
    }

    int leftSpeed = BASE_SPEED + pfCalculation;
    int rightSpeed = BASE_SPEED - pfCalculation;

    if (distanceToGoal > 1.0f) {
        // Slow down the robot as it approaches the target
        if (distanceToGoal < 10) {
            leftSpeed *= distanceToGoal / 10;
            rightSpeed *= distanceToGoal / 10;

            // Keep a minimum speed for the robot so it doesn't stall
            if (leftSpeed < 30) {
                leftSpeed = 30;
            }

            if (rightSpeed < 30) {
                rightSpeed = 30;
            }
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
            moveHeadToNextPosition();
            prevServoMillis = curMillis;
        }
    }
}

void moveHeadToNextPosition() {
    if (headMovingRight) {
        headIndex++;
        if (headIndex == NUM_POSITIONS - 1) {
            headMovingRight = false;
        }
    } else {
        headIndex--;
        if (headIndex == 0) {
            headMovingRight = true;
        }
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
