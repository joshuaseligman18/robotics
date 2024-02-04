#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

// Hardware on the robot
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
// travelled by each motor in cm
float positionLeft = 0.0f;
float positionRight = 0.0f;
int motorSpeed = 75;

// Keeps track of the goals
const int NUM_GOALS = 3;
// All goals are in cm
float goals[NUM_GOALS] = { 30.48f, 0.0f, 45.72f };
int goalIndex = 0;

// Encoder resolution
const int COUNTS_PER_REVOLUTION = 12;
const float GEAR_RATIO = 75.81f;
// Total encoder clicks per revolution
const float CLICKS_PER_REVOLUTION = COUNTS_PER_REVOLUTION * GEAR_RATIO;
// Both measured in cm
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

void setup() {
    Serial.begin(57600);
    buzzer.play("c32");
    delay(1000);
}

void loop() {
    // Only need to check encoders if there are still goals to complete
    if (goalIndex < NUM_GOALS) {
        checkEncoders();
    }

    // Only need to check goals if the robot is stopped
    if (motorSpeed == 0) {
        checkGoals();
    }
}

void checkEncoders() {
    currentMillis = millis();
    if (currentMillis > prevMillis + PERIOD) {
        // Update the position of the robot in encoder counts
        countsLeft += encoders.getCountsAndResetLeft();
        countsRight += encoders.getCountsAndResetRight();

        // Update the position of the robot in cm
        positionLeft += (countsLeft - prevCountsLeft) * WHEEL_CIRCUMFERENCE / CLICKS_PER_REVOLUTION;
        positionRight += (countsRight - prevCountsRight) * WHEEL_CIRCUMFERENCE / CLICKS_PER_REVOLUTION;
        Serial.print("(");
        Serial.print(positionLeft);
        Serial.print(", ");
        Serial.print(positionRight);
        Serial.println(")");

        // Get the distance to the goal and obtain the abs for consistent
        // calculations regardless of direction
        float distanceToGoal = goals[goalIndex] - positionLeft;
        float distanceToGoalAbs = abs(distanceToGoal);

        // 0.02 is an epsilon to handle the absolute value
        // because cannot say distance < 0 
        if (distanceToGoalAbs > 0.02) {
            // Slow down the robot as it approaches the target
            if (distanceToGoalAbs < 10) {
                motorSpeed = 75 * distanceToGoalAbs / 10;

                // Keep a minimum speed for the robot so it doesn't stall
                if (motorSpeed < 25) {
                    motorSpeed = 25;
                }
            } else {
                motorSpeed = 75;
            }

            // Negate the speed if have to go backward
            if (distanceToGoal < 0) {
                motorSpeed = -1 * motorSpeed;
            }
        } else {
            motorSpeed = 0;
        }
        motors.setSpeeds(motorSpeed, motorSpeed);

        prevCountsLeft = countsLeft;
        prevCountsRight = countsRight;
        prevMillis = currentMillis;
    }
}

void checkGoals() {
    // Completed a goal
    if (goalIndex < NUM_GOALS) {
        Serial.print("Finished goal ");
        Serial.println(goalIndex);
        goalIndex++;
    }

    // If still more to go, give a tiny amount of time to delay
    if (goalIndex < NUM_GOALS) {
        delay(250);
    } else {
        // Otherwise beep because we are done
        buzzer.play("c32");
        delay(1000);
    }
}

