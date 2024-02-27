#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Servo headServo;

// Flags for filtering out log messages
const boolean HEAD_DEBUG = false;
const boolean TIMING_DEBUG = false;
const boolean US_DEBUG = true;

// Flags for if the components should be running
const boolean SERVO_ON = true;
const boolean US_ON = true;

// Last time the head moved
unsigned long headPm;
// Time between head movements
const unsigned long HEAD_MOVEMENT_PERIOD = 120;

// Head settings and positions
const int HEAD_SERVO_PIN = 22;
const int NUM_HEAD_POSITIONS = 7;
const int HEAD_POSITIONS[NUM_HEAD_POSITIONS] = {135, 120, 105, 90, 75, 60, 45};

// Current head position variables
boolean headDirectionClockwise = false;
int currentHeadPosition = 1;

// US pins
const int ECHO_PIN = 4;
const int TRIG_PIN = 5;

// US variables
const float MAX_DISTANCE = 300.0f;
// Wait time after head movement to start US reading
const unsigned long WAIT_AFTER_HEAD_START_MOVING = 80;
boolean usReadFlag = false;

float distanceReadings[NUM_HEAD_POSITIONS];

void setup() {
    Serial.begin(57600);

    // Set up servo pins
    headServo.attach(HEAD_SERVO_PIN);
    headServo.write(90);
    
    // Set up US pins
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    // Initialize data
    for (int i = 0; i < NUM_HEAD_POSITIONS; i++) {
        distanceReadings[i] = MAX_DISTANCE;
    }

    delay(3000);
    buzzer.play("c32");
}

void loop() {
    moveHead();
    usReadCm();
}

void moveHead() {
    unsigned long curTime = millis();
    // Check to make sure it is time to move again
    if (curTime > headPm + HEAD_MOVEMENT_PERIOD) {
        // Set the position of the servo for the next movement
        if (headDirectionClockwise) {
            if (currentHeadPosition >= NUM_HEAD_POSITIONS - 1) {
                // Switch directions
                headDirectionClockwise = !headDirectionClockwise;
                currentHeadPosition--;
            } else {
                // Keep moving in the same direction
                currentHeadPosition++;
            }
        } else {
            if (currentHeadPosition <= 0) {
                // Switch directions
                headDirectionClockwise = !headDirectionClockwise;
                currentHeadPosition++;
            } else {
                // Keep moving in the same direction
                currentHeadPosition--;
            }
        }

        if (SERVO_ON) {
            headServo.write(HEAD_POSITIONS[currentHeadPosition]);
        }

        if (TIMING_DEBUG) {
            Serial.print("Move head initiated: ");
            Serial.println(curTime);
        }

        if (HEAD_DEBUG) {
            Serial.print(currentHeadPosition);
            Serial.print(" - ");
            Serial.println(HEAD_POSITIONS[currentHeadPosition]);
        }


        // Update the values for the US to work
        headPm = curTime;
        usReadFlag = false;
    }
}

void usReadCm() {
    unsigned long curTime = millis();
    // Only work if it is reading time and have not already read
    if (curTime > headPm + WAIT_AFTER_HEAD_START_MOVING && !usReadFlag) {
        if (TIMING_DEBUG) {
            Serial.print("US read initiated: ");
            Serial.println(curTime);
        }

        if (US_ON) {
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

            // Store the data
            distanceReadings[currentHeadPosition] = distance;
        }

        if (TIMING_DEBUG) {
            Serial.print("US read finished: ");
            Serial.println(millis());
        }

        // Print the readings array
        if (US_DEBUG) {
            Serial.print("Distance readings: [ ");
            for (int i = 0; i < NUM_HEAD_POSITIONS; i++) {
                Serial.print(distanceReadings[i]);
                if (i  < NUM_HEAD_POSITIONS - 1) {
                    Serial.print(" - ");
                }
            }
            Serial.println(" ]");
        }

        // We have read now, so no more readings
        usReadFlag = true;
    }
}
