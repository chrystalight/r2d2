#include <AccelStepper.h>
#include "MeOrion.h"

// Define motor driver pins (assuming A4988/DRV8825 drivers)
#define MOTOR_1_STEP 2
#define MOTOR_1_DIR 3
#define MOTOR_2_STEP 4
#define MOTOR_2_DIR 5
#define MOTOR_3_STEP 6
#define MOTOR_3_DIR 7

// Create stepper motor objects
AccelStepper motor1(AccelStepper::DRIVER, MOTOR_1_STEP, MOTOR_1_DIR);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR_2_STEP, MOTOR_2_DIR);
AccelStepper motor3(AccelStepper::DRIVER, MOTOR_3_STEP, MOTOR_3_DIR);

// Line follower module
MeLineFollower lineFinder(PORT_3);

// Define states
enum State {
    IDLE,
    FOLLOW_LINE,
    PERFORM_TASK,
    RETURN_HOME
};

State currentState = IDLE;

// Robot motion parameters
float linearVelocity = 0;  // Forward/backward speed
float angularVelocity = 0; // Rotation speed

void setup() {
    Serial.begin(9600);

    // Initialize motors
    motor1.setMaxSpeed(1000);
    motor1.setAcceleration(500);
    motor2.setMaxSpeed(1000);
    motor2.setAcceleration(500);
    motor3.setMaxSpeed(1000);
    motor3.setAcceleration(500);
}

void loop() {
    switch (currentState) {
        case IDLE:
            Serial.println("IDLE");
            if (/* condition to start */) {
                currentState = FOLLOW_LINE;
            }
            break;

        case FOLLOW_LINE:
            Serial.println("FOLLOWING LINE");
            followLine();
            if (/* condition to reach location */) {
                currentState = PERFORM_TASK;
            }
            break;

        case PERFORM_TASK:
            Serial.println("PERFORMING TASK");
            performTask();
            currentState = RETURN_HOME;
            break;

        case RETURN_HOME:
            Serial.println("RETURNING HOME");
            followLine();
            if (/* condition to reach home */) {
                currentState = IDLE;
            }
            break;
    }
}

// Line following function using MeOrion module
void followLine() {
    int sensorState = lineFinder.readSensors();

    switch (sensorState) {
        case S1_IN_S2_IN:  
            Serial.println("Centered on line"); 
            move(100, 0); // Move straight
            break;

        case S1_IN_S2_OUT:  
            Serial.println("Drifting right"); 
            move(80, -50); // Steer left
            break;

        case S1_OUT_S2_IN:  
            Serial.println("Drifting left"); 
            move(80, 50); // Steer right
            break;

        case S1_OUT_S2_OUT:  
            Serial.println("Lost the line"); 
            move(0, 0); // Stop or search for line
            break;
    }
}

// Function to set robot frame velocity (linear and angular)
void move(float vx, float vy, float omega) {

    // Compute wheel angular velocities
    float w1, w2, w3;
    computeWheelSpeeds(vx, vy, omega, w1, w2, w3);

    // Convert to stepper speeds
    float speed1 = computeStepperSpeed(w1);
    float speed2 = computeStepperSpeed(w2);
    float speed3 = computeStepperSpeed(w3);

    // Set motor speeds and directions
    motor1.setSpeed(speed1);
    motor2.setSpeed(speed2);
    motor3.setSpeed(speed3);

    // Run motors continuously
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();
}

// Task execution function
void performTask() {
    Serial.println("Executing task...");
    delay(3000); // Simulated task duration
}
void computeWheelSpeeds(float vx, float vy, float omega, float &w1, float &w2, float &w3) {
    float factor = 1.0 / wheel_radius;
    w1 = factor * (vx + robot_radius * omega);
    w2 = factor * (-0.5 * vx + 0.866 * vy + robot_radius * omega);
    w3 = factor * (-0.5 * vx - 0.866 * vy + robot_radius * omega);
}

// Convert wheel angular velocity (rad/s) to stepper motor speed (steps/s)
float computeStepperSpeed(float wheel_speed) {
    float rpm = (wheel_speed * 60) / (2 * PI);
    rpm = constrain(rpm, -stepper_max_rpm, stepper_max_rpm);
    float steps_per_sec = (rpm * steps_per_rev * gear_ratio) / 60;
    return steps_per_sec;
}
