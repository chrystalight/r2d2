#include <AccelStepper.h>

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

const float wheel_radius = 0.05;  // 5 cm wheel radius
const float robot_radius = 0.15;  // 15 cm distance from center to wheel
const int steps_per_rev = 200;    // NEMA 17 (1.8° per step)
const float gear_ratio = 1.0;     // Modify if using gears
const float stepper_max_rpm = 100; // Max RPM

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
    motor1.setSpeed(speed1*5);
    motor2.setSpeed(speed2*5);
    motor3.setSpeed(speed3*5);

    // Run motors continuously
    motor1.runSpeed();
    motor2.runSpeed();
    motor3.runSpeed();
}
void computeWheelSpeeds(float vx, float vy, float omega, float &w1, float &w2, float &w3) {

    float factor = 1.0 / wheel_radius;
    w1 = factor * (vx + robot_radius * omega);
    w2 = factor * (-0.5 * vx + 0.866 * vy + robot_radius * omega);
    w3 = factor * (-0.5 * vx - 0.866 * vy + robot_radius * omega);
}
float computeStepperSpeed(float wheel_speed) {
    float rpm = (wheel_speed * 60) / (2 * PI);
    rpm = constrain(rpm, -stepper_max_rpm, stepper_max_rpm);
    float steps_per_sec = (rpm * steps_per_rev * gear_ratio) / 60;
    return steps_per_sec;
}
void loop(){
  unsigned long start_time = millis();
  while (millis() - start_time < 3000){
    move(0, 0,1);
  }

    

}
