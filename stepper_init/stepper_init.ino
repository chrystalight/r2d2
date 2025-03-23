#include <AccelStepper.h>

#define MOTOR_1_STEP 2
#define MOTOR_1_DIR 3

AccelStepper motor1(AccelStepper::DRIVER, MOTOR_1_STEP, MOTOR_1_DIR);
void setup() {
    Serial.begin(9600);

    // Initialize motors
    motor1.setMaxSpeed(1000);
    motor1.setAcceleration(500);

}
void loop(){
  float speed1 = 100;
  motor1.setSpeed(speed1);
  motor1.runSpeed();

}
