#define CMD (byte)0x00 
#define SET_SPEED1 0x31
#define SET_SPEED2 0x32 

const float wheel_radius = 0.04;  
const float wheel_base = 0.27;  // distance between wheels

void setDifferentialDriveSpeed(float v, float omega) {
    // Compute wheel speeds using differential drive kinematics
    float v_left = v - (omega * wheel_base / 2);
    float v_right = v + (omega * wheel_base / 2);

    // Convert to motor speed (assumes linear mapping)
    int speed_left = (int)(v_left / wheel_radius);
    int speed_right = (int)(v_right / wheel_radius);

    // Send commands to motors
    Serial.write(CMD);
    Serial.write(SET_SPEED1);
    Serial.write(speed_left);

    Serial.write(CMD);
    Serial.write(SET_SPEED2);
    Serial.write(speed_right);
}
