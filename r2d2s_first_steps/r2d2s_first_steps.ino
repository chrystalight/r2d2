#include <SoftwareSerial.h>
#include <Servo.h>

#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header




// THE MAIN MOTOR DRIVER MUST BE CONNECTED TO THE TX/RX OR THE ARDUINO !!


//********************MAIN MOTOR DRIVER STUFF STARTS****************************
#define CMD (byte)0x00  // MD49 command address of 0
#define GET_VER 0x29
#define GET_ENC1 0x23
#define GET_ENC2 0x24
#define GET_VI 0x2C
#define GET_ERROR 0x2D
#define SET_ACCEL 0x33
#define SET_SPEED1 0x31
#define SET_SPEED2 0x32

byte enc1a, enc1b, enc1c, enc1d = 0;
byte enc2a, enc2b, enc2c, enc2d = 0;
byte bat_volt, mot1_cur, mot2_cur = 0;
byte ver = 0;
byte error = 0;
//********************MAIN MOTOR DRIVER STUFF ENDS****************************

//********************ARM EXTENSION STUFF STARTS****************************

Servo arm_servo;  // create servo object to control a servo

//UPDATE THESE VALUES WITH THE ACTUAL FINAL VALUES !!!
int arm_retracted_pos = 0;
int arm_extended_pos = 180;
int arm_servo_pin = 1;

//********************ARM EXTENSION STUFF ENDS****************************


//********************PUMP STUFF STARTS****************************
int pump_in_1 = 0; 
int pump_in_2 = 0;

int ms_per_ml_1 = 0;
int ms_per_ml_2 = 0;
int drink_vol = 250;

//********************PUMP STUFF STARTS****************************

//********************HEAD SPIN STUFF STARTS****************************
int headspin_in_1 = 0;
int headspin_in_2 = 0;
int headspin_pwm = 0;
int headspin_pwm_val = 0; //tuned value that spins the head at a reasonable speen

//using an L298N chip, the following functions are encoded within the two inputs:
//in1   in2   spin dir
//0     0     OFF
//1     0     FORWARD
//0     1     BACKWARD
//1     1     OFF

//pwm pin controls the speed of the headspin motor

//********************HEAD SPIN STUFF ENDS****************************

//********************LCD SCREEN STUFF STARTS****************************
const int LCD_COLS = 16;
const int LCD_ROWS = 2;
hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

//********************LCD SCREEN STUFF ENDS****************************

unsigned long previousMillis = 0;  // will store last time LED was updated
//

void setup() {

  //***MAIN MOTOR DRIVER SETUP**
  Serial.begin(9600);
  Serial.write(CMD);  // command byte
  Serial.write(SET_ACCEL);
  Serial.write(5);  // Set accelleration to 5
  //**EXTENDING ARM SETUP**
  arm_servo.attach();
  //**LCD SCREEN SETUP**
  int	status = lcd.begin(LCD_COLS, LCD_ROWS);
}

void go_forward(){
  Serial.write(CMD);
  Serial.write(SET_SPEED1);
  Serial.write(-180);  // Set motor 1 speed

  Serial.write(CMD);
  Serial.write(SET_SPEED2);
  Serial.write(-180);  // Set motor 2 speed
}

void go_backwards(){
  Serial.write(CMD);
  Serial.write(SET_SPEED1);
  Serial.write(-180);  // Set motor 1 speed

  Serial.write(CMD);
  Serial.write(SET_SPEED2);
  Serial.write(-180);  // Set motor 2 speed
}

void stop(){
  Serial.write(CMD);
  Serial.write(SET_SPEED1);
  Serial.write(0);  // Set motor 1 speed

  Serial.write(CMD);
  Serial.write(SET_SPEED2);
  Serial.write(0);  // Set motor 2 speed
}

void dispenseDrink(int strength){
  //where strength is an integer value between 0 and 1024 representing the strength of the drink
  //pump 1 dispenses alcohol, pump 2 dispenses mixer
  int ml_alcohol, ml_mixer;

  arm_servo.write(arm_extended_pos);
  //map strength to ms:
  ms_alcohol = int((strength/1024)*drink_vol*ms_per_ml_1);
  ms_mixer = int((1-strength/1024)*drink_vol*ms_per_ml_1);
  
  delay(200); //delay to ensure arm movement is complete
  digitalWrite(pump_in_1, HIGH);
  delay(ms_alcohol);
  digitalWrite(pump_in_1, LOW);

  digitalWrite(pump_in_2, HIGH);
  delay(ms_alcohol);
  digitalWrite(pump_in_2, LOW);

  arm_servo.write(arm_retracted_pos);
  completeDrink();

  delay(200); //delay to ensure arm movement is complete

}

void completeDrink(){
  //function to notify the user that the drink has been dispensed
  //communicate to the user module
}

void startHeadSpin(bool clockwise){
  if(clockwise){
    digitalWrite(headspin_in_1, HIGH);
    digitalWrite(headspin_in_2, LOW);
  }
  else{
    digitalWrite(headspin_in_1, LOW);
    digitalWrite(headspin_in_2, HIGH);
  }
  analogWrite(headspin_pwm, headspin_pwm_val);
}

void stopHeadSpin(bool clockwise){
  if(clockwise){
    digitalWrite(headspin_in_1, HIGH);
    digitalWrite(headspin_in_2, LOW);
  }
  else{
    digitalWrite(headspin_in_1, LOW);
    digitalWrite(headspin_in_2, HIGH);
  }
  analogWrite(headspin_pwm, headspin_pwm_val);
}

void loop() {


}



  // Serial.write(CMD);
  // Serial.write(GET_VER);  // Recieve version back
  // delay(50);
  // if (Serial.available() > 0) {
  //   ver = Serial.read();
  // }

  // Serial.write(CMD);
  // Serial.write(GET_ERROR);  // Recieve error byte back
  // delay(50);
  // if (Serial.available() > 0) {
  //   error = Serial.read();
  // }

  // Serial.write(CMD);
  // Serial.write(GET_VI);  // Recieve battery volts and both motor currents back
  // delay(50);
  // if (Serial.available() > 2) {
  //   bat_volt = Serial.read();
  //   mot1_cur = Serial.read();
  //   mot2_cur = Serial.read();
  // }

  // Serial.write(CMD);
  // Serial.write(GET_ENC1);  // Recieve encoder 1 value
  // delay(50);
  // if (Serial.available() > 3) {
  //   enc1a = Serial.read();
  //   enc1b = Serial.read();
  //   enc1c = Serial.read();
  //   enc1d = Serial.read();
  // }

  // Serial.write(CMD);
  // Serial.write(GET_ENC2);  // Recieve encoder 2 value
  // delay(50);
  // if (Serial.available() > 3) {
  //   enc2a = Serial.read();
  //   enc2b = Serial.read();
  //   enc2c = Serial.read();
  //   enc2d = Serial.read();
  // }