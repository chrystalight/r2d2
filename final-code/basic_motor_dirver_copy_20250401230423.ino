// Include required libraries
#include <Servo.h>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include <SoftwareSerial.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>

// BLE Configuration
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4
#define FACTORYRESET_ENABLE            1
#define MINIMUM_FIRMWARE_VERSION       "0.6.6"
#define MODE_LED_BEHAVIOUR            "MODE"
#define BLUEFRUIT_HWSERIAL_NAME       "Adafruit Bluefruit LE"

// BLE Objects
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// BLE Service and Characteristic UUIDs
#define BLE_SERVICE_UUID              "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_CHARACTERISTIC_UUID       "19B10001-E8F2-537E-4F6C-D104768A1214"

// BLE Commands
#define CMD_INTERRUPT                 0x01
#define CMD_GET_POT                  0x02
#define CMD_POT_VALUE                0x03

// System Constants
const unsigned long SYSTEM_TIMEOUT = 30000;  // 30 second system timeout
const unsigned long CUP_DETECTION_TIMEOUT = 10000;  // 10 second timeout for cup detection
const unsigned long SERVO_MOVE_DELAY = 15;  // ms between servo movements
unsigned long PUMP_DURATION = 10000;  // ms pump activation time
const unsigned long SENSOR_READ_INTERVAL = 100;  // ms between sensor reads
const unsigned long LCD_UPDATE_INTERVAL = 500;  // ms between LCD updates
const unsigned long BLE_UPDATE_INTERVAL = 100;  // ms between BLE updates

// Hardware Configuration
const int LCD_COLS = 16;
const int LCD_ROWS = 2;
const float WHEEL_BASE = 0.27;  // Distance between wheels in meters

// Pin Definitions
const int SERVO_PIN = A0;
const int PUMP_PIN = 11;
const int HEADSPIN_PIN1 = A1;
const int HEADSPIN_PIN2 = A2;
const int BUTTON_PIN = 2;
const int CUP_SENSOR_PIN = A3;

// Sensor Thresholds
const int CUP_DETECTION_THRESHOLD = 400;
const int SENSOR_READINGS_TO_AVERAGE = 5;

// MD49 Motor Driver Commands
#define CMD (byte)0x00  // MD49 command address of 0
#define GET_VER 0x29
#define GET_ENC1 0x23
#define GET_ENC2 0x24
#define GET_VI 0x2C
#define GET_ERROR 0x2D
#define SET_ACCEL 0x33
#define SET_SPEED1 0x31
#define SET_SPEED2 0x32

// System States
enum SystemState {
    STATE_IDLE = 1,
    STATE_TRAVELLING,
    STATE_DISPENSING,
    STATE_RETURNING,
    STATE_ERROR
};

// Hardware Objects
bool lcdInitialized = false;
Servo myservo;  // Servo object for dispensing mechanism
hd44780_I2Cexp lcd;
Servo myservo;

// System Variables
volatile bool buttonPressed = false;
volatile bool systemError = false;
volatile SystemState currentState = STATE_IDLE;

// Timing Variables
unsigned long lastMillis = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastSensorRead = 0;
unsigned long cupDetectionStartTime = 0;
unsigned long previousSpinMillis = 0;
unsigned long spinStartTime = 0;

// State Variables
bool headSpinning = false;
bool clockwise = true;
bool firstGo = true;
int spinDuration = 4500;
int servoPosition = 0;

// Motor encoder values
byte enc1a, enc1b, enc1c, enc1d = 0;
byte enc2a, enc2b, enc2c, enc2d = 0;
byte bat_volt, mot1_cur, mot2_cur = 0;
byte ver = 0;
byte error = 0;

// Function Declarations
void setDifferentialDriveSpeed(float v, float omega);
void startHeadspin();
void stopHeadspin();
void scheduleNextSpin();
bool waitForCupDetection();
bool initializeHardware();
void updateLcdDisplay();
void handleError();
void setupBLE();
void handleBLEInterrupt();
void requestPotValue();

// Interrupt Service Routine for button press
void buttonISR() {
    buttonPressed = true;
}

// Initialize all hardware components
bool initializeHardware() {
    // Initialize BLE
    setupBLE();
    
    // Initialize pump control
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    
    // Initialize button with pull-up resistor
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize cup sensor pin
    pinMode(CUP_SENSOR_PIN, INPUT);
    
    // Initialize servo
    if (!myservo.attach(SERVO_PIN)) {
        return false;
    }
    myservo.write(0);
    
    // Initialize LCD
    int status = lcd.begin(LCD_COLS, LCD_ROWS);
    if(status) {
        hd44780::fatalError(status);
        return false;
    }
    lcd.print("Initializing ");
    
    // Initialize serial communication with MD49
    Serial.begin(9600);
    Serial.write(CMD);
    Serial.write(0x38);
    Serial.write(CMD);
    Serial.write(0x34);
    Serial.write(0x01);
    
    return true;
}

// Function to wait for cup detection with timeout and averaging
bool waitForCupDetection() {
    cupDetectionStartTime = millis();
    int readings[SENSOR_READINGS_TO_AVERAGE];
    int readIndex = 0;
    
    while (millis() - cupDetectionStartTime < CUP_DETECTION_TIMEOUT) {
        if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL) {
            // Take multiple readings and average them
            readings[readIndex] = analogRead(CUP_SENSOR_PIN);
            readIndex = (readIndex + 1) % SENSOR_READINGS_TO_AVERAGE;
            
            // Calculate average
            int sum = 0;
            for (int i = 0; i < SENSOR_READINGS_TO_AVERAGE; i++) {
                sum += readings[i];
            }
            int average = sum / SENSOR_READINGS_TO_AVERAGE;
            
            if (average < CUP_DETECTION_THRESHOLD) {
                return true;
            }
            
            lastSensorRead = millis();
        }
    }
    
    return false;
}

// Update LCD display with current state
void updateLcdDisplay() {
    if (millis() - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
        lcd.setCursor(0,0);
        switch (currentState) {
            case STATE_IDLE:
                lcd.print("STATE: IDLE     ");
                break;
            case STATE_TRAVELLING:
                lcd.print("STATE:TRAVELLING");
                break;
            case STATE_DISPENSING:
                lcd.print("DISPENSING DRINK");
                break;
            case STATE_RETURNING:
                lcd.print("GOING HOME      ");
                break;
            case STATE_ERROR:
                lcd.print("ERROR!          ");
                break;
        }
        lastLcdUpdate = millis();
    }
}

// Handle system errors
void handleError() {
    systemError = true;
    currentState = STATE_ERROR;
    stopHeadspin();
    setDifferentialDriveSpeed(0, 0);
    digitalWrite(PUMP_PIN, LOW);
    myservo.write(0);
}

// BLE Functions
void error(const __FlashStringHelper*err) {
    Serial.println(err);
    while (1);
}

void setupBLE() {
    if (!ble.begin(VERBOSE_MODE)) {
        error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }

    if (FACTORYRESET_ENABLE) {
        if (!ble.factoryReset()) {
            error(F("Couldn't factory reset"));
        }
    }

    if (!ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
        error(F("Callback requires at least 0.6.6"));
    }

    // Set up BLE service and characteristic
    ble.sendCommandWithArg(F("AT+GATTADDSERVICE=UUID=0x"), BLE_SERVICE_UUID);
    ble.sendCommandWithArg(F("AT+GATTADDCHAR=UUID=0x"), BLE_CHARACTERISTIC_UUID);
    
    // Enable notifications
    ble.sendCommandCheckOK(F("AT+GATTSTART"));
    
    // Set LED mode
    ble.sendCommandCheckOK(F("AT+HWModeLED=" MODE_LED_BEHAVIOUR));
    
    Serial.println(F("BLE Setup Complete"));
}

void handleBLEInterrupt() {
    if (ble.read()) {
        uint8_t cmd = ble.read();
        if (cmd == CMD_INTERRUPT) {
            buttonPressed = true;
        } else if (cmd == CMD_POT_VALUE) {
            // Read potentiometer value (2 bytes)
            uint16_t potValue = (ble.read() << 8) | ble.read();
            // Map potentiometer value (0-1023) to pump duration (1000-10000ms)
            PUMP_DURATION = map(potValue, 0, 1023, 1000, 10000);
        }
    }
}

void requestPotValue() {
    ble.write(CMD_GET_POT);
}

void setup() {
    if (!initializeHardware()) {
        // If hardware initialization fails, enter error state
        handleError();
        return;
    }
    
    // Attach interrupt for button press (FALLING edge)
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    
    lastMillis = millis();
    lcd.print("Initializing ");
    delay(3000);
}

void loop() {
    unsigned long currentMillis = millis();
    
    // Handle BLE communication
    if (currentMillis - lastMillis >= BLE_UPDATE_INTERVAL) {
        handleBLEInterrupt();
        requestPotValue();
        lastMillis = currentMillis;
    }
    
    // Check for system timeout
    if (currentMillis - lastMillis > SYSTEM_TIMEOUT) {
        handleError();
    }
    
    // Update display
    updateLcdDisplay();
    
    // Main state machine
    switch (currentState) {
        case STATE_IDLE:
            // Handle head spinning in idle state
            if (!headSpinning) {
                if (currentMillis - previousSpinMillis >= spinInterval) {
                    startHeadspin();
                    headSpinning = true;
                    spinStartTime = currentMillis;
                }
            } else {
                if (currentMillis - spinStartTime >= spinDuration) {
                    stopHeadspin();
                    headSpinning = false;
                    scheduleNextSpin();
                }
            }
            
            // Check for button press to transition to travelling state
            if(buttonPressed) {
                if(headSpinning) {
                    stopHeadspin();
                }
                currentState = STATE_TRAVELLING;
                lastMillis = currentMillis;  // Reset timing when entering travelling state
                buttonPressed = false;
            }
            break;
            
        case STATE_TRAVELLING:
            if (currentMillis - lastMillis < 4000) {  // First 4 seconds: drive forward
                setDifferentialDriveSpeed(-80, 0);
            } else if (currentMillis - lastMillis < 5150) {  // Next 1.15 seconds: turn
                lcd.setCursor(0,0);
                lcd.print("SPINNING");
                setDifferentialDriveSpeed(0, -600);
            } else {  // After 5.15 seconds total: move to next state
                currentState = STATE_DISPENSING;
            }
            break;
            
        case STATE_DISPENSING:
            setDifferentialDriveSpeed(0, 0);
            lcd.setCursor(0,0);
            lcd.print("WAITING FOR CUP");
            
            // Wait for cup detection
            if (!waitForCupDetection()) {
                lcd.setCursor(0,0);
                lcd.print("NO CUP DETECTED");
                delay(2000);
                currentState = STATE_IDLE;
                break;
            }
            
            // Move servo to dispensing position
            if (servoPosition < 180) {
                servoPosition++;
                myservo.write(servoPosition);
                delay(SERVO_MOVE_DELAY);
            } else {
                // Activate pump
                lcd.setCursor(0,0);
                lcd.print("POURING           ");
                digitalWrite(PUMP_PIN, HIGH);
                delay(PUMP_DURATION);  // Blocking delay for pump
                digitalWrite(PUMP_PIN, LOW);
                
                // Return servo to home position
                if (servoPosition > 0) {
                    servoPosition--;
                    myservo.write(servoPosition);
                    delay(SERVO_MOVE_DELAY);
                } else {
                    currentState = STATE_RETURNING;
                    lastMillis = currentMillis;  // Reset timing for returning state
                }
            }
            break;
            
        case STATE_RETURNING:
            if (currentMillis - lastMillis < 800) {  // First 800ms: turn
                setDifferentialDriveSpeed(0, -600);
            } else if (currentMillis - lastMillis < 4600) {  // Next 3800ms: drive forward
                lcd.setCursor(0,0);
                lcd.print("GOING HOME");
                setDifferentialDriveSpeed(-80, 0);
            } else if (currentMillis - lastMillis < 6600) {  // Next 2000ms: turn
                setDifferentialDriveSpeed(0, 600);
            } else {  // After 6.6 seconds total: move to next state
                setDifferentialDriveSpeed(0, 0);
                lcd.setCursor(0,0);
                lcd.print("HOME");
                currentState = STATE_IDLE;
            }
            break;
            
        case STATE_ERROR:
            // Stay in error state until system is reset
            break;
    }
}

// Motor Control Functions
void setDifferentialDriveSpeed(float v, float omega) {
    // Safety checks for input ranges
    // Limit linear velocity to reasonable range (-100 to 100)
    v = constrain(v, -100, 100);
    
    // Limit angular velocity to reasonable range (-1000 to 1000)
    omega = constrain(omega, -1000, 1000);
    
    // Compute wheel speeds using differential drive kinematics
    float v_left = v - (omega * WHEEL_BASE / 2);
    float v_right = v + (omega * WHEEL_BASE / 2);
    
    // Convert to motor speed (assumes linear mapping)
    int speed_left = (int)(v_left);
    int speed_right = (int)(v_right);
    
    Serial.write(CMD);
    Serial.write(SET_SPEED1);
    Serial.write(speed_left);
    
    Serial.write(CMD);
    Serial.write(SET_SPEED2);
    Serial.write(speed_right);
}

// Head Spinning Control Functions
void startHeadspin() {
    if(clockwise) {
        digitalWrite(HEADSPIN_PIN1, HIGH);
        digitalWrite(HEADSPIN_PIN2, LOW);
        clockwise = false;
    } else {
        digitalWrite(HEADSPIN_PIN2, HIGH);
        digitalWrite(HEADSPIN_PIN1, LOW);
        clockwise = true;
    }
    Serial.println("Headspin started!");
}

void stopHeadspin() {
    digitalWrite(HEADSPIN_PIN1, LOW);
    digitalWrite(HEADSPIN_PIN2, LOW);
    Serial.println("Headspin stopped!");
}

void scheduleNextSpin() {
    previousSpinMillis = millis();
    spinInterval = random(10000, 15001); // 10-15 seconds
}
