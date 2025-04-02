#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE Service and Characteristic UUIDs (must match Arduino side)
#define BLE_SERVICE_UUID              "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_CHARACTERISTIC_UUID       "19B10001-E8F2-537E-4F6C-D104768A1214"

// BLE Commands (must match Arduino side)
#define CMD_INTERRUPT                 0x01
#define CMD_GET_POT                  0x02
#define CMD_POT_VALUE                0x03

// Pin Definitions
const int POT_PIN = 34;  // Analog pin for potentiometer
const int INTERRUPT_BUTTON = 0;  // GPIO pin for interrupt button

// BLE Objects
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Callback class for BLE connection events
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

// Callback class for BLE characteristic events
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            uint8_t cmd = value[0];
            
            // Handle commands from Arduino
            switch (cmd) {
                case CMD_GET_POT:
                    // Read potentiometer and send value
                    uint16_t potValue = analogRead(POT_PIN);
                    uint8_t response[3] = {CMD_POT_VALUE, (uint8_t)(potValue >> 8), (uint8_t)(potValue & 0xFF)};
                    pCharacteristic->setValue(response, 3);
                    pCharacteristic->notify();
                    break;
            }
        }
    }
};

void setup() {
    Serial.begin(115200);
    
    // Initialize potentiometer pin
    pinMode(POT_PIN, INPUT);
    
    // Initialize interrupt button
    pinMode(INTERRUPT_BUTTON, INPUT_PULLUP);
    
    // Initialize BLE
    BLEDevice::init("R2D2_Controller");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    // Create BLE Service
    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
    
    // Create BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Device Ready");
}

void loop() {
    // Handle button press for interrupt
    if (digitalRead(INTERRUPT_BUTTON) == LOW) {
        if (deviceConnected) {
            uint8_t interruptCmd = CMD_INTERRUPT;
            pCharacteristic->setValue(&interruptCmd, 1);
            pCharacteristic->notify();
            delay(200);  // Debounce delay
        }
    }
    
    // Handle device disconnection
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // Give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // Restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    
    delay(10); // Small delay to prevent overwhelming the BLE stack
} 