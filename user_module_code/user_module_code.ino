#include <Arduino.h>
#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define BUTTON1_PIN  12  // Adjust to your wiring
#define BUTTON2_PIN  14  // Adjust to your wiring
#define POT_PIN      34  // ADC pin for potentiometer
#define LED_PIN      27  // LED strip data pin
#define NUM_LEDS     30  // Adjust based on your LED strip

CRGB leds[NUM_LEDS];
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "abcdef01-1234-5678-1234-56789abcdef0"

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(POT_PIN, INPUT);

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    FastLED.show();

    // Initialize BLE
    BLEDevice::init("ESP32C_UserModule");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ   |
        BLECharacteristic::PROPERTY_WRITE  |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    pServer->getAdvertising()->start();
}

void loop() {
    int button1State = digitalRead(BUTTON1_PIN);
    int button2State = digitalRead(BUTTON2_PIN);
    int potValue = analogRead(POT_PIN);
    int ledCount = map(potValue, 0, 4095, 0, NUM_LEDS);
    
    // Update LED strip
    for (int i = 0; i < NUM_LEDS; i++) {
        if (i < ledCount) {
            leds[i] = CRGB::Blue;
        } else {
            leds[i] = CRGB::Black;
        }
    }
    FastLED.show();
    
    // Transmit data via BLE
    if (deviceConnected) {
        String data = String(button1State) + "," + String(button2State) + "," + String(potValue);
        pCharacteristic->setValue(data.c_str());
        pCharacteristic->notify();
    }
    
    delay(100);
}
