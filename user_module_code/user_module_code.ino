#include <FastLED.h>
#include <SoftwareSerial.h>

#define BUTTON1_PIN  52  // Adjust to your wiring
#define BUTTON2_PIN  48  // Adjust to your wiring

#define POT_PIN      A1  // ADC pin for potentiometer
#define LED_PIN      7  // LED strip data pin
#define NUM_LEDS     5  // Adjust based on your LED strip

SoftwareSerial BTSerial(10, 11);
CRGB leds[NUM_LEDS];
void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(POT_PIN, INPUT);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
}

void loop() {
  int button1State = digitalRead(BUTTON1_PIN);
  int button2State = digitalRead(BUTTON2_PIN);
  int potValue = 1022 - analogRead(POT_PIN);
  int ledCount;
  
  if (potValue < 200) {
    ledCount = 1; 
  } else if (potValue >= 200 && potValue < 820) {
    ledCount = 2;
  } else if (potValue >= 820 && potValue < 950) {
    ledCount = 3;
  } else if (potValue >= 950 && potValue < 1000) {
    ledCount = 4;
  } else {
    ledCount = 5;
  }

  if (button1State == HIGH) { // Button pressed
    Serial.println("Button 1 Pressed!");
  } else {
    Serial.println("Button 1 Not Pressed");
  }

  if (button2State == HIGH) { // Button pressed
    Serial.println("Button 2 Pressed!");
  } else {
    Serial.println("Button 2 Not Pressed");
  }

  Serial.println(potValue);

  for (int i = 0; i < NUM_LEDS; i++) {
        if (i < ledCount) {
            leds[i] = CRGB::Blue;
        } else {
            leds[i] = CRGB::Black;
        }
    }
    FastLED.show();

  String data = String(button1State) + "," + String(button2State) + "," + String(potValue);
  BTSerial.println(data);

  delay(50);
}