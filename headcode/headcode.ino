#include <FastLED.h>

// Define the number of LEDs per strip
#define NUM_LEDS 11 

// Define data pins for each strip
#define DATA_PIN_1 10  // First LED strip
#define DATA_PIN_2 11  // Second LED strip
#define DATA_PIN_3 12 // Third LED strip

#define SPEAKER_PIN 6

// Define LED arrays
CRGB leds1[NUM_LEDS];  // Red strip
CRGB leds2[NUM_LEDS];  // Green strip
CRGB leds3[NUM_LEDS];  // Blue strip



uint8_t position = 0;  // Position of the chasing pixel

#include <Arduino.h>

#define SPEAKER_PIN 6

// Timing variables
unsigned long phraseStartTime = 0;
unsigned long lastToneTime = 0;
unsigned long lastLEDTime = 0;
unsigned long phraseDelay = 0;
unsigned long toneDelay = 0;
unsigned long ledDelay = 0;

int k = 0;  // Base frequency
int i = 0;  // Tone counter
bool phraseActive = false;
bool ledState = false;
bool playingPhrase1 = false;
bool playingPhrase2 = false;

void startPhrase1() {
    k = random(1000, 2000);
    phraseActive = true;
    playingPhrase1 = true;
    playingPhrase2 = false;
    i = 0;
    phraseDelay = random(100, 2000);
    lastToneTime = millis();
}

void startPhrase2() {
    k = random(1000, 2000);
    phraseActive = true;
    playingPhrase1 = false;
    playingPhrase2 = true;
    i = 0;
    phraseDelay = random(100, 2000);
    lastToneTime = millis();
}

void updatePhrase() {
    if (!phraseActive) return;

    if (playingPhrase1) {
        if (i <= phraseDelay) {
            if (millis() - lastToneTime >= toneDelay) {
                tone(SPEAKER_PIN, k + (-i * 2));
                toneDelay = random(1, 3);
                lastToneTime = millis();
                i++;
            }
        } else {
            playingPhrase1 = false;
            i = 0;
            phraseDelay = random(100, 1000);
            lastToneTime = millis();
        }
    } else if (!playingPhrase1 && i <= phraseDelay) {
        if (millis() - lastToneTime >= toneDelay) {
            tone(SPEAKER_PIN, k + (i * 10));
            toneDelay = random(1, 3);
            lastToneTime = millis();
            i++;
        }
    } else {
        phraseActive = false;
        noTone(SPEAKER_PIN);
    }
}

void startRandomSequence() {
    int choice = random(1, 7);
    switch (choice) {
        case 1: startPhrase1(); break;
        case 2: startPhrase2(); break;
        case 3: startPhrase1(); startPhrase2(); break;
        case 4: startPhrase1(); startPhrase2(); startPhrase1(); break;
        case 5: startPhrase1(); startPhrase2(); startPhrase1(); startPhrase2(); startPhrase1(); break;
        case 6: startPhrase2(); startPhrase1(); startPhrase2(); break;
    }
}


void setup() {
    FastLED.addLeds<WS2812, DATA_PIN_1, GRB>(leds1, NUM_LEDS);
    FastLED.addLeds<WS2812, DATA_PIN_2, GRB>(leds2, NUM_LEDS);
    FastLED.addLeds<WS2812, DATA_PIN_3, GRB>(leds3, NUM_LEDS);

    fill_solid(leds2, NUM_LEDS, CRGB::Green);

    pinMode(SPEAKER_PIN, OUTPUT);
    randomSeed(analogRead(0));
}




void loop() {
    // Adjusted pulsing effect for red & green
    uint8_t pulse = sin8(millis() / 10); // Smooth sine wave pulsing
    uint8_t min_brightness = 80;         // Set a minimum brightness level for red
    uint8_t red_brightness = map(pulse, 0, 255, min_brightness, 255); // Scale brightness

    fill_solid(leds1, NUM_LEDS, CHSV(0, 255, red_brightness));   // Red pulsing with min brightness

    // Chasing effect on blue strip with background glow
    fill_solid(leds3, NUM_LEDS, CRGB(24, 24, 24)); // White glow at 50% brightness
    leds3[position] = CRGB(0, 0, 255);  // Max brightness blue

    position = (position + 1) % NUM_LEDS; // Move to the next LED in the strip

    FastLED.show();

        if (!phraseActive) {
        startRandomSequence();
    }

    updatePhrase();
    
}

