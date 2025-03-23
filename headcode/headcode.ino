#include <FastLED.h>

// LED Strip Configuration
#define NUM_LEDS 11
#define DATA_PIN_1 10  // Red strip
#define DATA_PIN_2 11  // Green strip
#define DATA_PIN_3 12  // Blue strip

CRGB leds1[NUM_LEDS];  // Red strip
CRGB leds2[NUM_LEDS];  // Green strip
CRGB leds3[NUM_LEDS];  // Blue strip

uint8_t position = 0;  // Position of the chasing pixel

// Speaker Configuration
#define SPEAKER_PIN 6

// Timing Variables
unsigned long lastLEDUpdate = 0;
unsigned long lastToneTime = 0;
unsigned long lastPhraseStartTime = 0;

const unsigned long LED_UPDATE_INTERVAL = 50; // Smooth LED update timing
unsigned long phraseDelay = 0;
unsigned long toneDelay = 0;

int k = 0;  // Base frequency
int i = 0;  // Tone counter
bool phraseActive = false;
bool playingPhrase1 = false;
bool playingPhrase2 = false;

// === LED Animation Function ===
void updateLEDs() {
    if (millis() - lastLEDUpdate >= LED_UPDATE_INTERVAL) {
        lastLEDUpdate = millis();

        // Adjusted pulsing effect for red & green
        uint8_t pulse = sin8(millis() / 10);
        uint8_t min_brightness = 80;
        uint8_t red_brightness = map(pulse, 0, 255, min_brightness, 255);

        fill_solid(leds1, NUM_LEDS, CHSV(0, 255, red_brightness)); // Red pulsing

        // Chasing blue effect
        fill_solid(leds3, NUM_LEDS, CRGB(24, 24, 24)); // Background glow
        leds3[position] = CRGB(0, 0, 255);
        position = (position + 1) % NUM_LEDS;

        FastLED.show();
    }
}

// === Sound Phrase Functions ===
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

// === Updates Sound Output (Non-Blocking) ===
void updatePhrase() {
    if (!phraseActive) return;

    if (millis() - lastToneTime >= toneDelay) {
        if (playingPhrase1) {
            if (i <= phraseDelay) {
                tone(SPEAKER_PIN, k + (-i * 2)); // Descending tone
                i++;
            } else {
                playingPhrase1 = false;
            }
        } else if (playingPhrase2) {
            if (i <= phraseDelay) {
                tone(SPEAKER_PIN, k + (i * 10)); // Ascending tone
                i++;
            } else {
                playingPhrase2 = false;
            }
        }

        lastToneTime = millis();
        toneDelay = random(1, 3);
    }

    if (!playingPhrase1 && !playingPhrase2) {
        phraseActive = false;
        noTone(SPEAKER_PIN);
    }
}

// === Randomly Start Sound Sequences ===
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

// === Check If It's Time to Start a Random Phrase ===
void checkAndStartRandomPhrase() {
    unsigned long idleInterval = random(5000, 15000); // Idle trigger interval (5-15 sec)
    if (!phraseActive && millis() - lastPhraseStartTime > idleInterval) {
        startRandomSequence();
        lastPhraseStartTime = millis();
    }
}

// === External Trigger for Sound Phrase ===
void checkTriggerInput() {
    // Replace with actual trigger condition (e.g., button press)
    bool externalTrigger = false;  // Change this to true when an event happens

    if (externalTrigger && !phraseActive) {
        startPhrase1();
    }
}

// === Setup Function ===
void setup() {
    FastLED.addLeds<WS2812, DATA_PIN_1, GRB>(leds1, NUM_LEDS);
    FastLED.addLeds<WS2812, DATA_PIN_2, GRB>(leds2, NUM_LEDS);
    FastLED.addLeds<WS2812, DATA_PIN_3, GRB>(leds3, NUM_LEDS);

    fill_solid(leds2, NUM_LEDS, CRGB::Green); // Initialize Green LEDs

    pinMode(SPEAKER_PIN, OUTPUT);
    randomSeed(analogRead(0)); // Seed randomness
}

// === Main Loop ===
void loop() {
    updateLEDs();             // LED animations always run
    checkAndStartRandomPhrase(); // Random phrases trigger at intervals
    checkTriggerInput();      // Check for external trigger input
    updatePhrase();           // Play sound sequences without blocking LEDs
}
