// BUZZER Library Example - Software PWM Approach
// This example demonstrates working buzzer control using software PWM
// Compatible with CH32V003J4M6 GPIO library - avoids Timer1 conflicts

#include "ch32v003fun.h"
#include "gpios.h"

#define BUZZER_PIN PIN_3  // PA2 (physical pin 3) - any GPIO pin works
#define BUTTON_PIN PIN_5  // PC1 (physical pin 5)

// Software PWM tone generation function
void playTone(uint8_t pin, uint32_t frequency, uint32_t duration_ms) {
    if (frequency == 0) {
        // Rest/silence
        digitalWrite(pin, LOW);
        Delay_Ms(duration_ms);
        return;
    }
    
    uint32_t period_us = 1000000 / frequency;  // Period in microseconds
    uint32_t half_period_us = period_us / 2;   // 50% duty cycle
    uint32_t total_cycles = (duration_ms * 1000) / period_us;
    
    for (uint32_t i = 0; i < total_cycles; i++) {
        digitalWrite(pin, HIGH);
        Delay_Us(half_period_us);
        digitalWrite(pin, LOW);
        Delay_Us(half_period_us);
    }
}

// Musical note frequencies (Hz)
#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_C5  523

int main() {
    SystemInit();
    gpio_init();
    
    // Initialize buzzer pin as output
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Initialize button with pullup
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Musical notes and durations
    uint16_t melody[] = {NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, 0};  // 0 = rest
    uint32_t durations[] = {300, 300, 300, 600, 200};
    uint8_t melody_length = 5;
    
    while (1) {
        // Check for button press (active low)
        if (digitalRead(BUTTON_PIN) == LOW) {
            // Debounce delay
            Delay_Ms(50);
            if (digitalRead(BUTTON_PIN) == LOW) {
                
                // Play simple beep (2kHz for 200ms)
                playTone(BUZZER_PIN, 2000, 200);
                Delay_Ms(100);  // Gap between beeps
                
                // Play different frequency (440Hz for 300ms)
                playTone(BUZZER_PIN, 440, 300);
                Delay_Ms(100);
                
                // Play melody
                for (uint8_t i = 0; i < melody_length; i++) {
                    playTone(BUZZER_PIN, melody[i], durations[i]);
                    Delay_Ms(50);  // Small gap between notes
                }
                
                // Wait for button release
                while (digitalRead(BUTTON_PIN) == LOW) {
                    Delay_Ms(10);
                }
            }
        }
        
        Delay_Ms(10);  // Main loop delay
    }
    
    return 0;
}