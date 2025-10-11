/*
 * Buzzer + GPIO Library Integration Example
 * 
 * This example demonstrates how to use the BUZZER library 
 * with the GPIO library without Timer1 conflicts.
 * 
 * Hardware:
 * - Passive buzzer connected to PC4 (PIN_7) 
 * - LED connected to PC1 (PIN_5) for visual feedback
 * 
 * The buzzer library now works within the GPIO library's
 * 8-bit PWM framework to avoid timer conflicts.
 */

#include "ch32v003fun.h"
#include "gpios.h"
#include "buzzer.h"

int main() {
    SystemInit();
    
    // Initialize GPIO library first
    gpio_init();
    
    // Initialize LED on PC1 for visual feedback
    pinMode(PC1, OUTPUT);
    digitalWrite(PC1, LOW);
    
    // Initialize buzzer on PC4 (passive buzzer)
    Buzzer buzzer;
    uint8_t result = buzzer_init(&buzzer, PC4, BUZZER_PASSIVE);
    
    if (result != BUZZER_OK) {
        // Error handling - blink LED rapidly if initialization failed
        while(1) {
            digitalWrite(PC1, HIGH);
            Delay_Ms(100);
            digitalWrite(PC1, LOW);
            Delay_Ms(100);
        }
    }
    
    // Success - LED on for 1 second
    digitalWrite(PC1, HIGH);
    Delay_Ms(1000);
    digitalWrite(PC1, LOW);
    
    while(1) {
        // Musical scale demonstration
        uint16_t notes[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5};
        uint32_t durations[] = {500, 500, 500, 500, 500, 500, 500, 1000};
        
        // Visual feedback during melody
        digitalWrite(PC1, HIGH);
        buzzer_playMelody(&buzzer, notes, durations, 8);
        digitalWrite(PC1, LOW);
        
        Delay_Ms(2000);
        
        // Individual tone test - 440Hz (A4) for 1 second
        digitalWrite(PC1, HIGH);
        buzzer_tone(&buzzer, NOTE_A4, 1000);
        digitalWrite(PC1, LOW);
        
        Delay_Ms(1000);
        
        // Beep test - 2kHz for 500ms
        digitalWrite(PC1, HIGH);
        buzzer_beep(&buzzer, 500);
        digitalWrite(PC1, LOW);
        
        Delay_Ms(2000);
    }
    
    return 0;
}