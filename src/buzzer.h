// buzzer.h
#ifndef BUZZER_H
#define BUZZER_H

#include <stddef.h>
#include <stdbool.h>

#include "ch32v003fun.h"
#include "gpios.h"  // Include GPIO library for pin reservation

// Error codes
#define BUZZER_OK                0  // Success
#define BUZZER_ERROR_PIN_RESERVED 1  // Pin already reserved by another library
#define BUZZER_ERROR_INVALID_PIN  2  // Invalid pin specified
#define BUZZER_ERROR_INVALID_FREQ 3  // Invalid frequency specified
#define BUZZER_ERROR_NOT_INIT     4  // Buzzer not initialized
#define BUZZER_ERROR_TIMER_CONFLICT 5  // Timer already in use
#define BUZZER_ERROR_INVALID_TYPE 6  // Invalid buzzer type

// Buzzer types
typedef enum {
    BUZZER_ACTIVE,  // Simple on/off buzzer (any GPIO pin)
    BUZZER_PASSIVE  // PWM-controlled buzzer (PWM-capable pins only)
} BuzzerType;

// Musical notes frequencies (Hz)
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523

typedef struct {
    BuzzerType type;        // Buzzer type (active or passive)
    uint8_t gpio_pin;       // GPIO pin used
    TIM_TypeDef* timer;     // Timer used (for passive buzzers)
    uint8_t timer_channel;  // Timer channel used
    bool initialized;       // Initialization status
    uint16_t current_freq;  // Current frequency (for passive buzzers)
} Buzzer;

// Function prototypes
uint8_t buzzer_init(Buzzer* buzzer, uint8_t gpio_pin, BuzzerType type);
void buzzer_deinit(Buzzer* buzzer);
uint8_t buzzer_beep(Buzzer* buzzer, uint32_t duration_ms);
uint8_t buzzer_tone(Buzzer* buzzer, uint16_t frequency, uint32_t duration_ms);
uint8_t buzzer_stop(Buzzer* buzzer);
bool buzzer_isInitialized(Buzzer* buzzer);
uint8_t buzzer_validatePin(uint8_t gpio_pin, BuzzerType type);
uint8_t buzzer_playMelody(Buzzer* buzzer, const uint16_t* frequencies, const uint32_t* durations, uint8_t length);
void buzzer_setVolume(Buzzer* buzzer, uint8_t volume);  // For passive buzzers (duty cycle)

#endif  // BUZZER_H
