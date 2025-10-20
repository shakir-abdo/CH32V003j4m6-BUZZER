// buzzer.c
#include "buzzer.h"

#include <stdbool.h>

#define MCU_CORE_CLOCK FUNCONF_SYSTEM_CORE_CLOCK

// Static variables for timer management
static bool timer1_initialized = false;
static uint8_t timer1_channels_used = 0;  // Bitmask for channels in use

// Forward declarations
static uint8_t configure_active_buzzer(uint8_t gpio_pin);
static uint8_t configure_passive_buzzer(uint8_t gpio_pin,
                                        uint8_t* timer_channel);
static uint8_t timer1_init(void);
static void configure_timer_channel(uint8_t timer_channel);

// Validate GPIO pin for buzzer usage
uint8_t buzzer_validatePin(uint8_t gpio_pin, BuzzerType type) {
    // Check if pin is valid
    if (!isValidPin(gpio_pin)) {
        return BUZZER_ERROR_INVALID_PIN;
    }

    if (type == BUZZER_ACTIVE) {
        // Active buzzer can use any GPIO pin
        return BUZZER_OK;
    } else {
        // Passive buzzer needs PWM-capable pins
        // Use GPIO library's isPWMPin for validation
        if (!isPWMPin(gpio_pin)) {
            return BUZZER_ERROR_INVALID_PIN;
        }
        return BUZZER_OK;
    }
}  // Configure active buzzer (simple GPIO)
static uint8_t configure_active_buzzer(uint8_t gpio_pin) {
    // Configure pin as push-pull output
    uint8_t result = pinMode(gpio_pin, OUTPUT);
    if (result != GPIO_OK) {
        return BUZZER_ERROR_INVALID_PIN;
    }

    // Start with buzzer off
    digitalWrite(gpio_pin, LOW);

    return BUZZER_OK;
}

// Initialize Timer1 for PWM (compatible with GPIO library)
static uint8_t timer1_init(void) {
    if (timer1_initialized) {
        return BUZZER_OK;  // Already initialized
    }

    // Check if GPIO library has already initialized TIM1
    // If TIM1 is already running, use existing configuration
    if (TIM1->CTLR1 & TIM_CEN) {
        // Timer is already initialized by GPIO library
        // Don't reconfigure - work with existing 8-bit PWM setup
        timer1_initialized = true;
        return BUZZER_OK;
    }

    // Timer not initialized yet - set up compatible with GPIO library
    // Enable Timer1 clock
    RCC->APB2PCENR |= RCC_APB2Periph_TIM1;

    // Timer1 Configuration - compatible with GPIO library
    TIM1->PSC = 0;      // No prescaler (same as GPIO lib)
    TIM1->ATRLR = 255;  // 8-bit PWM resolution (same as GPIO lib)

    // Enable main output for advanced timer
    TIM1->BDTR |= TIM_MOE;

    // Start timer
    TIM1->CTLR1 |= TIM_CEN;

    timer1_initialized = true;
    return BUZZER_OK;
}

// Configure timer channel for PWM
static void configure_timer_channel(uint8_t timer_channel) {
    switch (timer_channel) {
        case 1:  // TIM1_CH1 (PD4)
            // Configure CH1 for PWM mode 1
            TIM1->CHCTLR1 &= ~(TIM_OC1M);
            TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;  // PWM mode 1
            TIM1->CHCTLR1 |= TIM_OC1PE;                // Preload enable
            TIM1->CCER |= TIM_CC1E;                    // Enable CH1 output
            TIM1->CH1CVR = 0;                          // Start with 0% duty
            timer1_channels_used |= (1 << 0);
            break;

        case 4:  // TIM1_CH4 (PC4)
            // Configure CH4 for PWM mode 1
            TIM1->CHCTLR2 &= ~(TIM_OC4M);
            TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;  // PWM mode 1
            TIM1->CHCTLR2 |= TIM_OC4PE;                // Preload enable
            TIM1->CCER |= TIM_CC4E;                    // Enable CH4 output
            TIM1->CH4CVR = 0;                          // Start with 0% duty
            timer1_channels_used |= (1 << 3);
            break;
    }
}

// Configure passive buzzer (PWM-based)
static uint8_t configure_passive_buzzer(uint8_t gpio_pin,
                                        uint8_t* timer_channel) {
    // Extract port/pin for TIM1 hardware PWM check
    uint8_t port = (gpio_pin >> 4) & 0x0F;
    uint8_t pin = gpio_pin & 0x0F;

    // Check if this is a TIM1-capable pin (PC4 or PD4)
    // These support hardware frequency generation
    if (port == 0x02 && pin == 4) {
        // PC4 = TIM1_CH4
        *timer_channel = 4;
        if (timer1_channels_used & (1 << 3)) {
            return BUZZER_ERROR_TIMER_CONFLICT;
        }

        // Initialize Timer1
        uint8_t result = timer1_init();
        if (result != BUZZER_OK) {
            return result;
        }

        // Configure GPIO for TIM1_CH4
        RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
        GPIOC->CFGLR &= ~(0xf << (4 * 4));
        GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF) << (4 * 4);

        // Configure timer channel
        configure_timer_channel(*timer_channel);
        timer1_channels_used |= (1 << 3);

    } else if (port == 0x03 && pin == 4) {
        // PD4 = TIM1_CH1 (but GPIO lib uses TIM2, so use software PWM)
        *timer_channel = 0;  // Mark as software PWM

        // Use GPIO library's PWM (TIM2-based)
        uint8_t result = pinMode(gpio_pin, OUTPUT);
        if (result != GPIO_OK) {
            return BUZZER_ERROR_INVALID_PIN;
        }

    } else {
        // Other PWM pins (PC1, PC2) - use software PWM via GPIO library
        *timer_channel = 0;  // Mark as software PWM

        uint8_t result = pinMode(gpio_pin, OUTPUT);
        if (result != GPIO_OK) {
            return BUZZER_ERROR_INVALID_PIN;
        }
    }

    return BUZZER_OK;
}

// Initialize buzzer
uint8_t buzzer_init(Buzzer* buzzer, uint8_t gpio_pin, BuzzerType type) {
    if (!buzzer) {
        return BUZZER_ERROR_INVALID_PIN;
    }

    // Clean up previous initialization
    if (buzzer->initialized) {
        buzzer_deinit(buzzer);
    }

    // Validate inputs
    if (type != BUZZER_ACTIVE && type != BUZZER_PASSIVE) {
        return BUZZER_ERROR_INVALID_TYPE;
    }

    uint8_t result = buzzer_validatePin(gpio_pin, type);
    if (result != BUZZER_OK) {
        return result;
    }

    // Try to reserve the GPIO pin
    if (reservePin(gpio_pin, "BUZZER") != GPIO_OK) {
        return BUZZER_ERROR_PIN_RESERVED;
    }

    // Configure based on buzzer type
    uint8_t timer_channel = 0;
    if (type == BUZZER_ACTIVE) {
        result = configure_active_buzzer(gpio_pin);
    } else {
        result = configure_passive_buzzer(gpio_pin, &timer_channel);
    }

    if (result != BUZZER_OK) {
        releasePin(gpio_pin);  // Release pin on failure
        return result;
    }

    // Initialize buzzer structure
    buzzer->type = type;
    buzzer->gpio_pin = gpio_pin;
    buzzer->timer = (type == BUZZER_PASSIVE) ? TIM1 : NULL;
    buzzer->timer_channel = timer_channel;
    buzzer->initialized = true;
    buzzer->current_freq = 0;

    return BUZZER_OK;
}

// Deinitialize buzzer and release resources
void buzzer_deinit(Buzzer* buzzer) {
    if (!buzzer || !buzzer->initialized) {
        return;
    }

    // Stop any ongoing sound
    buzzer_stop(buzzer);

    // Release timer channel if passive buzzer
    if (buzzer->type == BUZZER_PASSIVE) {
        switch (buzzer->timer_channel) {
            case 1:                                 // CH1
                TIM1->CCER &= ~TIM_CC1E;            // Disable CH1 output
                TIM1->CH1CVR = 0;                   // Clear compare value
                timer1_channels_used &= ~(1 << 0);  // Clear bit 0
                break;

            case 4:                                 // CH4
                TIM1->CCER &= ~TIM_CC4E;            // Disable CH4 output
                TIM1->CH4CVR = 0;                   // Clear compare value
                timer1_channels_used &= ~(1 << 3);  // Clear bit 3
                break;
        }

        // If no channels are in use, disable timer
        if (timer1_channels_used == 0 && timer1_initialized) {
            TIM1->CTLR1 &= ~TIM_CEN;                 // Stop timer
            TIM1->BDTR &= ~TIM_MOE;                  // Disable main output
            RCC->APB2PCENR &= ~RCC_APB2Periph_TIM1;  // Disable clock
            timer1_initialized = false;
        }
    }

    // Release reserved pin
    releasePin(buzzer->gpio_pin);

    // Reset buzzer structure
    buzzer->gpio_pin = 0;
    buzzer->timer = NULL;
    buzzer->timer_channel = 0;
    buzzer->initialized = false;
    buzzer->current_freq = 0;
}

// Check if buzzer is initialized
bool buzzer_isInitialized(Buzzer* buzzer) {
    return (buzzer && buzzer->initialized);
}

// Play a tone (frequency + duration)
uint8_t buzzer_tone(Buzzer* buzzer, uint16_t frequency, uint32_t duration_ms) {
    if (!buzzer || !buzzer->initialized) {
        return BUZZER_ERROR_NOT_INIT;
    }

    if (frequency == 0) {
        return BUZZER_ERROR_INVALID_FREQ;
    }

    if (buzzer->type == BUZZER_ACTIVE) {
        // Active buzzer: simple on/off with delay
        digitalWrite(buzzer->gpio_pin, HIGH);
        Delay_Ms(duration_ms);
        digitalWrite(buzzer->gpio_pin, LOW);
    } else {
        // Passive buzzer: Check if using hardware (TIM1) or software PWM

        if (buzzer->timer_channel == 4) {
            // Hardware PWM on PC4 (TIM1_CH4) - use PWM modulation
            uint32_t half_period_ms = 500 / frequency;
            if (half_period_ms == 0) half_period_ms = 1;
            uint32_t cycles = (duration_ms * frequency) / 1000;
            if (cycles == 0) cycles = 1;

            for (uint32_t i = 0; i < cycles; i++) {
                TIM1->CH4CVR = 128;  // 50% duty
                Delay_Ms(half_period_ms);
                TIM1->CH4CVR = 0;
                Delay_Ms(half_period_ms);
            }

        } else {
            // Software PWM for other pins (PIN_1, PC1, PC2, PD4 via TIM2)
            // Generate square wave using digitalWrite
            uint32_t half_period_us = 500000 / frequency;    // microseconds
            if (half_period_us < 100) half_period_us = 100;  // Min 100us

            uint32_t cycles = (duration_ms * frequency) / 1000;
            if (cycles == 0) cycles = 1;

            for (uint32_t i = 0; i < cycles; i++) {
                digitalWrite(buzzer->gpio_pin, HIGH);
                Delay_Us(half_period_us);
                digitalWrite(buzzer->gpio_pin, LOW);
                Delay_Us(half_period_us);
            }
        }

        buzzer->current_freq = frequency;
        buzzer_stop(buzzer);
    }

    return BUZZER_OK;
}

// Simple beep
uint8_t buzzer_beep(Buzzer* buzzer, uint32_t duration_ms) {
    if (!buzzer || !buzzer->initialized) {
        return BUZZER_ERROR_NOT_INIT;
    }

    if (buzzer->type == BUZZER_ACTIVE) {
        digitalWrite(buzzer->gpio_pin, HIGH);
        Delay_Ms(duration_ms);
        digitalWrite(buzzer->gpio_pin, LOW);
    } else {
        return buzzer_tone(buzzer, 2000, duration_ms);  // 2kHz beep
    }

    return BUZZER_OK;
}

// Stop buzzer
uint8_t buzzer_stop(Buzzer* buzzer) {
    if (!buzzer || !buzzer->initialized) {
        return BUZZER_ERROR_NOT_INIT;
    }

    if (buzzer->type == BUZZER_ACTIVE) {
        digitalWrite(buzzer->gpio_pin, LOW);
    } else {
        // Stop PWM - either hardware or software
        if (buzzer->timer_channel == 4) {
            // Hardware PWM on PC4
            TIM1->CH4CVR = 0;
        } else {
            // Software PWM - just set pin LOW
            digitalWrite(buzzer->gpio_pin, LOW);
        }
    }

    buzzer->current_freq = 0;
    return BUZZER_OK;
}

// Set volume for passive buzzers (duty cycle adjustment)
void buzzer_setVolume(Buzzer* buzzer, uint8_t volume) {
    if (!buzzer || !buzzer->initialized || buzzer->type != BUZZER_PASSIVE) {
        return;
    }

    if (volume > 100) volume = 100;  // Limit to 100%

    // Work within GPIO library's 8-bit PWM framework (ATRLR = 255)
    // Calculate duty cycle value (0-255) based on volume percentage
    uint32_t duty = (255 * volume) / 100;

    switch (buzzer->timer_channel) {
        case 1:  // CH1 (PD4)
            TIM1->CH1CVR = duty;
            break;
        case 4:  // CH4 (PC4)
            TIM1->CH4CVR = duty;
            break;
    }
}

// Play melody (sequence of tones)
uint8_t buzzer_playMelody(Buzzer* buzzer, const uint16_t* frequencies,
                          const uint32_t* durations, uint8_t length) {
    if (!buzzer || !buzzer->initialized) {
        return BUZZER_ERROR_NOT_INIT;
    }

    if (!frequencies || !durations || length == 0) {
        return BUZZER_ERROR_INVALID_FREQ;
    }

    for (uint8_t i = 0; i < length; i++) {
        if (frequencies[i] == 0) {
            // Rest (silence)
            buzzer_stop(buzzer);
            Delay_Ms(durations[i]);
        } else {
            uint8_t result = buzzer_tone(buzzer, frequencies[i], durations[i]);
            if (result != BUZZER_OK) {
                return result;
            }
        }

        // Small gap between notes
        Delay_Ms(50);
    }

    return BUZZER_OK;
}
