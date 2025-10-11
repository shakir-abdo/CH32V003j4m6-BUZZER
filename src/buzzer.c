// buzzer.c
#include "buzzer.h"
#include <stdbool.h>

#define MCU_CORE_CLOCK FUNCONF_SYSTEM_CORE_CLOCK

// Static variables for timer management
static bool timer1_initialized = false;
static uint8_t timer1_channels_used = 0;  // Bitmask for channels in use

// Forward declarations
static uint8_t configure_active_buzzer(uint8_t gpio_pin);
static uint8_t configure_passive_buzzer(uint8_t gpio_pin, uint8_t* timer_channel);
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
        // Passive buzzer needs PWM-capable pins (PC4 or PD4)
        if (gpio_pin != PC4 && gpio_pin != PD4) {
            return BUZZER_ERROR_INVALID_PIN;
        }
        return BUZZER_OK;
    }
}

// Configure active buzzer (simple GPIO)
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
    switch(timer_channel) {
        case 1:  // TIM1_CH1 (PD4)
            // Configure CH1 for PWM mode 1
            TIM1->CHCTLR1 &= ~(TIM_OC1M);
            TIM1->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1;  // PWM mode 1
            TIM1->CHCTLR1 |= TIM_OC1PE;                 // Preload enable
            TIM1->CCER |= TIM_CC1E;                     // Enable CH1 output
            TIM1->CH1CVR = 0;                           // Start with 0% duty
            timer1_channels_used |= (1 << 0);
            break;
            
        case 4:  // TIM1_CH4 (PC4)
            // Configure CH4 for PWM mode 1
            TIM1->CHCTLR2 &= ~(TIM_OC4M);
            TIM1->CHCTLR2 |= TIM_OC4M_2 | TIM_OC4M_1;  // PWM mode 1
            TIM1->CHCTLR2 |= TIM_OC4PE;                 // Preload enable
            TIM1->CCER |= TIM_CC4E;                     // Enable CH4 output
            TIM1->CH4CVR = 0;                           // Start with 0% duty
            timer1_channels_used |= (1 << 3);
            break;
    }
}

// Configure passive buzzer (PWM-based)
static uint8_t configure_passive_buzzer(uint8_t gpio_pin, uint8_t* timer_channel) {
    // Determine timer channel based on GPIO pin
    switch(gpio_pin) {
        case PC4:
            *timer_channel = 4;  // TIM1_CH4
            // Check if channel is already in use
            if (timer1_channels_used & (1 << 3)) {
                return BUZZER_ERROR_TIMER_CONFLICT;
            }
            break;
            
        case PD4:
            *timer_channel = 1;  // TIM1_CH1
            // Check if channel is already in use
            if (timer1_channels_used & (1 << 0)) {
                return BUZZER_ERROR_TIMER_CONFLICT;
            }
            break;
            
        default:
            return BUZZER_ERROR_INVALID_PIN;
    }
    
    // Initialize Timer1 if needed
    uint8_t result = timer1_init();
    if (result != BUZZER_OK) {
        return result;
    }
    
    // Use GPIO library's analogWrite to initialize PWM on the pin
    // This ensures compatibility with GPIO library's timer configuration
    uint8_t gpio_result = analogWrite(gpio_pin, 0);  // Start with 0% duty cycle
    if (gpio_result != GPIO_OK) {
        return BUZZER_ERROR_INVALID_PIN;
    }
    
    // Mark channel as used (analogWrite already configures the channel)
    timer1_channels_used |= (*timer_channel == 1) ? (1 << 0) : (1 << 3);
    
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
        switch(buzzer->timer_channel) {
            case 1:  // CH1
                TIM1->CCER &= ~TIM_CC1E;        // Disable CH1 output
                TIM1->CH1CVR = 0;               // Clear compare value
                timer1_channels_used &= ~(1 << 0);  // Clear bit 0
                break;
                
            case 4:  // CH4
                TIM1->CCER &= ~TIM_CC4E;        // Disable CH4 output
                TIM1->CH4CVR = 0;               // Clear compare value
                timer1_channels_used &= ~(1 << 3);  // Clear bit 3
                break;
        }
        
        // If no channels are in use, disable timer
        if (timer1_channels_used == 0 && timer1_initialized) {
            TIM1->CTLR1 &= ~TIM_CEN;  // Stop timer
            TIM1->BDTR &= ~TIM_MOE;   // Disable main output
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
        // Passive buzzer: Work within GPIO library's 8-bit PWM framework
        // GPIO library sets ATRLR = 255, so PWM frequency = 48MHz / 256 = 187.5kHz
        // We generate audio frequency by toggling PWM duty cycle at the desired rate
        
        // Calculate timing for software-generated frequency
        // Half period in milliseconds for square wave generation
        uint32_t half_period_ms = 500 / frequency;  // 500ms / freq gives half period
        if (half_period_ms == 0) half_period_ms = 1;  // Minimum 1ms
        
        uint32_t cycles = (duration_ms * frequency) / 1000;  // Number of complete cycles
        if (cycles == 0) cycles = 1;
        
        // Generate square wave by alternating between 50% and 0% duty cycle
        for (uint32_t i = 0; i < cycles; i++) {
            // High phase - 50% duty cycle (128 out of 255)
            switch(buzzer->timer_channel) {
                case 1:  // CH1 (PD4)
                    TIM1->CH1CVR = 128;
                    break;
                case 4:  // CH4 (PC4)
                    TIM1->CH4CVR = 128;
                    break;
            }
            Delay_Ms(half_period_ms);
            
            // Low phase - 0% duty cycle
            switch(buzzer->timer_channel) {
                case 1:  // CH1 (PD4)
                    TIM1->CH1CVR = 0;
                    break;
                case 4:  // CH4 (PC4)
                    TIM1->CH4CVR = 0;
                    break;
            }
            Delay_Ms(half_period_ms);
        }
        
        buzzer->current_freq = frequency;
        
        // Ensure buzzer is stopped after tone
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
        // Set duty cycle to 0
        switch(buzzer->timer_channel) {
            case 1:  // CH1
                TIM1->CH1CVR = 0;
                break;
            case 4:  // CH4
                TIM1->CH4CVR = 0;
                break;
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
    
    switch(buzzer->timer_channel) {
        case 1:  // CH1 (PD4)
            TIM1->CH1CVR = duty;
            break;
        case 4:  // CH4 (PC4)
            TIM1->CH4CVR = duty;
            break;
    }
}

// Play melody (sequence of tones)
uint8_t buzzer_playMelody(Buzzer* buzzer, const uint16_t* frequencies, const uint32_t* durations, uint8_t length) {
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