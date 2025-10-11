# CH32V003 Buzzer Library

A simple buzzer library for CH32V003 microcontroller, supporting multiple approaches for buzzer control.

## Features

- Supports both active and passive buzzers
- Hardware PWM using TIM1 (may conflict with GPIO library)
- **RECOMMENDED:** Software PWM approach for reliable operation
- Includes predefined musical notes
- Simple API for beeping and tone generation

## ⚠️ IMPORTANT USAGE NOTE

**For reliable buzzer operation, use the software PWM approach demonstrated in `example/software_pwm_buzzer.c`**

The hardware PWM approach can conflict with the GPIO library's Timer1 usage. The software PWM method works with any GPIO pin and avoids timer conflicts.

## Hardware Configuration

### Software PWM Method (Recommended)
- Works with **any GPIO pin** (PA1, PA2, PC1, PC2, PC4, PD4)
- No timer conflicts with other libraries
- See `example/software_pwm_buzzer.c` for implementation

### Hardware PWM Method (Legacy)
- Uses **PC4** pin with TIM1_CH4 for PWM generation
- May conflict with GPIO library Timer1 usage
- For passive buzzers only

### Connections
- Connect one terminal of the buzzer to your chosen GPIO pin
- Connect the other terminal to ground

## Usage

```c
#include "buzzer.h"

int main() {
    SystemInit();

    Buzzer buzzer;

    // Initialize for passive buzzer
    buzzer_init(&buzzer, BUZZER_PASSIVE);

    // Play a tone (frequency in Hz, duration in milliseconds)
    buzzer_tone(&buzzer, NOTE_C4, 1000);  // Play C4 for 1 second

    // Simple beep
    buzzer_beep(&buzzer, 500);  // Beep for 500ms

    while(1) {
        // Your main loop
    }
}
```

## Musical Notes

The library includes predefined frequencies for musical notes:
- NOTE_C4 (262 Hz)
- NOTE_D4 (294 Hz)
- NOTE_E4 (330 Hz)
- NOTE_F4 (349 Hz)
- NOTE_G4 (392 Hz)
- NOTE_A4 (440 Hz)
- NOTE_B4 (494 Hz)
- NOTE_C5 (523 Hz)

## Limitations

- This library only works with PC4 pin
- Other pins are not supported due to specific timer configurations
- Maximum frequency is limited by the system clock

## License

This library is released under the MIT License.
