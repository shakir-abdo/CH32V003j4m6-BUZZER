// main.c
#include "buzzer.h"
#include "ch32v003fun.h"

Buzzer buzzer;

int main() {
    SystemInit();
    // Initialize buzzer on PC4
    buzzer_init(&buzzer, BUZZER_PASSIVE);

    while (1) {
        // Play a simple melody
        buzzer_tone(&buzzer, NOTE_C4, 50);
        Delay_Ms(1000);
    }
}
