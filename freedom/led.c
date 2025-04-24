// led.c
// Initializes GPIO pins for red/green LEDs and implements RTOS threads

#include "led.h"
#include "cmsis_os2.h"
#include "MKL25Z4.h"

extern osMessageQueueId_t red_led_message_queue;
extern osMessageQueueId_t green_led_message_queue;

// Arrays of pin numbers
static const int redPinsA[]  = { RED_PIN1, RED_PIN2, RED_PIN4, RED_PIN5, RED_PIN6 };
static const int redPinsC[]  = { RED_PIN7, RED_PIN8 };
static const int greenPins[] = {
    GREEN_PIN1, GREEN_PIN2, GREEN_PIN3, GREEN_PIN4,
    GREEN_PIN5, GREEN_PIN6, GREEN_PIN7, GREEN_PIN8
};

// Masks for fast on/off
#define RED_PORTA_MASK  (MASK(RED_PIN1) | MASK(RED_PIN2) | MASK(RED_PIN4) | MASK(RED_PIN5) | MASK(RED_PIN6))
#define RED_PORTC_MASK  (MASK(RED_PIN7) | MASK(RED_PIN8))
#define ALL_GREEN_MASK  (MASK(GREEN_PIN1) | MASK(GREEN_PIN2) | MASK(GREEN_PIN3) | MASK(GREEN_PIN4) | \
                         MASK(GREEN_PIN5) | MASK(GREEN_PIN6) | MASK(GREEN_PIN7) | MASK(GREEN_PIN8))

// Configure LED pins as GPIO outputs, start OFF
void initLED(void) {
    // Enable clocks for ports A, C, D
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    // RED LEDs on PORTA
    for (size_t i = 0; i < sizeof(redPinsA)/sizeof(redPinsA[0]); i++) {
        int p = redPinsA[i];
				PORTA->PCR[p] &= ~PORT_PCR_MUX_MASK;
        PORTA->PCR[p] |= PORT_PCR_MUX(1);
        PTA->PDDR     |= MASK(p);
        PTA->PCOR     |= MASK(p);
    }
    // RED LEDs on PORTC
    for (size_t i = 0; i < sizeof(redPinsC)/sizeof(redPinsC[0]); i++) {
        int p = redPinsC[i];
				PORTC->PCR[p] &= ~PORT_PCR_MUX_MASK;
        PORTC->PCR[p] |= PORT_PCR_MUX(1);
        PTC->PDDR     |= MASK(p);
        PTC->PCOR     |= MASK(p);
    }
    // RED LED on PTD
		PORTD->PCR[RED_PIN3] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[RED_PIN3] |= PORT_PCR_MUX(1);
    PTD->PDDR          |= MASK(RED_PIN3);
    PTD->PCOR          |= MASK(RED_PIN3);

    // GREEN LEDs on PORTC
    for (size_t i = 0; i < sizeof(greenPins)/sizeof(greenPins[0]); i++) {
        int p = greenPins[i];
				PORTC->PCR[p] &= ~PORT_PCR_MUX_MASK;
        PORTC->PCR[p] |= PORT_PCR_MUX(1);
        PTC->PDDR     |= MASK(p);
        PTC->PCOR     |= MASK(p);
    }
}

// Red LED thread: blinks at 500ms if running, 250ms otherwise
void red_led_thread(void *argument) {
    for (;;) {
        int isRunning = 0;
        osMessageQueueGet(red_led_message_queue, &isRunning, NULL, osWaitForever);

        // Turn on
        PTA->PSOR = RED_PORTA_MASK;
        PTC->PSOR = RED_PORTC_MASK;
        PTD->PSOR = MASK(RED_PIN3);
        osDelay(isRunning ? 500 : 250);

        // Turn off
        PTA->PCOR = RED_PORTA_MASK;
        PTC->PCOR = RED_PORTC_MASK;
        PTD->PCOR = MASK(RED_PIN3);
        osDelay(isRunning ? 500 : 250);
    }
}

// Green LED thread: rotating effect when running, solid on otherwise
void green_led_thread(void *argument) {
    size_t idx = 0;
    const size_t count = sizeof(greenPins)/sizeof(greenPins[0]);
    for (;;) {
        int isRunning = 0;
        osMessageQueueGet(green_led_message_queue, &isRunning, NULL, osWaitForever);

        if (isRunning) {
            // Turn all off, then one on
            PTC->PCOR = ALL_GREEN_MASK;
            PTC->PSOR = MASK(greenPins[idx]);
            idx = (idx + 1) % count;
            osDelay(50);
        } else {
            // Solid on
            PTC->PSOR = ALL_GREEN_MASK;
        }
    }
}

