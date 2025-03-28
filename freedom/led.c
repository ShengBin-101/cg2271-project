

/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

//LED Pins
#define RED_PIN1 1 //PTA1
#define RED_PIN2 2 //PTA2
#define RED_PIN3 4 //PTD4
#define RED_PIN4 12 //PTA12
#define RED_PIN5 4 //PTA4
#define RED_PIN6 5 //PTA5 
#define RED_PIN7 8 //PTC8
#define RED_PIN8 9 //PTC9

#define GREEN_PIN1 7 //PTC7
#define GREEN_PIN2 0 //PTC0
#define GREEN_PIN3 3 //PTC3
#define GREEN_PIN4 4 //PTC4
#define GREEN_PIN5 5 //PTC5
#define GREEN_PIN6 6 //PTC6
#define GREEN_PIN7 10 //PTC10
#define GREEN_PIN8 11 //PTC11

#define MASK(x) (1 << (x))

osMessageQueueId_t red_led_message_queue, green_led_message_queue;

static void delay(volatile uint32_t nof) {
	while(nof != 0) {
		__asm("NOP");
		nof--;
	}
}

void initLED(void) {
	int PORTAPins[] = {RED_PIN1, RED_PIN2, RED_PIN4, RED_PIN5, RED_PIN6};
	int PORTCPins[] = {RED_PIN7, RED_PIN8, GREEN_PIN1, GREEN_PIN2, GREEN_PIN3, GREEN_PIN4, GREEN_PIN5, GREEN_PIN6, GREEN_PIN7, GREEN_PIN8};
	int PORTDPin = RED_PIN3;
	
	SIM->SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

	for (int i = 0; i < sizeof(PORTAPins) / sizeof(PORTAPins[0]); i++) {
		int pin = PORTAPins[i];
		PORTA->PCR[pin] &= ~PORT_PCR_MUX_MASK;
		PORTA->PCR[pin] |= PORT_PCR_MUX(1);
		PTA->PDDR |= MASK(pin);
		PTA->PCOR |= MASK(pin);
	}
	
	for (int i = 0; i < sizeof(PORTCPins) / sizeof(PORTCPins[0]); i++) {
		int pin = PORTCPins[i];
		PORTC->PCR[pin] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[pin] |= PORT_PCR_MUX(1);
		PTC->PDDR |= MASK(pin);
		PTC->PCOR |= MASK(pin);
	}
	
	PORTD->PCR[PORTDPin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PORTDPin] |= PORT_PCR_MUX(1);
	PTD->PDDR |= MASK(PORTDPin);
	PTD->PCOR |= MASK(PORTDPin);
}

int parse_robot_status(void) {
	return 1;
}

void red_led_thread (void *argument) {
    for (;;) {
		int isRunning;
		osMessageQueueGet(red_led_message_queue, &isRunning, NULL, osWaitForever);
		int delayTiming = isRunning ? 500 : 250;

        //switch on led
        uint32_t PORTAState = MASK(RED_PIN1) | MASK(RED_PIN2) | MASK(RED_PIN4) | MASK(RED_PIN5) | MASK(RED_PIN6);
		uint32_t PORTCState = MASK(RED_PIN7) | MASK(RED_PIN8);
		PTA->PSOR = PORTAState;
		PTC->PSOR = PORTCState;
		PTD->PSOR = MASK(RED_PIN3);
        osDelay(delayTiming);

        //switch off led
        PTA->PCOR = PORTAState;
		PTC->PCOR = PORTCState;
		PTD->PCOR = MASK(RED_PIN3);
		osDelay(delayTiming);
	}
}

void green_led_thread (void *argument) {
    for (;;) {
		int isRunning;
		osMessageQueueGet(green_led_message_queue, &isRunning, NULL, osWaitForever);
		if (isRunning) {
			int green_led_array[] = {GREEN_PIN1, GREEN_PIN2, GREEN_PIN3, GREEN_PIN4, GREEN_PIN5, GREEN_PIN6, GREEN_PIN7, GREEN_PIN8};
			for (int i = 0; i < sizeof(green_led_array) / sizeof(green_led_array[0]); i++) {
                int onPin = green_led_array[i];
                PTC->PSOR = MASK(onPin);
                int offPin = i == 0 ? green_led_array[7] : green_led_array[i - 1];
                PTC->PCOR = MASK(offPin);
                osDelay(500);
            }
		} else {
			uint32_t state = MASK(GREEN_PIN1) | MASK(GREEN_PIN2) | MASK(GREEN_PIN3)| MASK(GREEN_PIN4) | MASK(GREEN_PIN5) | MASK(GREEN_PIN6) | MASK(GREEN_PIN7) | MASK(GREEN_PIN8);
			PTC->PSOR = state;
		}
	}
}

void control_thread (void *argument) {
  for (;;) {	
	    int isRunning = 1;
		osMessageQueuePut(red_led_message_queue, &isRunning, NULL, 0);
		osMessageQueuePut(green_led_message_queue, &isRunning, NULL, 0);
	}
}
 
// int main (void) {
 
// 	initLED();
//     // System Initialization
//     SystemCoreClockUpdate();
//     // ...
 
//     osKernelInitialize();                 // Initialize CMSIS-RTOS    
// 	red_led_message_queue = osMessageQueueNew(1, sizeof(int), NULL);
// 	green_led_message_queue = osMessageQueueNew(1, sizeof(int), NULL);
// 	osThreadNew(control_thread, NULL, NULL);
// 	osThreadNew(green_led_thread, NULL, NULL);
// 	osThreadNew(red_led_thread, NULL, NULL);
//     osKernelStart();                      // Start thread execution
//     for (;;) {}
// }