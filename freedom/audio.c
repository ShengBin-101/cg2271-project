#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"

#define BUZZER_PIN 0

enum note_t { C6, D6, E6, F6, G6, A6, B6, REST};

// Delay Function
static void delay(volatile uint32_t nof) {
	while(nof!=0) {
		__asm("NOP");
	nof--;
	}
}

void initPWM(void) {
	// Enable clk to PORTB
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// Set pins 0 and 1 to be connected to the TPM module
	PORTB->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK; // clear current MUX
	PORTB->PCR[BUZZER_PIN] |= PORT_PCR_MUX(3); // config MUX to 3 (timer/PWM)
	
	// (pg 208) Powers on (enable clock for) TPM1 module
	// TPM = Timer or PWM module
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	// (pg 195) Reset current state of the mask
	// TPMSRC = TPM source
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	// Configures TPM to use MCG FLL clock (pg 367)
	// MCG = Main Clock Generator
	// PLL = Phase Locked Loop
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	// (pg 553) Reset clock mode and prescaler
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	// set clock mode to increment on internal clock
	// prescaler = 128 aka 2^7
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	// (pg 553) Set to 0 --> Up counting mode --> Edge aligned PWM
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// (pg 556) Reset old masks
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) |
		(TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	// Set edge aligned PWM, high true pulses	
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

int frequency_to_MOD(int freq) {
	// fPWM = fTPM / (MOD + 1) = 48MHz / ps / (MOD + 1)
    return (int)((48000000.0 / (128.0 * freq)) - 1);
}

void play_note(enum note_t note) {
	int frequency = 0;
	switch (note) {
		case C6: frequency = 1046; break;
		case D6: frequency = 1174; break;
		case E6: frequency = 1318; break;
		case F6: frequency = 1396; break;
		case G6: frequency = 1568; break;
		case A6: frequency = 1760; break;
		case B6: frequency = 1975; break;
		case REST: frequency = 0; break;
		default: break;
	}

	TPM1->MOD = frequency_to_MOD(frequency);
	TPM1_C0V = frequency_to_MOD(frequency) * 0.9;

//	delay(1000000);  // Hold the note for the specified duration
	osDelay(400);
	TPM1->MOD = 0;    // Silence between notes
//	delay(10000);  // Short gap between notes
	osDelay(40);


}

// "Mary Had a Little Lamb" melody
const enum note_t melody[] = {
	E6, D6, C6, D6, E6, E6, E6,
	D6, D6, D6, E6, G6, G6,
	E6, D6, C6, D6, E6, E6, E6, 
	E6, D6, D6, E6, D6, C6
};

void play_tune(const enum note_t *melody, int length) {
	for (int i = 0; i < length; i++) {
		play_note(melody[i]);
	}
}

void tAudio(void *argument) {
	while (1) {
		play_tune(melody, sizeof(melody) / sizeof(melody[0]));
		osDelay(1000);  // Pause before repeating the song
	}
}

 /*----------------------------------------------------------------------------
  * Application main thread
  *---------------------------------------------------------------------------*/

int main(void) {
	SystemCoreClockUpdate();
	initPWM();
	osKernelInitialize(); // Initialize the RTOS kernel
	osThreadNew(tAudio, NULL, NULL); // Create music task
	osKernelStart(); // Start the RTOS scheduler

  for (;;) {}
}
