#include "buzzer.h"

// "Tune 1" melody (when endRun is false)
// "Mary Had a Little Lamb" melody
const enum note_t melody1[] = {
	G6,G6,G6,G6,G6,G6,G6, REST
};
	/*
	E6, D6, C6, D6, E6, E6, E6,
	D6, D6, D6, E6, G6, G6,
	E6, D6, C6, D6, E6, E6, E6, 
	E6, D6, D6, E6, D6, C6, REST
};*/

// "Tune 2" melody (when endRun is true)
const enum note_t melody2[] = {
    C6,C6,C6,C6,C6,C6, REST
};

void initBuzzerPWM(void) {
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
	// freq = fTPM / (MOD + 1) = (48MHz / PS) / (MOD + 1)
	// MOD = 48MHz / (PS * freq) - 1
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
	TPM1_C0V = frequency_to_MOD(frequency) * VOLUME;

	osDelay(NOTE_T);
	TPM1->MOD = 0;    // Silence between notes
	osDelay(PAUSE_T);
}

uint32_t flags = 0; // Initialize flags variable

void play_tune(const enum note_t *melody, int length) {
	uint32_t prev_flag = osThreadFlagsWait(0x0001 | 0x0002, osFlagsWaitAny, osWaitForever);
	for (int i = 0; i < length; i++) {
		play_note(melody[i]);
		if(prev_flag != osThreadFlagsWait(0x0001 | 0x0002, osFlagsWaitAny, osWaitForever))
			break;

	}
}

void tAudio(void *argument) {
	for (;;){
				// Wait for the thread flag to be set
    uint32_t flags = osThreadFlagsWait(0x0001 | 0x0002, osFlagsWaitAny, osWaitForever);

		// Play the appropriate tune based on the value of endRun
		if (flags & 0x0001) {
			play_tune(melody2, sizeof(melody2) / sizeof(melody2[0])); // Play Tune 2
		} else {
			play_tune(melody1, sizeof(melody1) / sizeof(melody1[0])); // Play Tune 1
		}
        // // Play the tune
        // play_tune(melody, sizeof(melody) / sizeof(melody[0]));
//		osDelay(END_T);
    
	}
}