#include "buzzer.h"

const note_with_duration_t melody1[] = {
  {Bb6, FULL}, {Ab6, FULL}, {Gb6, FULL}, {F6, FULL},
  {REST, HALF}, {F6, FULL}, {F6, FULL}, {F6, FULL}, {Eb6, FULL}, {REST, HALF},
  {F6, HALF}, {Ab6, HALF}, {Db6, HALF}, {Db6, FULL}, {Db6, HALF},
  {F6, HALF}, {F6, HALF}, {F6, HALF}, {F6, HALF}, {F6, HALF}, {F6, HALF}, {F6, HALF}, {Eb6, FULL},
  {REST, HALF}, {F6, HALF}, {Ab6, HALF}, {Db6, HALF}, {Db6, FULL}, {Db6, HALF}
};

const note_with_duration_t melody2[] = {
  {G4, FULL}, {C5, FULL}, {C5, HALF}, {D5, HALF}, {C5, HALF}, {B4, HALF}, {A4, FULL},
  {A4, FULL}, {A4, HALF}, {D5, HALF}, {D5, HALF}, {E5, HALF}, {D5, FULL},
  {B4, FULL}, {G4, FULL}, {G4, HALF}, {E5, HALF}, {E5, HALF}, {F5, HALF}, {E5, FULL}, {C5, FULL},
  {A4, FULL}, {G4, FULL}, {A4, FULL}, {D5, FULL}, {C5, FULL}, {REST, FULL}
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
    case C4: frequency = 261; break;
    case Db4: frequency = 277; break;
    case D4: frequency = 293; break;
    case Eb4: frequency = 311; break;
    case E4: frequency = 329; break;
    case F4: frequency = 349; break;
    case Gb4: frequency = 370; break;
    case G4: frequency = 392; break;
    case Ab4: frequency = 415; break;
    case A4: frequency = 440; break;
    case Bb4: frequency = 466; break;
    case B4: frequency = 493; break;
    
    case C5: frequency = 523; break;
    case Db5: frequency = 554; break;
    case D5: frequency = 587; break;
    case Eb5: frequency = 622; break;
    case E5: frequency = 659; break;
    case F5: frequency = 698; break;
    case Gb5: frequency = 740; break;
    case G5: frequency = 784; break;
    case Ab5: frequency = 831; break;
    case A5: frequency = 880; break;
    case Bb5: frequency = 932; break;
    case B5: frequency = 987; break;
    
    case C6: frequency = 1046; break;
    case Db6: frequency = 1108; break;
    case D6: frequency = 1174; break;
    case Eb6: frequency = 1244; break;
    case E6: frequency = 1318; break;
    case F6: frequency = 1396; break;
    case Gb6: frequency = 1480; break;
    case G6: frequency = 1568; break;
    case Ab6: frequency = 1661; break;
    case A6: frequency = 1760; break;
    case Bb6: frequency = 1864; break;
    case B6: frequency = 1975; break;

    case REST: frequency = 0; break;
  }

  TPM1->MOD = frequency_to_MOD(frequency);
  TPM1_C0V = frequency_to_MOD(frequency) * VOLUME;
}



uint32_t flags = 0; // Initialize flags variable

void play_tune(const note_with_duration_t *melody, int length) {
  
	for (int i = 0; i < length; i++) {
				if (osThreadFlagsGet() != flags) break;
			  play_note(melody[i].note);
        osDelay(melody[i].duration); // Use the note-specific duration
        TPM1->MOD = 0;      // Silence between notes
        osDelay(PAUSE_T);
    
			
   }
}



void tAudio(void *argument) {
    
	for (;;){
				// Wait for the thread flag to be set
		flags = osThreadFlagsWait(0x0001 | 0x0002, osFlagsWaitAny, osWaitForever);
		// Play the appropriate tune based on the value of endRun
		if (flags == 0x0001) {
			play_tune(melody1, sizeof(melody1) / sizeof(melody1[0])); // Play Tune 1
		} else {
			play_tune(melody2, sizeof(melody2) / sizeof(melody2[0])); // Play Tune 2
		}
	}
}