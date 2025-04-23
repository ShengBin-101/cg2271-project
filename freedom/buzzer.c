#include "buzzer.h"

// Thread flag variable
static uint32_t playingFlag = 0;

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


// Initialize TPM1 channel 0 for edge-aligned PWM
void initBuzzerPWM(void) {
    // Enable clock for PORTB
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    // Mux BUZZER_PIN to TPM1_CH0
    PORTB->PCR[BUZZER_PIN] = PORT_PCR_MUX(3);

    // Enable TPM1 clock, select MCGFLLCLK
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
    SIM->SOPT2  = (SIM->SOPT2 & ~SIM_SOPT2_TPMSRC_MASK) | SIM_SOPT2_TPMSRC(1);

    // Edge-aligned PWM, prescaler=128
    TPM1->SC  = TPM_SC_CMOD(1) | TPM_SC_PS(7);
    TPM1_C0SC = TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1);
}

// Convert frequency (Hz) to TPM1->MOD register value
static int frequency_to_MOD(int freq) {
    return (int)((48000000.0 / (128.0 * freq)) - 1);
}

// Map note to frequency in Hz
static int note_to_frequency(enum note_t note) {
    switch (note) {
        case C4:  return 261;
        case Db4: return 277;
        case D4:  return 293;
        case Eb4: return 311;
        case E4:  return 329;
        case F4:  return 349;
        case Gb4: return 370;
        case G4:  return 392;
        case Ab4: return 415;
        case A4:  return 440;
        case Bb4: return 466;
        case B4:  return 493;
        case C5:  return 523;
        case Db5: return 554;
        case D5:  return 587;
        case Eb5: return 622;
        case E5:  return 659;
        case F5:  return 698;
        case Gb5: return 740;
        case G5:  return 784;
        case Ab5: return 831;
        case A5:  return 880;
        case Bb5: return 932;
        case B5:  return 987;
        case C6:  return 1046;
        case Db6: return 1108;
        case D6:  return 1174;
        case Eb6: return 1244;
        case E6:  return 1318;
        case F6:  return 1396;
        case Gb6: return 1480;
        case G6:  return 1568;
        case Ab6: return 1661;
        case A6:  return 1760;
        case Bb6: return 1864;
        case B6:  return 1975;
        case REST: return 0;
    }
    return 0;
}

// Play one note by setting MOD and C0V
void play_note(enum note_t note) {
    int f   = note_to_frequency(note);
    int mod = frequency_to_MOD(f);
    TPM1->MOD  = mod;
    TPM1_C0V   = (uint16_t)(mod * VOLUME);
}

// Play through a melody; abort if thread flags change
void play_tune(const note_with_duration_t *melody, int length) {
    for (int i = 0; i < length; i++) {
        if (osThreadFlagsGet() != playingFlag) break;
        play_note(melody[i].note);
        osDelay(melody[i].duration);
        TPM1->MOD = 0;    // silence
        osDelay(PAUSE_T);
    }
}

// Audio thread: waits for 0x0001 or 0x0002, then plays melody1 or melody2
void tAudio(void *argument) {
    for (;;) {
        playingFlag = osThreadFlagsWait(0x0001 | 0x0002,
                                        osFlagsWaitAny, osWaitForever);
        if (playingFlag == 0x0001) {
            play_tune(melody1, sizeof(melody1)/sizeof(*melody1));
        } else {
            play_tune(melody2, sizeof(melody2)/sizeof(*melody2));
        }
        osDelay(END_T);
    }
}
