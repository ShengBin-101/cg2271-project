#ifndef BUZZER_H
#define BUZZER_H

#include "MKL25Z4.h"
#include "cmsis_os2.h"

#define BUZZER_PIN 0


#define VOLUME 0.9

enum note_t { 
  C4, Db4, D4, Eb4, E4, F4, Gb4, G4, Ab4, A4, Bb4, B4,  // Octave 4
  C5, Db5, D5, Eb5, E5, F5, Gb5, G5, Ab5, A5, Bb5, B5,  // Octave 5
  C6, Db6, D6, Eb6, E6, F6, Gb6, G6, Ab6, A6, Bb6, B6,  // Octave 6
  REST
};
	
#define NOTE_T 500
#define PAUSE_T 50
#define END_T 1000


typedef struct {
    enum note_t note;
    uint32_t duration; // Duration in milliseconds (or your chosen unit)
} note_with_duration_t;

#define FULL 300
#define HALF 150


void initBuzzerPWM(void);
int frequency_to_MOD(int freq);
void play_note(enum note_t note);
void play_tune(const note_with_duration_t *melody, int length);
void tAudio(void *argument);

#endif // BUZZER_H