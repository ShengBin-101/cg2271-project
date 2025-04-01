#ifndef BUZZER_H
#define BUZZER_H

#include "MKL25Z4.h"
#include "cmsis_os2.h"

#define BUZZER_PIN 0


#define VOLUME 0.9

enum note_t { C6, D6, E6, F6, G6, A6, B6, REST };

#define NOTE_T 500
#define PAUSE_T 100
#define END_T 1000


void initBuzzerPWM(void);
int frequency_to_MOD(int freq);
void play_note(enum note_t note);
void play_tune(const enum note_t *melody, int length);
void tAudio(void *argument);

#endif // BUZZER_H
